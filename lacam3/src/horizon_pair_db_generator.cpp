#include "../include/horizon_pair_db_generator.hpp"
#include "../include/thread_pool.hpp"
#include "../include/drawing.hpp"
#include "../include/moves.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <thread>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>


// Number of steps ahead we are aware of
int HorizonPairDBGenerator::HORIZON = 5;

// MDD::frontiers[0] is the start vertex (time 0).
// This is why we add 1 to HORIZON.
// Given two MDDs aligned to the same time, we look up to frontiers[HORIZON + 1].
// When comparing time shifted MDDs, the shifted one needs to look forward an extra step.
// Thats why we add another 1.
int HorizonPairDBGenerator::MDD_SIZE = HorizonPairDBGenerator::HORIZON + 2;

static const int NUM_OF_THREADS = 8;

// Runs a dedicated thread that redraws the progress bar at a fixed rate while
// workers only bump the atomic counter. The calling thread waits for all
// futures (rethrowing any task exception) and then stops the printer.
template <typename T>
static void wait_with_progress(std::vector<std::future<void>>& futures, const std::atomic<T>& done, size_t total) {
  std::mutex mtx;
  std::condition_variable cv;
  bool stop = false;

  std::thread printer([&]() {
    std::unique_lock lock(mtx);
    while (!stop) {
      print_horizon_pair_db_progress_bar(done.load(), total);
      cv.wait_for(lock, std::chrono::milliseconds(100), [&]() { return stop; });
    }
  });

  auto stop_printer = [&]() {
    {
      std::lock_guard lock(mtx);
      stop = true;
    }
    cv.notify_one();
    printer.join();
  };

  try {
    for (auto& fut : futures) fut.get();
  } catch (...) {
    stop_printer();
    throw;
  }
  stop_printer();
  print_horizon_pair_db_progress_bar(total, total);
}


static DistTable* create_dist_table(Graph* G) {
  std::cout << "Creating the big DistTable" << std::endl;
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  std::cout << "Finished creating the big DistTable" << std::endl;
  return D;
}

HorizonPairDBGenerator::HorizonPairDBGenerator(Graph* _G, std::string _name)
    : G(_G), D(create_dist_table(G)), V_SIZE(_G->V.size()), name(_name) {}


void HorizonPairDBGenerator::generate_mdds() {
  size_t V_SIZE = G->V.size();

  // Pre-allocate master storage
  mdd_id_by_v_g.assign(V_SIZE * V_SIZE, 0);
  mdd_by_id.resize(V_SIZE * V_SIZE);
  mdd_by_t_s.assign(V_SIZE * (MDD_SIZE + 1), std::vector<uint32_t>());

  std::mutex mdd_count_mtx; // Only used to safely increment the global ID
  std::mutex mdd_mtx2;      // Protects push_back on mdd_by_t_s

  const long long total = (long long)V_SIZE * (long long)V_SIZE;
  std::atomic done = 0;


  // Task processes an ENTIRE start vertex (v_i)
  auto task = [&](int v_i) {
    // Completely thread-local map! No mutex needed to access this.
    absl::flat_hash_map<MDD, uint32_t> local_mdd_to_id;
    
    for (int g_i = 0; g_i < (int)V_SIZE; g_i++) {
      int agent_id = v_i * (int)V_SIZE + g_i;
      
      MDD mdd; 
      mdd.populate(D, G->V[v_i], G->V[g_i], MDD_SIZE);
      
      uint32_t mdd_id;
      bool is_new_mdd = false;

      auto it = local_mdd_to_id.find(mdd);
      if (it != local_mdd_to_id.end()) {
        mdd_id = it->second;
      } else {
        // 1. Get a globally unique ID
        {
          std::lock_guard lock(mdd_count_mtx);
          mdd_id = mdd_count++;
        }
        is_new_mdd = true;
        
        // 2. Lock-free write! mdd_by_id is pre-allocated and mdd_id is strictly unique.
        mdd_by_id[mdd_id] = mdd;
      }

      // Lock-free write! agent_id is strictly unique per loop iteration.
      mdd_id_by_v_g[agent_id] = mdd_id;

      if (is_new_mdd) {
        // Protect mdd_by_t_s because different v_i threads might hit the same space-time vertex
        std::lock_guard lock(mdd_mtx2);
        for (int t = 0; t < (int)mdd.frontiers.size(); t++) {
          for (Vertex* v : mdd.frontiers[t]) {
            int idx = t * (int)V_SIZE + v->id;
            mdd_by_t_s[idx].push_back(mdd_id); 
          }
        }
      }

      // 3. Emplace into the local map AFTER iterating frontiers so we don't move an empty MDD
      if (is_new_mdd) {
        local_mdd_to_id.emplace(std::move(mdd), mdd_id);
      }

      ++done;
    }
    
    // Update progress bar once per start vertex instead of every inner loop
  };

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<void>> futures;
  futures.reserve(V_SIZE);

  // Submit ONE task per start vertex
  for (int v_i = 0; v_i < (int)V_SIZE; v_i++) {
    futures.push_back(pool.submit([&task, v_i]() { task(v_i); }));
  }

  wait_with_progress(futures, done, total);
  std::cout << "\nNum of agents: " << V_SIZE * V_SIZE << std::endl;
  std::cout << "Num of MDDs  : " << mdd_count << std::endl;
}


uint8_t HorizonPairDBGenerator::calculate_sync_time_penalty(MDD& mdd1, MDD& mdd2) {
  if (!mdd1.check_joint_mdd_conflict(mdd2, G, HORIZON))
    return 0;
  if (!mdd1.check_joint_mdd_conflict(mdd2, G, HORIZON, 1, 0))
    return 1;
  if (!mdd1.check_joint_mdd_conflict(mdd2, G, HORIZON, 0, 1))
    return 1;
  return 2;
}


uint8_t HorizonPairDBGenerator::calculate_time_shifted_penalty(MDD& mdd_next, MDD& mdd_now) {
  const static int DT_NEXT = 0;
  const static int DT_NOW  = 1;
  if (!mdd_next.check_joint_mdd_conflict(mdd_now, G, HORIZON, DT_NEXT, DT_NOW))
    return 0;
  if (!mdd_next.check_joint_mdd_conflict(mdd_now, G, HORIZON, DT_NEXT + 1, DT_NOW))
    return 1;
  if (!mdd_next.check_joint_mdd_conflict(mdd_now, G, HORIZON, DT_NEXT, DT_NOW - 1))
    return 1;
  return 2;
}


void HorizonPairDBGenerator::flag_mdds_for_conflicts() {
  flagged_for_conflict.assign(mdd_count, {});

  const uint32_t total = mdd_count;
  std::atomic<uint32_t> done = 0;


  ThreadPool pool(NUM_OF_THREADS);

  // Maps each worker thread to a fixed slot so it can reuse its own 'visited' vector.
  std::unordered_map<std::thread::id, int> thread_slot;
  for (int i = 0; i < (int)pool.workers.size(); i++)
    thread_slot[pool.workers[i].get_id()] = i;

  // One 'visited' vector per thread, pre-allocated and reused across mdd_ids.
  std::vector<std::vector<bool>> visited_per_thread(
      pool.workers.size(), std::vector<bool>(mdd_count, false));

  // Task processes a SINGLE mdd_id. flagged_for_conflict[mdd_id] is only ever
  // written by this task, so no lock is needed for it.
  auto task = [&](uint32_t mdd_id) {
    std::vector<bool>& visited = visited_per_thread[thread_slot.at(std::this_thread::get_id())];
    std::fill(visited.begin(), visited.end(), false);

    MDD& mdd = mdd_by_id[mdd_id];
    std::vector<uint32_t>& flagged = flagged_for_conflict[mdd_id];
    for (int t = 0; t < MDD_SIZE; t++) {
      for (Vertex* v : mdd.frontiers[t]) {
        
        int NUM_OF_TIME_SHIFTS = 1;
        std::vector<uint32_t>* timeshift_to_other_mdds[3]; 
        size_t t_s = static_cast<size_t>(t) * G->V.size() + v->id;
        timeshift_to_other_mdds[0] = &mdd_by_t_s[t_s];
        
        if (t < MDD_SIZE - 1) {
          size_t t_plus_1_s = static_cast<size_t>(t + 1) * G->V.size() + v->id;
          timeshift_to_other_mdds[NUM_OF_TIME_SHIFTS] = &mdd_by_t_s[t_plus_1_s];
          NUM_OF_TIME_SHIFTS++;
        }
        
        if (t > 0) {
          size_t t_minus_1_s = static_cast<size_t>(t - 1) * G->V.size() + v->id;
          timeshift_to_other_mdds[NUM_OF_TIME_SHIFTS] = &mdd_by_t_s[t_minus_1_s];
          NUM_OF_TIME_SHIFTS++;
        }

        for (int dt = 0; dt < NUM_OF_TIME_SHIFTS; dt++) {
          std::vector<uint32_t>& mdd_set = *timeshift_to_other_mdds[dt];

          for (uint32_t other_mdd_id : mdd_set) {
            if (other_mdd_id <= mdd_id) continue;
            
            if (visited[other_mdd_id]) continue;
            visited[other_mdd_id] = true;

            MDD& other_mdd = mdd_by_id[other_mdd_id];
            if (other_mdd.frontiers[0][0]->id == mdd.frontiers[0][0]->id) continue;

            flagged.push_back(other_mdd_id);
          }
        }
      }
    }

    std::sort(flagged.begin(), flagged.end());
    flagged.erase(std::unique(flagged.begin(), flagged.end()), flagged.end());

    ++done;
  };

  std::vector<std::future<void>> futures;
  futures.reserve(mdd_count);
  for (uint32_t mdd_id = 0; mdd_id < mdd_count; mdd_id++)
    futures.push_back(pool.submit([&task, mdd_id]() { task(mdd_id); }));

  wait_with_progress(futures, done, total);
  std::cout << std::endl;
}


void HorizonPairDBGenerator::generate_sync_time_conflicts() {
  std::cout << "Generating sync-time penalties" << std::endl;
  penalties.resize(mdd_count);
  std::atomic<uint32_t> num_of_conflicts1 = 0;
  std::atomic<uint32_t> num_of_conflicts2 = 0;

  const uint32_t total = mdd_count;
  std::atomic<uint32_t> done = 0;


  ThreadPool pool(NUM_OF_THREADS);

  // Task processes a SINGLE mdd_id. penalties[mdd_id] is only ever written by
  // this task, so no lock is needed for it.
  auto task = [&](uint32_t mdd_id) {
    MDD& mdd = mdd_by_id[mdd_id];
    for (uint32_t other_mdd_id : flagged_for_conflict[mdd_id]) {
      MDD& other_mdd = mdd_by_id[other_mdd_id];
      uint8_t penalty = calculate_sync_time_penalty(mdd, other_mdd);

      if (penalty > 0) {
        penalties[mdd_id].push_back({other_mdd_id, penalty});
        if (penalty == 1)
          num_of_conflicts1++;
        else
          num_of_conflicts2++;
      }
    }

    ++done;
  };

  std::vector<std::future<void>> futures;
  futures.reserve(mdd_count);
  for (uint32_t mdd_id = 0; mdd_id < mdd_count; mdd_id++)
    futures.push_back(pool.submit([&task, mdd_id]() { task(mdd_id); }));

  wait_with_progress(futures, done, total);
  std::cout << "\nNum of conflicts with penalty 1: " << num_of_conflicts1.load() << std::endl;
  std::cout << "Num of conflicts with penalty 2: " << num_of_conflicts2.load() << std::endl;
}


// TODO: this logic only works for grid graphs and won't work for general graphs.
inline bool check_is_accessible(Vertex* from, Vertex* to) {
  int dx = std::abs(from->x - to->x);
  int dy = std::abs(from->y - to->y);
  return (dx + dy < 2);
}


MDD create_constrained_move_MDD(MDD& mdd1, Vertex* next_v1) {
  MDD new_mdd;
  new_mdd.frontiers.resize(mdd1.frontiers.size());
  new_mdd.frontiers[0] = mdd1.frontiers[0];
  new_mdd.frontiers[1] = { next_v1 };
  for (auto t = 2; t < new_mdd.frontiers.size(); t++) {
    for (Vertex* mdd1_v : mdd1.frontiers[t]) {
      // Find whether mdd1_v is accessible
      bool is_accessible = false;
      for (Vertex* v_constrained_parent : new_mdd.frontiers[t - 1]) {
        if (check_is_accessible(v_constrained_parent, mdd1_v)) { is_accessible = true; break; }
      }
      if (is_accessible) new_mdd.frontiers[t].push_back(mdd1_v);
    }
  }
  return new_mdd;
}


void HorizonPairDBGenerator::generate_constrained_move_conflicts() {
  time_shifted_penalties.resize(mdd_count);

  const uint32_t total = mdd_count;
  std::atomic<uint32_t> done = 0;

  std::cout << "Calculating reverse conflict flags... " << std::endl;
  std::vector<std::vector<uint32_t>> reversed_flagged_for_conflict(mdd_count);
  for (uint32_t mdd1 = 0; mdd1 < mdd_count; mdd1++) {
    for (uint32_t mdd2 : flagged_for_conflict[mdd1]) {
      reversed_flagged_for_conflict[mdd2].push_back(mdd1);
    }
  }

  std::cout << "Generating time shifted conflicts" << std::endl;

  ThreadPool pool(NUM_OF_THREADS);

  auto task = [&](uint32_t mdd_id) {
    MDD& mdd = mdd_by_id[mdd_id];
    // A wait move doesn't need a penalty.
    if (mdd.frontiers[1].size() == 1 && mdd.frontiers[1][0] == mdd.frontiers[0][0]) {
      ++done;
      return;
    }
    for (uint32_t other_mdd_id : flagged_for_conflict[mdd_id]) {
      MDD& other_mdd = mdd_by_id[other_mdd_id];
      uint8_t penalty = calculate_time_shifted_penalty(mdd, other_mdd);
      if (penalty > 0) {
        time_shifted_penalties[mdd_id].push_back( { other_mdd_id, penalty });
      }
    }
    for (uint32_t other_mdd_id : reversed_flagged_for_conflict[mdd_id]) {
      MDD& other_mdd = mdd_by_id[other_mdd_id];
      uint8_t penalty = calculate_time_shifted_penalty(mdd, other_mdd);
      if (penalty > 0) {
        time_shifted_penalties[mdd_id].push_back( { other_mdd_id, penalty });
      }
    }
    ++done;
  };

  std::vector<std::future<void>> futures;
  futures.reserve(mdd_count);
  for (uint32_t mdd_id = 0; mdd_id < mdd_count; mdd_id++)
    futures.push_back(pool.submit([&task, mdd_id]() { task(mdd_id); }));

  wait_with_progress(futures, done, total);
  std::cout << std::endl;
}


void HorizonPairDBGenerator::interactive_mdd_test() {
  using namespace drawing_detail;
  const int W = G->width;
  const int H = G->height;

  while (true) {
    std::cout << "Enter v_i (start vertex id, 0-" << G->V.size() - 1 << "): ";
    int v_i;
    if (!(std::cin >> v_i)) break;

    std::cout << "Enter g_i (goal vertex id, 0-" << G->V.size() - 1 << "): ";
    int g_i;
    if (!(std::cin >> g_i)) break;

    if (v_i < 0 || v_i >= (int)G->V.size() || g_i < 0 || g_i >= (int)G->V.size()) {
      std::cout << "Invalid vertex id(s), must be within [0, " << G->V.size() - 1 << "]\n";
      continue;
    }

    MDD mdd;
    mdd.populate(D, G->V[v_i], G->V[g_i], HORIZON);

    const int goal_index = G->V[g_i]->index;

    std::unordered_map<int, int> depth_of_index;
    for (size_t depth = 0; depth < mdd.frontiers.size(); depth++)
      for (Vertex* v : mdd.frontiers[depth])
        depth_of_index.emplace(v->index, (int)depth);  // keep earliest depth

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int idx = W * y + x;
        auto it = depth_of_index.find(idx);
        if (idx == goal_index) {
          std::cout << GREEN << 'G' << RESET;
        } else if (it != depth_of_index.end()) {
          const char* color = (it->second == 0) ? GREEN : RED;
          std::cout << color << (char)('0' + it->second % 10) << RESET;
        } else {
          std::cout << (G->U[idx] ? '.' : '#');
        }
      }
      std::cout << '\n';
    }

    std::cout << mdd.str() << '\n';
  }
}


uint8_t HorizonPairDBGenerator::get_penalty(Vertex* v1, Vertex* g1, Vertex* v2, Vertex* g2) {
  uint32_t agent1 = v1->id * V_SIZE + g1->id;
  uint32_t agent2 = v2->id * V_SIZE + g2->id;
  uint32_t mdd_id1 = mdd_id_by_v_g[agent1];
  uint32_t mdd_id2 = mdd_id_by_v_g[agent2];
  uint32_t min_mdd_id = mdd_id1 < mdd_id2 ? mdd_id1 : mdd_id2;
  uint32_t max_mdd_id = mdd_id1 > mdd_id2 ? mdd_id1 : mdd_id2;
  for (const MDD_Penalty& entry : penalties[min_mdd_id]) {
    if (entry.mdd_id == max_mdd_id) return entry.penalty;
  }
  return 0;
};


uint8_t HorizonPairDBGenerator::get_constrained_move_penalty(Vertex* v1, Vertex* v1_next, Vertex* g1, Vertex* v2, Vertex* g2) {
  uint32_t agent2 = v2->id * V_SIZE + g2->id;
  uint32_t mdd_id2 = mdd_id_by_v_g[agent2];

  // Scenario 3: agent 1 pushes agent 2
  if (v2->id == v1_next->id) {
    MDD& mdd2_next = mdd_by_id[mdd_id2];
    uint8_t penalty = 2;
    for (Vertex* v2_next : mdd2_next.frontiers[1]) {
      if (v2_next->id == v1->id) continue;
      // std::cout << "[DEBUG 4]" << std::endl;
      // std::cout << "v1_next->id: " << v1_next->id << std::endl;
      // std::cout << "g1->id: " << g1->id << std::endl;
      // std::cout << "v2_next->id: " << v2_next->id << std::endl;
      // std::cout << "g2->id: " << g2->id << std::endl;
      int8_t p = get_penalty(v1_next, g1, v2_next, g2);
      if (p == 0) return 0;
      if (p < penalty) penalty = p;
    }
    return penalty; 
  }

  // Scenario 2: agent 1 is at v1_next at time t+1, agent 2 is at v2 at time t.
  uint32_t agent1 = v1_next->id * V_SIZE + g1->id;
  uint32_t mdd1 = mdd_id_by_v_g[agent1];
  const auto& entries = time_shifted_penalties[mdd1];
  std::cout << "mdd2: " << mdd_id2 << std::endl;
  std::cout << "shifted penalties of " << mdd1 << std::endl;
  for (const MDD_Penalty& entry : entries) {
    std::cout << entry.mdd_id << std::endl;
    if (entry.mdd_id == mdd_id2) return entry.penalty;
  }
  return 0;
};


void HorizonPairDBGenerator::test_db_1() {
  std::cout << "Running test_db_1" << std::endl;
  std::vector<std::string> grid = {
    ".@.",
    "...",
    "...",
  };

  Graph* G = new Graph(grid);
  HorizonPairDBGenerator DB(G, "test");
  DB.generate_mdds();
  DB.flag_mdds_for_conflicts();
  DB.generate_sync_time_conflicts();

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(0, 0);
  auto A_goal  = coord(1, 2);

  auto B_start = coord(1, 0);
  auto B_goal  = coord(0, 2);

  auto C_start = coord(1, 0);
  auto C_goal  = coord(1, 1);

  auto D_start = A_goal; // swap conflict with A
  auto D_goal  = A_start;

  int actual_penalty;
  int expected_penalty;

  //
  // Test A, B
  //
  expected_penalty = 0;
  actual_penalty = DB.get_penalty(A_start, A_goal, B_start, B_goal);
  if (expected_penalty != actual_penalty) {
    std::cout << "Got penalty != 0 for A and B" << std::endl;
    exit(0);
  }

  //
  // Test A, C
  //
  expected_penalty = 2;
  actual_penalty = DB.get_penalty(A_start, A_goal, C_start, C_goal);
  if (expected_penalty != actual_penalty) {
    std::cout << "Got penalty != 2 for A and C" << std::endl;
    exit(0);
  }

  //
  // Test A, D
  //
  expected_penalty = 2;
  actual_penalty = DB.get_penalty(A_start, A_goal, D_start, D_goal);
  if (expected_penalty != actual_penalty) {
    std::cout << "Got penalty != 2 for A and D" << std::endl;
    exit(0);
  }

  delete G;
}


void HorizonPairDBGenerator::test_db_time_shift() {
  std::cout << "Running test_db_time_shift" << std::endl;
  std::vector<std::string> grid = {
    ".@.",
    "...",
    "...",
  };

  Graph* G = new Graph(grid);
  HorizonPairDBGenerator DB(G, "test");
  DB.generate_mdds();
  DB.flag_mdds_for_conflicts();
  DB.generate_sync_time_conflicts();
  DB.generate_constrained_move_conflicts();

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  { // Scenario 1: A follows B (no conflict)
    auto A_start = coord(0, 0);
    auto A_next = coord(1, 0);
    auto A_goal  = coord(1, 2);

    auto B_start = coord(1, 0);
    auto B_goal  = coord(0, 2);

    int expected_penalty = 0;
    int actual_penalty = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 1 failed" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 1 passed" << std::endl;

  { // Scenario 2: B moves first and interfere A
    auto A_start = coord(0, 0);
    auto A_goal  = coord(1, 2);

    auto B_start = coord(1, 1);
    auto B_next = coord(1, 0);
    auto B_goal  = coord(2, 0);

    int expected_penalty = 2;
    int actual_penalty = DB.get_constrained_move_penalty(B_start, B_next, B_goal, A_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 2 failed. Expected: 2, Actual: " << actual_penalty << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 2 passed" << std::endl;

  { // Scenario 3: A follows B but B's goal is on A's path (conflict)
    auto A_start = coord(0, 0);
    auto A_next = coord(1, 0);
    auto A_goal  = coord(0, 2);

    auto B_start = coord(1, 0);
    auto B_goal  = coord(1, 2);

    int expected_penalty = 2;
    int actual_penalty = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 3 failed" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 3 passed" << std::endl;

  { // Scenario 4: Swap conflict when A pushes B
    auto A_start = coord(0, 0);
    auto A_next = coord(1, 0);
    auto A_goal  = coord(0, 2);

    auto B_start = coord(1, 0);
    auto B_goal  = coord(0, 0);

    int expected_penalty = 2;
    int actual_penalty = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 4 failed" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 4 passed" << std::endl;

  { // Scenario 5: Swap conflict
    auto A_start = coord(0, 0);
    auto A_next = coord(1, 0); 
    auto A_goal  = coord(0, 2);

    auto B_start = coord(0, 2);
    auto B_goal  = coord(0, 0);

    auto C_start = coord(1, 2);
    auto C_goal  = coord(0, 0);

    int expected_penalty = 2;
    int actual_penalty_B = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    int actual_penalty_C = DB.get_constrained_move_penalty(A_start, A_next, A_goal, C_start, C_goal);
    if (expected_penalty != actual_penalty_B) {
      std::cout << "Scenario 5 failed (B)" << std::endl;
      exit(0);
    }
    if (expected_penalty != actual_penalty_C) {
      std::cout << "Scenario 5 failed (C)" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 5 passed" << std::endl;

  delete G;
}


void HorizonPairDBGenerator::test_db_time_shift_2() {
  std::cout << "Running test_db_time_shift_2" << std::endl;
  std::vector<std::string> grid = {
    "....",
    "....",
    "....",
    "....",
  };

  Graph* G = new Graph(grid);
  HorizonPairDBGenerator DB(G, "test");
  DB.generate_mdds();
  DB.flag_mdds_for_conflicts();
  DB.generate_sync_time_conflicts();
  DB.generate_constrained_move_conflicts();

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  // The famous vertex conflict example if there is not shift. A wait can resolve this conflict.
  auto A_start = coord(1, 0);
  auto A_goal  = coord(2, 3);

  auto B_start = coord(0, 1);
  auto B_goal  = coord(3, 2);

  { // Scenario 1: No conflict when shifting B
    auto A_next = coord(2, 0);
    auto B_start = coord(0, 2);

    int expected_penalty = 0;
    int actual_penalty = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 1 failed" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 1 passed" << std::endl;

  { // Scenario 2: A conflict if A moves down
    auto A_next = coord(2, 0);

    int expected_penalty = 1;
    int actual_penalty = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 2 failed" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 2 passed" << std::endl;

  { // Scenario 3: A conflict if A moves right
    auto A_next = coord(1, 1);

    int expected_penalty = 1;
    int actual_penalty = DB.get_constrained_move_penalty(A_start, A_next, A_goal, B_start, B_goal);
    if (expected_penalty != actual_penalty) {
      std::cout << "Scenario 3 failed" << std::endl;
      exit(0);
    }
  }
  std::cout << "Scenario 3 passed" << std::endl;

  delete G;
}


// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

void HorizonPairDBGenerator::write_header(std::ofstream& out, const HorizonPairDBFileHeader& header) {
  out.write(reinterpret_cast<const char*>(&header.num_mdds), sizeof(header.num_mdds));
  out.write(reinterpret_cast<const char*>(&header.offset_mdd_by_id), sizeof(header.offset_mdd_by_id));
  out.write(reinterpret_cast<const char*>(&header.offset_mdd_id_by_v_g), sizeof(header.offset_mdd_id_by_v_g));
  out.write(reinterpret_cast<const char*>(&header.offset_conflicts), sizeof(header.offset_conflicts));
  out.write(reinterpret_cast<const char*>(&header.offset_constrained_move_conflicts), sizeof(header.offset_constrained_move_conflicts));
}

// Frontier vertices are stored as v->id (index into G->V), not v->index.
void HorizonPairDBGenerator::write_mdd(std::ofstream& out, const MDD& mdd) {
  uint8_t num_frontiers = static_cast<uint8_t>(mdd.frontiers.size());
  out.write(reinterpret_cast<const char*>(&num_frontiers), sizeof(num_frontiers));
  for (const auto& frontier : mdd.frontiers) {
    uint8_t num_vertices = static_cast<uint8_t>(frontier.size());
    out.write(reinterpret_cast<const char*>(&num_vertices), sizeof(num_vertices));
    for (Vertex* v : frontier) {
      uint16_t vid = static_cast<uint16_t>(v->id);
      out.write(reinterpret_cast<const char*>(&vid), sizeof(vid));
    }
  }
  const uint8_t sentinel = HORIZON_PAIR_DB_MDD_SENTINEL;
  out.write(reinterpret_cast<const char*>(&sentinel), sizeof(sentinel));
}

void HorizonPairDBGenerator::write_mdd_by_id_section(std::ofstream& out) {
  std::cout << "Saving MDDs..." << std::endl;
  for (uint32_t id = 0; id < mdd_count; id++) {
    write_mdd(out, mdd_by_id[id]);
    if (id % 256 == 0 || id + 1 == mdd_count) print_horizon_pair_db_progress_bar(id + 1, mdd_count);
  }
  std::cout << std::endl;
  out.write(reinterpret_cast<const char*>(&mdd_count), sizeof(mdd_count));
}

void HorizonPairDBGenerator::write_mdd_id_by_v_g_section(std::ofstream& out) {
  std::cout << "Saving mdd_id_by_v_g..." << std::endl;
  out.write(reinterpret_cast<const char*>(mdd_id_by_v_g.data()),
            mdd_id_by_v_g.size() * sizeof(uint32_t));
  print_horizon_pair_db_progress_bar(1, 1);
  std::cout << std::endl;
}

void HorizonPairDBGenerator::write_conflicts_section(std::ofstream& out) {
  std::cout << "Saving conflicts..." << std::endl;
  const uint32_t num_conflicts = static_cast<uint32_t>(penalties.size());
  out.write(reinterpret_cast<const char*>(&num_conflicts), sizeof(num_conflicts));
  for (uint32_t id = 0; id < num_conflicts; id++) {
    const auto& entries = penalties[id];
    uint32_t num_entries = static_cast<uint32_t>(entries.size());
    out.write(reinterpret_cast<const char*>(&num_entries), sizeof(num_entries));
    for (const MDD_Penalty& entry : entries) {
      out.write(reinterpret_cast<const char*>(&entry.mdd_id), sizeof(entry.mdd_id));
      out.write(reinterpret_cast<const char*>(&entry.penalty), sizeof(entry.penalty));
    }
    if (id % 256 == 0 || id + 1 == num_conflicts) print_horizon_pair_db_progress_bar(id + 1, num_conflicts);
  }
  std::cout << std::endl;
}

void HorizonPairDBGenerator::write_constrained_move_conflicts_section(std::ofstream& out) {
  std::cout << "Saving constrained move conflicts..." << std::endl;
  const uint32_t num_entries_outer = static_cast<uint32_t>(time_shifted_penalties.size());
  out.write(reinterpret_cast<const char*>(&num_entries_outer), sizeof(num_entries_outer));
  for (uint32_t id = 0; id < num_entries_outer; id++) {
    const auto& entries = time_shifted_penalties[id];
    uint32_t num_entries = static_cast<uint32_t>(entries.size());
    out.write(reinterpret_cast<const char*>(&num_entries), sizeof(num_entries));
    for (const MDD_Penalty& entry : entries) {
      out.write(reinterpret_cast<const char*>(&entry.mdd_id), sizeof(entry.mdd_id));
      out.write(reinterpret_cast<const char*>(&entry.penalty), sizeof(entry.penalty));
    }
    if (id % 256 == 0 || id + 1 == num_entries_outer) print_horizon_pair_db_progress_bar(id + 1, num_entries_outer);
  }
  std::cout << std::endl;
}

bool HorizonPairDBGenerator::save_to_file() {
  std::filesystem::create_directories(HORIZON_PAIR_DB_ROOT_FOLDER);
  const std::string path = HORIZON_PAIR_DB_ROOT_FOLDER + name + HORIZON_PAIR_DB_FILE_EXTENSION;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    std::cerr << "HorizonPairDBGenerator::save_to_file: failed to open " << path << " for writing" << std::endl;
    return false;
  }

  HorizonPairDBFileHeader header{mdd_count, 0, 0, 0, 0};
  write_header(out, header);  // placeholder, patched with real offsets below

  header.offset_mdd_by_id = static_cast<uint64_t>(out.tellp());
  write_mdd_by_id_section(out);

  header.offset_mdd_id_by_v_g = static_cast<uint64_t>(out.tellp());
  write_mdd_id_by_v_g_section(out);

  header.offset_conflicts = static_cast<uint64_t>(out.tellp());
  write_conflicts_section(out);

  header.offset_constrained_move_conflicts = static_cast<uint64_t>(out.tellp());
  write_constrained_move_conflicts_section(out);

  out.seekp(0);
  write_header(out, header);

  if (!out) {
    std::cerr << "HorizonPairDBGenerator::save_to_file: I/O error while writing " << path << std::endl;
    return false;
  }
  std::cout << "Saved HorizonPairDB to " << path << std::endl;
  return true;
}
