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
int HorizonPairDBGenerator::HORIZON = 7;
static const int NUM_OF_THREADS = 12;

// MDD::frontiers[0] is the start vertex (time 0).
// This is why we add 1 to HORIZON.
// Given two MDDs aligned to the same time, we look up to frontiers[HORIZON + 1].
// When comparing time shifted MDDs, the shifted one needs to look forward an extra step.
// Thats why we add another 1.
int HorizonPairDBGenerator::MDD_SIZE = HorizonPairDBGenerator::HORIZON + 2;


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
  if (!mdd_next.check_joint_mdd_conflict(mdd_now, G, HORIZON, DT_NEXT, DT_NOW - 1))
    return 1;
  MDD mdd_next_delayed = mdd_next.get_mdd_with_wait();
  if (!mdd_next_delayed.check_joint_mdd_conflict(mdd_now, G, HORIZON, DT_NEXT, DT_NOW))
    return 1;
  return 2;
}


static std::string temp_path_prefix(const std::string& name) {
  return HORIZON_PAIR_DB_ROOT_FOLDER + name;
}

static void write_bulk_section(std::ofstream& out, const std::vector<std::vector<HorizonPairDBGenerator::MDD_Penalty>>& bulk, int local_size) {
  for (int i = 0; i < local_size; i++) {
    uint32_t num_entries = static_cast<uint32_t>(bulk[i].size());
    out.write(reinterpret_cast<const char*>(&num_entries), sizeof(num_entries));
    for (const auto& entry : bulk[i]) {
      out.write(reinterpret_cast<const char*>(&entry.mdd_id), sizeof(entry.mdd_id));
      out.write(reinterpret_cast<const char*>(&entry.penalty), sizeof(entry.penalty));
    }
  }
}


void HorizonPairDBGenerator::generate_all_conflicts_in_bulks(int bulk_size) {
  std::filesystem::create_directories(HORIZON_PAIR_DB_ROOT_FOLDER);
  const std::string temp_sync_path = temp_path_prefix(name) + "_sync_conflicts.tmp";
  const std::string temp_async_path = temp_path_prefix(name) + "_async_conflicts.tmp";
  std::ofstream out_sync(temp_sync_path, std::ios::binary | std::ios::trunc);
  std::ofstream out_async(temp_async_path, std::ios::binary | std::ios::trunc);
  if (!out_sync || !out_async) throw std::runtime_error("Failed to open temporary conflict files");

  const int num_bulks = (static_cast<int>(mdd_count) + bulk_size - 1) / bulk_size;
  std::cout << "Generating conflicts in " << num_bulks << " bulks" << std::endl;

  ThreadPool pool(NUM_OF_THREADS);
  std::unordered_map<std::thread::id, int> thread_slot;
  for (int i = 0; i < (int)pool.workers.size(); i++)
    thread_slot[pool.workers[i].get_id()] = i;
  std::vector<std::vector<bool>> visited_per_thread(
      pool.workers.size(), std::vector<bool>(mdd_count, false));

  for (int bulk = 0; bulk < num_bulks; bulk++) {
    const int start_mdd_id = bulk * bulk_size;
    const int end_mdd_id = std::min(static_cast<int>(mdd_count), start_mdd_id + bulk_size);
    const int local_size = end_mdd_id - start_mdd_id;

    std::vector<std::vector<MDD_Penalty>> bulk_sync(local_size);
    std::vector<std::vector<MDD_Penalty>> bulk_async(local_size);

    std::atomic<uint32_t> done = 0;

    // Each task writes only to bulk_*[local_idx], so no locking is needed.
    auto task = [&](int mdd_id) {
      const int local_idx = mdd_id - start_mdd_id;
      std::vector<bool>& visited = visited_per_thread[thread_slot.at(std::this_thread::get_id())];

      MDD& mdd = mdd_by_id[mdd_id];
      std::vector<uint32_t> candidates;
      for (int t = 0; t < MDD_SIZE; t++) {
        for (Vertex* v : mdd.frontiers[t]) {
          std::vector<uint32_t>* sets[3];
          int num_sets = 0;
          sets[num_sets++] = &mdd_by_t_s[static_cast<size_t>(t) * G->V.size() + v->id];
          if (t < MDD_SIZE - 1)
            sets[num_sets++] = &mdd_by_t_s[static_cast<size_t>(t + 1) * G->V.size() + v->id];
          if (t > 0)
            sets[num_sets++] = &mdd_by_t_s[static_cast<size_t>(t - 1) * G->V.size() + v->id];

          for (int k = 0; k < num_sets; k++) {
            for (uint32_t other_mdd_id : *sets[k]) {
              if (mdd_by_id[other_mdd_id].frontiers[0][0]->id == mdd.frontiers[0][0]->id) continue;
              if (visited[other_mdd_id]) continue;
              visited[other_mdd_id] = true;
              candidates.push_back(other_mdd_id);
            }
          }
        }
      }
      std::sort(candidates.begin(), candidates.end());
      candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

      for (uint32_t c : candidates) {
        visited[c] = false;
      }

      for (uint32_t other_mdd_id : candidates) {
        uint8_t penalty = calculate_time_shifted_penalty(mdd, mdd_by_id[other_mdd_id]);
        if (penalty > 0) bulk_async[local_idx].push_back({other_mdd_id, penalty});
      }
      for (uint32_t other_mdd_id : candidates) {
        if ((uint32_t)mdd_id >= other_mdd_id) continue;
        uint8_t penalty = calculate_sync_time_penalty(mdd, mdd_by_id[other_mdd_id]);
        if (penalty > 0) bulk_sync[local_idx].push_back({other_mdd_id, penalty});
      }
      ++done;
    };

    std::vector<std::future<void>> futures;
    futures.reserve(local_size);
    for (int mdd_id = start_mdd_id; mdd_id < end_mdd_id; mdd_id++)
      futures.push_back(pool.submit([&task, mdd_id]() { task(mdd_id); }));

    std::cout << "Bulk " << (bulk + 1) << "/" << num_bulks << std::endl;
    wait_with_progress(futures, done, local_size);
    std::cout << std::endl;

    write_bulk_section(out_sync, bulk_sync, local_size);
    write_bulk_section(out_async, bulk_async, local_size);
  }
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

bool HorizonPairDBGenerator::generate_and_save() {
  std::filesystem::create_directories(HORIZON_PAIR_DB_ROOT_FOLDER);
  const std::string path = HORIZON_PAIR_DB_ROOT_FOLDER + name + HORIZON_PAIR_DB_FILE_EXTENSION;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;

  // Write dummy header to reserve space
  HorizonPairDBFileHeader header{0, 0, 0, 0, 0};
  write_header(out, header);

  generate_mdds();
  header.num_mdds = mdd_count;

  header.offset_mdd_by_id = static_cast<uint64_t>(out.tellp());
  write_mdd_by_id_section(out);

  header.offset_mdd_id_by_v_g = static_cast<uint64_t>(out.tellp());
  write_mdd_id_by_v_g_section(out);

  generate_all_conflicts_in_bulks();
  mdd_by_t_s.clear();
  mdd_by_t_s.shrink_to_fit();

  const std::string temp_sync_path = temp_path_prefix(name) + "_sync_conflicts.tmp";
  const std::string temp_async_path = temp_path_prefix(name) + "_async_conflicts.tmp";

  std::cout << "Merging sync conflicts..." << std::endl;
  header.offset_conflicts = static_cast<uint64_t>(out.tellp());
  out.write(reinterpret_cast<const char*>(&mdd_count), sizeof(mdd_count));
  {
    std::ifstream in_sync(temp_sync_path, std::ios::binary);
    if (!in_sync) return false;
    out << in_sync.rdbuf();
    out.clear();  // rdbuf insertion sets failbit if the source is empty
  }

  std::cout << "Merging async conflicts..." << std::endl;
  header.offset_constrained_move_conflicts = static_cast<uint64_t>(out.tellp());
  out.write(reinterpret_cast<const char*>(&mdd_count), sizeof(mdd_count));
  {
    std::ifstream in_async(temp_async_path, std::ios::binary);
    if (!in_async) return false;
    out << in_async.rdbuf();
    out.clear();
  }

  out.seekp(0);
  write_header(out, header);
  out.close();

  std::filesystem::remove(temp_sync_path);
  std::filesystem::remove(temp_async_path);

  std::cout << "Saved HorizonPairDB to " << path << std::endl;
  return true;
}
