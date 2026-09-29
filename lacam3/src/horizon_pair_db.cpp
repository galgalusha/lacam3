#include "../include/horizon_pair_db.hpp"
#include "../include/thread_pool.hpp"
#include "../include/pair_wise_bin.hpp" // for thread pool
#include "../include/drawing.hpp"

#include <atomic>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>


const std::string ROOT_FOLDER = "./mapf_db/"; 
const int HorizonPairDB::HORIZON = 5;
static const int NUM_OF_THREADS = 8;

static void print_progress_bar(size_t done, size_t total) {
  const int bar_width = 40;
  float frac = total > 0 ? (float)done / (float)total : 1.0f;
  int filled = (int)(frac * bar_width);
  std::cout << "\r[";
  for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
  std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
}

// Prints median/average/p90/p95 of the number of entries per conflict map.
static void print_conflict_entry_stats(std::vector<uint32_t> entries_per_map) {
  if (entries_per_map.empty()) return;
  std::sort(entries_per_map.begin(), entries_per_map.end());

  const size_t n = entries_per_map.size();
  auto percentile = [&](double p) {
    size_t idx = std::min(n - 1, static_cast<size_t>(p * n));
    return entries_per_map[idx];
  };

  double sum = 0;
  for (uint32_t v : entries_per_map) sum += v;
  double average = sum / (double)n;
  double median = (n % 2 == 0)
      ? (entries_per_map[n / 2 - 1] + entries_per_map[n / 2]) / 2.0
      : entries_per_map[n / 2];

  std::cout << "Conflict map entry stats (n=" << n << "): "
            << "average=" << std::fixed << std::setprecision(2) << average
            << ", median=" << median
            << ", p90=" << percentile(0.90)
            << ", p95=" << percentile(0.95) << std::endl;
}

// Prints all entries of conflicts[mdd_id], one per line, sorted lexicographically
// by (other_mdd_id, value).
static void print_conflict_entries(uint32_t mdd_id, const absl::flat_hash_map<uint32_t, uint8_t>& map) {
  std::vector<std::pair<uint32_t, uint8_t>> entries(map.begin(), map.end());
  std::sort(entries.begin(), entries.end());

  std::cout << "Conflicts for mdd_id=" << mdd_id << " (n=" << entries.size() << "):" << std::endl;
  std::cout << "other_mdd_id\tvalue" << std::endl;
  for (const auto& [other_mdd_id, value] : entries)
    std::cout << other_mdd_id << '\t' << (int)value << std::endl;
}


static DistTable* create_dist_table(Graph* G) {
  std::cout << "Creating the big DistTable" << std::endl;
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  std::cout << "Finished creating the big DistTable" << std::endl;
  return D;
}

HorizonPairDB::HorizonPairDB(Graph* _G, std::string _name) : G(_G), D(create_dist_table(G)), V_SIZE(_G->V.size()), name(_name) {}


void HorizonPairDB::generate_mdds() {
  size_t V_SIZE = G->V.size();

  // Pre-allocate master storage
  mdd_id_by_v_g.assign(V_SIZE * V_SIZE, 0);
  mdd_by_id.resize(V_SIZE * V_SIZE);
  mdd_by_t_s.assign(V_SIZE * (HORIZON + 1), std::vector<uint32_t>());

  std::mutex mdd_count_mtx; // Only used to safely increment the global ID
  std::mutex mdd_mtx2;      // Protects push_back on mdd_by_t_s
  std::mutex io_mtx;        // Protects console output

  const long long total = (long long)V_SIZE * (long long)V_SIZE;
  std::atomic done = 0;
  const int bar_width = 40;

  auto print_bar = [&]() {
    std::lock_guard lock(io_mtx);
    float frac = total > 0 ? (float)done.load() / (float)total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
  };

  print_bar();

  // Task processes an ENTIRE start vertex (v_i)
  auto task = [&](int v_i) {
    // Completely thread-local map! No mutex needed to access this.
    absl::flat_hash_map<MDD, uint32_t> local_mdd_to_id;
    
    for (int g_i = 0; g_i < (int)V_SIZE; g_i++) {
      int agent_id = v_i * (int)V_SIZE + g_i;
      
      MDD mdd; 
      mdd.populate(D, G->V[v_i], G->V[g_i], HORIZON);
      
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
    print_bar(); 
  };

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;
  futures.reserve(V_SIZE);

  // Submit ONE task per start vertex
  for (int v_i = 0; v_i < (int)V_SIZE; v_i++) {
    futures.push_back(pool.submit([&task, v_i]() { task(v_i); return ThreadResult{}; }));
  }

  for (auto& fut : futures) fut.get();

  print_bar(); // Final 100% update
  std::cout << "\nNum of agents: " << V_SIZE * V_SIZE << std::endl;
  std::cout << "Num of MDDs  : " << mdd_count << std::endl;
}


uint8_t HorizonPairDB::get_conflict_penalty(MDD& mdd1, MDD& mdd2) {
  if (!mdd1.check_joint_mdd_conflict(mdd2, G))
    return 0;
  MDD mdd1_with_wait = mdd1.get_mdd_with_wait();
  if (mdd1_with_wait.check_joint_mdd_conflict(mdd2, G))
    return 2;
  else
    return 1;
}


void HorizonPairDB::generate_conflicts() {
  conflicts.resize(mdd_count);
  std::atomic<uint32_t> num_of_conflicts1 = 0;
  std::atomic<uint32_t> num_of_conflicts2 = 0;

  std::mutex io_mtx; // Protects console progress bar output
  const uint32_t total = mdd_count;
  std::atomic<uint32_t> done = 0;
  const int bar_width = 40;

  auto print_bar = [&]() {
    std::lock_guard lock(io_mtx);
    float frac = total > 0 ? (float)done.load() / (float)total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
  };

  print_bar();

  ThreadPool pool(NUM_OF_THREADS);

  // Maps each worker thread to a fixed slot so it can reuse its own 'visited' vector.
  std::unordered_map<std::thread::id, int> thread_slot;
  for (int i = 0; i < (int)pool.workers.size(); i++)
    thread_slot[pool.workers[i].get_id()] = i;

  // One 'visited' vector per thread, pre-allocated and reused across mdd_ids.
  std::vector<std::vector<bool>> visited_per_thread(
      pool.workers.size(), std::vector<bool>(mdd_count, false));

  // Task processes a SINGLE mdd_id. conflicts[mdd_id] is only ever written by
  // this task, so no lock is needed for it.
  auto task = [&](uint32_t mdd_id) {
    std::vector<bool>& visited = visited_per_thread[thread_slot.at(std::this_thread::get_id())];
    std::fill(visited.begin(), visited.end(), false);

    MDD& mdd = mdd_by_id[mdd_id];
    for (int t = 0; t <= HORIZON; t++) {
      for (Vertex* v : mdd.frontiers[t]) {
        
        int NUM_OF_MULTI_SETS = 1;
        std::vector<uint32_t>* multiset_of_mdd_ids[2]; 
        size_t t_s = static_cast<size_t>(t) * G->V.size() + v->id;
        multiset_of_mdd_ids[0] = &mdd_by_t_s[t_s];
        
        if (t < HORIZON) {
          NUM_OF_MULTI_SETS++;
          size_t t_plus_1_s = static_cast<size_t>(t + 1) * G->V.size() + v->id;
          multiset_of_mdd_ids[1] = &mdd_by_t_s[t_plus_1_s];
        }
        
        for (int k = 0; k < NUM_OF_MULTI_SETS; k++) {
          std::vector<uint32_t>& mdd_set = *multiset_of_mdd_ids[k];

          for (uint32_t other_mdd_id : mdd_set) {
            if (other_mdd_id <= mdd_id) continue;
            
            if (visited[other_mdd_id]) continue;
            visited[other_mdd_id] = true;

            MDD& other_mdd = mdd_by_id[other_mdd_id];
            uint8_t penalty = get_conflict_penalty(mdd, other_mdd);
            
            if (penalty > 0) {
              conflicts[mdd_id][other_mdd_id] = penalty;
              if (penalty == 1)
                num_of_conflicts1++;
              else
                num_of_conflicts2++;
            }
          }
        }
      }
    }

    ++done;
    print_bar();
  };

  std::vector<std::future<ThreadResult>> futures;
  futures.reserve(mdd_count);
  for (uint32_t mdd_id = 0; mdd_id < mdd_count; mdd_id++)
    futures.push_back(pool.submit([&task, mdd_id]() { task(mdd_id); return ThreadResult{}; }));

  for (auto& fut : futures) fut.get();

  print_bar(); // Final 100% update
  std::cout << "\nNum of conflicts with penalty 1: " << num_of_conflicts1.load() << std::endl;
  std::cout << "Num of conflicts with penalty 2: " << num_of_conflicts2.load() << std::endl;
}

void HorizonPairDB::interactive_mdd_test() {
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


void HorizonPairDB::test_db_1() {
  std::cout << "Running test_db_1" << std::endl;
  std::vector<std::string> grid = {
    ".@.",
    "...",
    "...",
  };
  Graph* G = new Graph(grid);
  HorizonPairDB DB(G, "test");
  DB.generate_mdds();
  DB.generate_conflicts();

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
  actual_penalty = DB.get_penalty(A_start->id, A_goal->id, B_start->id, B_goal->id);
  if (expected_penalty != actual_penalty) {
    std::cout << "Got penalty != 0 for A and B" << std::endl;
    exit(0);
  }

  //
  // Test A, C
  //
  expected_penalty = 2;
  actual_penalty = DB.get_penalty(A_start->id, A_goal->id, C_start->id, C_goal->id);
  if (expected_penalty != actual_penalty) {
    std::cout << "Got penalty != 2 for A and C" << std::endl;
    exit(0);
  }

  //
  // Test A, D
  //
  expected_penalty = 2;
  actual_penalty = DB.get_penalty(A_start->id, A_goal->id, D_start->id, D_goal->id);
  if (expected_penalty != actual_penalty) {
    std::cout << "Got penalty != 2 for A and D" << std::endl;
    exit(0);
  }

  delete G;
}


// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

void HorizonPairDB::write_header(std::ofstream& out, const FileHeader& header) {
  out.write(reinterpret_cast<const char*>(&header.num_mdds), sizeof(header.num_mdds));
  out.write(reinterpret_cast<const char*>(&header.offset_mdd_by_id), sizeof(header.offset_mdd_by_id));
  out.write(reinterpret_cast<const char*>(&header.offset_mdd_id_by_v_g), sizeof(header.offset_mdd_id_by_v_g));
  out.write(reinterpret_cast<const char*>(&header.offset_conflicts), sizeof(header.offset_conflicts));
}

bool HorizonPairDB::read_header(std::ifstream& in, FileHeader& header) {
  in.read(reinterpret_cast<char*>(&header.num_mdds), sizeof(header.num_mdds));
  in.read(reinterpret_cast<char*>(&header.offset_mdd_by_id), sizeof(header.offset_mdd_by_id));
  in.read(reinterpret_cast<char*>(&header.offset_mdd_id_by_v_g), sizeof(header.offset_mdd_id_by_v_g));
  in.read(reinterpret_cast<char*>(&header.offset_conflicts), sizeof(header.offset_conflicts));
  return static_cast<bool>(in);
}

// Frontier vertices are stored as v->id (index into G->V), not v->index.
void HorizonPairDB::write_mdd(std::ofstream& out, const MDD& mdd) {
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
  const uint8_t sentinel = 255;
  out.write(reinterpret_cast<const char*>(&sentinel), sizeof(sentinel));
}

bool HorizonPairDB::read_mdd(std::ifstream& in, MDD& mdd) {
  uint8_t num_frontiers;
  in.read(reinterpret_cast<char*>(&num_frontiers), sizeof(num_frontiers));
  mdd.frontiers.assign(num_frontiers, {});
  for (uint8_t t = 0; t < num_frontiers; t++) {
    uint8_t num_vertices;
    in.read(reinterpret_cast<char*>(&num_vertices), sizeof(num_vertices));
    mdd.frontiers[t].resize(num_vertices);
    for (uint8_t i = 0; i < num_vertices; i++) {
      uint16_t vid;
      in.read(reinterpret_cast<char*>(&vid), sizeof(vid));
      mdd.frontiers[t][i] = G->V[vid];  // v->id doubles as the index into G->V
    }
  }
  uint8_t sentinel;
  in.read(reinterpret_cast<char*>(&sentinel), sizeof(sentinel));
  if (!in) return false;
  if (sentinel != 255) {
    std::cerr << "HorizonPairDB::read_mdd: sentinel mismatch, file is corrupt" << std::endl;
    return false;
  }
  return true;
}

void HorizonPairDB::write_mdd_by_id_section(std::ofstream& out) {
  std::cout << "Saving MDDs..." << std::endl;
  for (uint32_t id = 0; id < mdd_count; id++) {
    write_mdd(out, mdd_by_id[id]);
    if (id % 256 == 0 || id + 1 == mdd_count) print_progress_bar(id + 1, mdd_count);
  }
  std::cout << std::endl;
  out.write(reinterpret_cast<const char*>(&mdd_count), sizeof(mdd_count));
}

bool HorizonPairDB::read_mdd_by_id_section(std::ifstream& in) {
  std::cout << "Loading MDDs..." << std::endl;
  mdd_by_id.resize(mdd_count);
  for (uint32_t id = 0; id < mdd_count; id++) {
    if (!read_mdd(in, mdd_by_id[id])) return false;
    if (id % 256 == 0 || id + 1 == mdd_count) print_progress_bar(id + 1, mdd_count);
  }
  std::cout << std::endl;

  uint32_t summary;
  in.read(reinterpret_cast<char*>(&summary), sizeof(summary));
  if (!in || summary != mdd_count) {
    std::cerr << "HorizonPairDB::read_mdd_by_id_section: summary mismatch, expected "
              << mdd_count << " got " << summary << std::endl;
    return false;
  }
  return true;
}

void HorizonPairDB::write_mdd_id_by_v_g_section(std::ofstream& out) {
  std::cout << "Saving mdd_id_by_v_g..." << std::endl;
  out.write(reinterpret_cast<const char*>(mdd_id_by_v_g.data()),
            mdd_id_by_v_g.size() * sizeof(uint32_t));
  print_progress_bar(1, 1);
  std::cout << std::endl;
}

bool HorizonPairDB::read_mdd_id_by_v_g_section(std::ifstream& in) {
  std::cout << "Loading mdd_id_by_v_g..." << std::endl;
  mdd_id_by_v_g.assign(V_SIZE * V_SIZE, 0);
  in.read(reinterpret_cast<char*>(mdd_id_by_v_g.data()),
          mdd_id_by_v_g.size() * sizeof(uint32_t));
  print_progress_bar(1, 1);
  std::cout << std::endl;
  return static_cast<bool>(in);
}

void HorizonPairDB::write_conflicts_section(std::ofstream& out) {
  std::cout << "Saving conflicts..." << std::endl;
  const uint32_t num_conflicts = static_cast<uint32_t>(conflicts.size());
  out.write(reinterpret_cast<const char*>(&num_conflicts), sizeof(num_conflicts));
  for (uint32_t id = 0; id < num_conflicts; id++) {
    const auto& map = conflicts[id];
    uint32_t num_entries = static_cast<uint32_t>(map.size());
    out.write(reinterpret_cast<const char*>(&num_entries), sizeof(num_entries));
    for (const auto& [key, value] : map) {
      out.write(reinterpret_cast<const char*>(&key), sizeof(key));
      out.write(reinterpret_cast<const char*>(&value), sizeof(value));
    }
    if (id % 256 == 0 || id + 1 == num_conflicts) print_progress_bar(id + 1, num_conflicts);
  }
  std::cout << std::endl;
}

bool HorizonPairDB::read_conflicts_section(std::ifstream& in) {
  std::cout << "Loading conflicts..." << std::endl;
  uint32_t num_conflicts;
  in.read(reinterpret_cast<char*>(&num_conflicts), sizeof(num_conflicts));
  if (!in || num_conflicts != mdd_count) {
    std::cerr << "HorizonPairDB::read_conflicts_section: count mismatch, expected "
              << mdd_count << " got " << num_conflicts << std::endl;
    return false;
  }

  conflicts.assign(num_conflicts, {});
  std::vector<uint32_t> entries_per_map;
  entries_per_map.reserve(num_conflicts);
  for (uint32_t id = 0; id < num_conflicts; id++) {
    uint32_t num_entries;
    in.read(reinterpret_cast<char*>(&num_entries), sizeof(num_entries));
    if (!in) return false;
    auto& map = conflicts[id];
    map.reserve(num_entries);
    for (uint32_t e = 0; e < num_entries; e++) {
      uint32_t key;
      uint8_t value;
      in.read(reinterpret_cast<char*>(&key), sizeof(key));
      in.read(reinterpret_cast<char*>(&value), sizeof(value));
      if (!in) return false;
      map.emplace(key, value);
    }
    entries_per_map.push_back(num_entries);
    if (id % 256 == 0 || id + 1 == num_conflicts) print_progress_bar(id + 1, num_conflicts);
  }
  std::cout << std::endl;
  print_conflict_entry_stats(entries_per_map);

  const uint32_t debug_mdd_id = 15151;
  if (debug_mdd_id < conflicts.size())
    print_conflict_entries(debug_mdd_id, conflicts[debug_mdd_id]);
  return true;
}

bool HorizonPairDB::save_to_file() {
  std::filesystem::create_directories(ROOT_FOLDER);
  const std::string path = ROOT_FOLDER + name + ".mdd_db";
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    std::cerr << "HorizonPairDB::save_to_file: failed to open " << path << " for writing" << std::endl;
    return false;
  }

  FileHeader header{mdd_count, 0, 0, 0};
  write_header(out, header);  // placeholder, patched with real offsets below

  header.offset_mdd_by_id = static_cast<uint64_t>(out.tellp());
  write_mdd_by_id_section(out);

  header.offset_mdd_id_by_v_g = static_cast<uint64_t>(out.tellp());
  write_mdd_id_by_v_g_section(out);

  header.offset_conflicts = static_cast<uint64_t>(out.tellp());
  write_conflicts_section(out);

  out.seekp(0);
  write_header(out, header);

  if (!out) {
    std::cerr << "HorizonPairDB::save_to_file: I/O error while writing " << path << std::endl;
    return false;
  }
  std::cout << "Saved HorizonPairDB to " << path << std::endl;
  return true;
}

bool HorizonPairDB::load_from_file() {
  const std::string path = ROOT_FOLDER + name + ".mdd_db";
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cerr << "HorizonPairDB::load_from_file: failed to open " << path << " for reading" << std::endl;
    return false;
  }

  FileHeader header;
  if (!read_header(in, header)) {
    std::cerr << "HorizonPairDB::load_from_file: failed to read header from " << path << std::endl;
    return false;
  }
  mdd_count = header.num_mdds;

  in.seekg(header.offset_mdd_by_id);
  if (!read_mdd_by_id_section(in)) return false;

  in.seekg(header.offset_mdd_id_by_v_g);
  if (!read_mdd_id_by_v_g_section(in)) return false;

  in.seekg(header.offset_conflicts);
  if (!read_conflicts_section(in)) return false;

  std::cout << "Loaded HorizonPairDB from " << path << std::endl;
  return true;
}