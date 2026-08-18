#include "../include/pair_wise_db.hpp"
#include "../include/pair_wise_bin.hpp"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>


void PairWiseDB::load_bin_file(uint16_t lo, uint16_t hi) {
  auto path = bin_file_name(lo, hi);
  std::ifstream f(path, std::ios::binary);
  if (!f) return;

  GoalsKey goals_key = to_goals_key(lo, hi);

  // Build RangeEntries locally from the sorted BinEntry stream (no shared state touched here)
  std::array<std::vector<RangeEntry>, MAX_VERTICES> local{};

  BinEntry entry;
  bool have_run = false;
  uint16_t run_i, run_j_start, run_next_j;
  uint8_t  run_dh, run_range;

  auto flush_run = [&]() {
    local[run_i].push_back({run_j_start, run_range, run_dh});
  };

  while (f.read(reinterpret_cast<char*>(&entry), sizeof(BinEntry))) {
    uint8_t dh = entry.dh > 255 ? 255 : (uint8_t)entry.dh;
    // Extend current run if same i_start, same dh, consecutive j_start, and range not saturated
    bool extends = have_run && entry.i_start == run_i && dh == run_dh &&
                   entry.j_start == run_next_j && run_range < 255;
    if (extends) {
      run_next_j++;
      run_range++;
    } else {
      if (have_run) flush_run();
      run_i      = entry.i_start;
      run_j_start = entry.j_start;
      run_next_j  = (uint16_t)(entry.j_start + 1);
      run_dh      = dh;
      run_range   = 1;
      have_run    = true;
    }
  }
  if (have_run) flush_run();

  // Merge into pair_data under the stripe mutex (ensure mutexes are initialized)
  if (goals_key_mutexes.empty()) goals_key_mutexes = std::vector<std::mutex>(1);
  size_t mutex_idx = goals_key % goals_key_mutexes.size();
  std::lock_guard<std::mutex> lock(goals_key_mutexes[mutex_idx]);
  auto& sa = pair_data[goals_key];
  for (size_t i = 0; i < MAX_VERTICES; ++i) {
    auto& src = local[i];
    if (src.empty()) continue;
    auto& dst = sa[i];
    dst.insert(dst.end(), src.begin(), src.end());
  }
}


void PairWiseDB::load_all(Instance* ins) {
  // Size goals_key_mutexes by the number of bin files on disk
  size_t num_files = 0;
  std::string db_dir = DB_PATH + name;
  if (std::filesystem::exists(db_dir)) {
    for (const auto& e : std::filesystem::directory_iterator(db_dir)) {
      int lo, hi;
      if (std::sscanf(e.path().filename().string().c_str(), "%d_%d.bin", &lo, &hi) == 2)
        ++num_files;
    }
  }
  goals_key_mutexes = std::vector<std::mutex>(std::max((size_t)1, num_files));

  // Collect unique (lo, hi) pairs for this instance
  std::set<std::pair<uint16_t, uint16_t>> file_set;
  int N = (int)ins->goals.size();
  for (int i = 0; i < N; ++i)
    for (int j = i + 1; j < N; ++j) {
      uint16_t gi = (uint16_t)ins->goals[i]->id, gj = (uint16_t)ins->goals[j]->id;
      file_set.insert({std::min(gi, gj), std::max(gi, gj)});
    }

  // Pre-populate pair_data keys single-threaded so threads never insert into the outer map
  for (auto [lo, hi] : file_set)
    pair_data[to_goals_key(lo, hi)]; // default-constructs StartArrays in-place

  const int total = (int)file_set.size();
  std::atomic<int> done = 0;
  const int bar_width = 40;

  auto print_bar = [&]() {
    float frac = total > 0 ? (float)done / total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100) << "%" << std::flush;
  };

  print_bar();

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;

  for (auto [lo, hi] : file_set) {
    futures.push_back(pool.submit([&, lo, hi]() {
      load_bin_file(lo, hi);
      done++;
      print_bar();
      return ThreadResult{};
    }));
  }
  for (auto& fut : futures) fut.get();
  std::cout << std::endl;
}


