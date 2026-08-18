#include "../include/pair_wise_db.hpp"
#include "../include/pair_wise_bin.hpp"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
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

  auto& sa = pair_data[goals_key];
  for (size_t i = 0; i < MAX_VERTICES; ++i) {
    auto& src = local[i];
    if (src.empty()) continue;
    auto& dst = sa[i];
    dst.insert(dst.end(), src.begin(), src.end());
  }
}


void PairWiseDB::load_all(Instance* ins) {
  // Collect unique (lo, hi) pairs for this instance
  std::cout << "Collecting goal pairs from bin files" << std::endl;
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

  std::cout << "Reading files" << std::endl;

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
  std::cout << "Pair DB loaded successfully" << std::endl;
}


void PairWiseDB::load_bin2_file(uint16_t lo, uint16_t hi) {
  std::string path = DB_PATH + name + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin2";
  std::ifstream f(path, std::ios::binary);
  if (!f) return;

  GoalsKey goals_key = to_goals_key(lo, hi);
  // pair_data[goals_key] was pre-allocated in load_all2; no mutex needed
  auto& sa = pair_data[goals_key];

  std::array<uint16_t, MAX_VERTICES> header{};
  f.read(reinterpret_cast<char*>(header.data()), sizeof(header));
  if (!f && f.gcount() != sizeof(header)) {
    std::cerr << "[load_bin2_file] short header read for " << lo << "_" << hi
              << " got=" << f.gcount() << "\n";
    return;
  }

  for (size_t i = 0; i < MAX_VERTICES; ++i) {
    uint16_t count = header[i];
    if (count > 0) {
      sa[i].resize(count);
      auto n = static_cast<std::streamsize>(count * sizeof(RangeEntry));
      f.read(reinterpret_cast<char*>(sa[i].data()), n);
      if (f.gcount() != n) {
        std::cerr << "[load_bin2_file] short payload read for " << lo << "_" << hi
                  << " slot=" << i << " expected=" << n << " got=" << f.gcount() << "\n";
        sa[i].clear();
      }
    }
  }
}


void PairWiseDB::load_all2(Instance* ins) {
  // ── Build required keys from this instance ─────────────────────────────────
  std::cout << "Collecting goal pairs" << std::endl;
  std::set<GoalsKey> required_keys;
  int N = (int)ins->goals.size();
  for (int i = 0; i < N; ++i)
    for (int j = i + 1; j < N; ++j) {
      uint16_t gi = (uint16_t)ins->goals[i]->id, gj = (uint16_t)ins->goals[j]->id;
      required_keys.insert(to_goals_key(std::min(gi, gj), std::max(gi, gj)));
    }

  // ── Pre-allocate map entries single-threaded ───────────────────────────────
  std::vector<GoalsKey> to_load;
  for (GoalsKey key : required_keys) {
      pair_data[key]; // default-construct StartArrays in-place
      to_load.push_back(key);
  }

  std::cout << "Reading bin2 files" << std::endl;

  const int total = (int)to_load.size();
  std::atomic<int> done = 0;
  const int bar_width = 40;
  std::mutex print_mtx;

  auto print_bar = [&]() {
    std::lock_guard<std::mutex> lk(print_mtx);
    float frac = total > 0 ? (float)done.load() / total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
  };

  print_bar();

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;
  futures.reserve(to_load.size());

  for (GoalsKey key : to_load) {
    uint16_t lo = (uint16_t)(key >> 16);
    uint16_t hi = (uint16_t)(key & 0xFFFF);
    futures.push_back(pool.submit([&, lo, hi]() -> ThreadResult {
      load_bin2_file(lo, hi);
      done++;
      print_bar();
      return ThreadResult{};
    }));
  }

  for (auto& fut : futures) fut.get();
  std::cout << std::endl;
  std::cout << "Pair DB (bin2) loaded successfully" << std::endl;
}
