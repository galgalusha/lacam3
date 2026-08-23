#include "../include/pair_wise_db.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>


void PairWiseDB::write_bin2_files() {
  const std::string dir = DB_PATH + name;

  // ── 1. Build goal_keys.bin2 by scanning existing .bin files ──────────────
  std::vector<GoalsKey> keys;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const auto& p = entry.path();
    if (p.extension() != ".bin") continue;
    const std::string stem = p.stem().string();
    const auto sep = stem.find('_');
    if (sep == std::string::npos) continue;
    uint16_t lo = (uint16_t)std::stoul(stem.substr(0, sep));
    uint16_t hi = (uint16_t)std::stoul(stem.substr(sep + 1));
    keys.push_back(to_goals_key(lo, hi));
  }

  {
    std::ofstream index_out(dir + "/goal_keys.bin2", std::ios::binary);
    uint32_t num_keys = (uint32_t)keys.size();
    index_out.write(reinterpret_cast<const char*>(&num_keys), sizeof(uint32_t));
    index_out.write(reinterpret_cast<const char*>(keys.data()), num_keys * sizeof(GoalsKey));
  }
  std::cout << "Wrote goal_keys.bin2 (" << keys.size() << " entries)" << std::endl;

  // ── 2. Write one .bin2 per loaded GoalsKey using a thread pool ────────────
  const int total = (int)pair_data.size();
  std::atomic<int> done = 0;
  const int bar_width = 40;

  auto print_bar = [&]() {
    float frac = total > 0 ? (float)done.load() / total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
  };

  print_bar();

  // Collect iterators so we can dispatch without iterating inside tasks
  std::vector<PairData::const_iterator> iters;
  iters.reserve(pair_data.size());
  for (auto it = pair_data.begin(); it != pair_data.end(); ++it)
    iters.push_back(it);

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;
  futures.reserve(iters.size());

  for (auto it : iters) {
    futures.push_back(pool.submit([&, it]() -> ThreadResult {
      const GoalsKey key = it->first;
      const Bin2Content& start_arrays = it->second;
      uint16_t lo = (uint16_t)(key >> 16);
      uint16_t hi = (uint16_t)(key & 0xFFFF);

      std::string path = dir + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin2";
      std::ofstream f(path, std::ios::binary);

      if (f) {
        // Build the 2 KB header on the stack
        std::array<uint16_t, MAX_VERTICES> header{};
        for (size_t i = 0; i < MAX_VERTICES; ++i)
          header[i] = (uint16_t)start_arrays[i].size();

        f.write(reinterpret_cast<const char*>(header.data()), sizeof(header));

        for (size_t i = 0; i < MAX_VERTICES; ++i) {
          if (header[i] > 0)
            f.write(reinterpret_cast<const char*>(start_arrays[i].data()),
                    header[i] * sizeof(RangeEntry));
        }
      }

      done++;
      print_bar();
      return ThreadResult{};
    }));
  }

  for (auto& fut : futures) fut.get();
  std::cout << std::endl;
  std::cout << "bin2 conversion complete (" << total << " files written)" << std::endl;
}
