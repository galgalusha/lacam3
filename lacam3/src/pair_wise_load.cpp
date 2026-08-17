#include "../include/pair_wise_h.hpp"
#include "../include/pair_wise_bin.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <unordered_set>


std::unordered_set<int> PairWiseHeuristic::bfs_with_margin(Vertex* start, Vertex* goal, int margin) const {
  const int optimal = D->get(goal->id, start->id);
  std::unordered_set<int> explored;
  std::queue<Vertex*> open;
  explored.insert(start->id);
  open.push(start);
  while (!open.empty()) {
    Vertex* v = open.front(); open.pop();
    for (Vertex* nb : v->neighbor) {
      if (explored.count(nb->id)) continue;
      if (D->get(goal->id, nb->id) <= optimal + margin) {
        explored.insert(nb->id);
        open.push(nb);
      }
    }
  }
  return explored;
}

void PairWiseHeuristic::load_bin_file(uint16_t lo, uint16_t hi, const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return;
  PairEntry entry;
  while (f.read(reinterpret_cast<char*>(&entry), sizeof(PairEntry))) {
    uint8_t dh;
    if (entry.dh > 255) {
      std::cout << "[pair_wise_h] warning: dh value " << entry.dh << " exceeds 255, trimming to 255" << std::endl;
      dh = 255;
    } else {
      dh = (uint8_t)entry.dh;
    }
    JointKey jk = to_joint_key(lo, hi, entry.i_start, entry.j_start);
    uint8_t idx = (uint8_t)(jk >> 32);
    uint32_t mk = (uint32_t)(jk & 0xFFFFFFFF);
    pair_data[idx][mk] = dh;
  }
}

void PairWiseHeuristic::load_bin_file_filtered(uint16_t lo, uint16_t hi, const std::string& path,
                                                bool nearest_is_lo, const std::unordered_set<int>& vertex_set) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return;
  PairEntry entry;
  while (f.read(reinterpret_cast<char*>(&entry), sizeof(PairEntry))) {
    int nearest_start = nearest_is_lo ? entry.i_start : entry.j_start;
    if (!vertex_set.count(nearest_start)) continue;
    uint8_t dh;
    if (entry.dh > 255) {
      dh = 255;
    } else {
      dh = (uint8_t)entry.dh;
    }
    JointKey jk = to_joint_key(lo, hi, entry.i_start, entry.j_start);
    uint8_t idx = (uint8_t)(jk >> 32);
    std::lock_guard<std::mutex> lock(pair_data_mtx[idx]);
    uint32_t mk = (uint32_t)(jk & 0xFFFFFFFF);
    pair_data[idx][mk] = dh;
  }
}

void PairWiseHeuristic::load_all(Instance* ins) {
  int N = (int)ins->goals.size();
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      uint16_t gi = (uint16_t)ins->goals[i]->id;
      uint16_t gj = (uint16_t)ins->goals[j]->id;
      uint16_t lo = std::min(gi, gj), hi = std::max(gi, gj);
      std::string path = DB_PATH + name + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin";
      load_bin_file(lo, hi, path);
    }
  }
}

void PairWiseHeuristic::load_some(Instance* ins, int margin, int num_of_threads) {
  const int N = (int)ins->goals.size();
  const int total = N * (N - 1) / 2;
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

  ThreadPool pool(num_of_threads);

  std::vector<std::future<ThreadResult>> futures;

  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      futures.push_back(pool.submit([&, i, j]() {
        const int dist_i = D->get(ins->goals[i]->id, ins->starts[i]->id);
        const int dist_j = D->get(ins->goals[j]->id, ins->starts[j]->id);
        const int nearest = (dist_i <= dist_j) ? i : j;

        const uint16_t gi = (uint16_t)ins->goals[i]->id;
        const uint16_t gj = (uint16_t)ins->goals[j]->id;
        const uint16_t lo = std::min(gi, gj), hi = std::max(gi, gj);
        const bool nearest_is_lo = (ins->goals[nearest]->id == lo);

        const auto vertex_set = bfs_with_margin(ins->starts[nearest], ins->goals[nearest], margin);
        const std::string path = DB_PATH + name + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin";

        load_bin_file_filtered(lo, hi, path, nearest_is_lo, vertex_set);

        done++;
        print_bar();
        return ThreadResult{};
      }));
    }
  }

  for (auto& fut : futures) fut.get();

  std::cout << std::endl;
}
