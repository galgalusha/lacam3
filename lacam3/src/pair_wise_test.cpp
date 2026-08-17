#include "../include/pair_wise_db.hpp"
#include "../include/pair_wise_bin.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>


void PairWiseHeuristic::integration_test() {
  std::string dir = DB_PATH + name;
  if (!std::filesystem::exists(dir)) {
    std::cout << "[integration_test] directory not found: " << dir << std::endl;
    return;
  }

  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    const std::string fname = e.path().filename().string();
    if (fname.rfind("tmp_", 0) == 0) continue;
    int lo, hi;
    if (std::sscanf(fname.c_str(), "%d_%d.bin", &lo, &hi) == 2)
      files.push_back(e.path());
  }

  std::mt19937 rng(42);
  if ((int)files.size() > 100) {
    std::shuffle(files.begin(), files.end(), rng);
    files.resize(100);
  }

  long long assert_pass = 0, assert_fail = 0;
  std::array<long long, 12> dh_counts{};
  std::vector<std::string> failures;

  for (const auto& fpath : files) {
    std::string fname = fpath.filename().string();
    int lo_i = 0, hi_i = 0;
    std::sscanf(fname.c_str(), "%d_%d.bin", &lo_i, &hi_i);
    uint16_t lo = (uint16_t)lo_i, hi = (uint16_t)hi_i;

    load_bin_file(lo, hi, fpath.string());

    std::ifstream f(fpath.string(), std::ios::binary);
    PairEntry entry;
    while (f.read(reinterpret_cast<char*>(&entry), sizeof(PairEntry))) {
      uint8_t expected = entry.dh > 255 ? 255 : (uint8_t)entry.dh;
      uint8_t actual = get(hi, lo, entry.j_start, entry.i_start);

      int bucket = expected <= 10 ? (int)expected : 11;
      dh_counts[bucket]++;

      if (actual == expected) {
        ++assert_pass;
      } else {
        ++assert_fail;
        if (failures.size() < 20)
          failures.push_back("file=" + fname + " start=(" + std::to_string(entry.i_start) +
                             "," + std::to_string(entry.j_start) + ") expected=" +
                             std::to_string(expected) + " got=" + std::to_string(actual));
      }
    }

    for (auto& m : pair_data) m.clear();
  }

  std::cout << "[integration_test] Results over " << files.size() << " file(s):" << std::endl;
  std::cout << "  Assertions passed: " << assert_pass << std::endl;
  std::cout << "  Assertions failed: " << assert_fail << std::endl;
  std::cout << "  dh distribution:" << std::endl;
  for (int d = 1; d <= 10; ++d)
    std::cout << "    dh=" << d << ": " << dh_counts[d] << std::endl;
  std::cout << "    dh>10: " << dh_counts[11] << std::endl;
  if (!failures.empty()) {
    std::cout << "  Failed assertions (first " << failures.size() << "):" << std::endl;
    for (const auto& msg : failures) std::cout << "    " << msg << std::endl;
  }
}

void PairWiseHeuristic::test() {
  std::vector<std::string> grid = {
    "....",
    "....",
    "@@@.",
    "..@.",
    "....",
  };
  Graph* G = new Graph(grid);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(0, 0);
  auto A_goal  = coord(4, 1);

  auto B_start = coord(1, 1);
  auto B_goal  = coord(3, 0);

  auto C_start = coord(4, 2);
  auto C_goal  = coord(4, 2);

  Config starts = { A_start, B_start, C_start };
  Config goals  = { A_goal,  B_goal,  C_goal  };

  std::string test2 = "test2";
  std::string test2_goals = "test2_goals";
  PairWiseHeuristic pwh_no_goals(G, test2);
  pwh_no_goals.construct_for_instance(goals);
  PairWiseHeuristic pwh_goals(G, test2_goals);
  pwh_goals.construct_for_instance_only_goals(goals);
  merge_goal_folder(DB_PATH + test2, DB_PATH + test2_goals);

  PairWiseHeuristic pwh(G, test2);
  Instance* ins = new Instance(G, starts, goals, starts.size());
  pwh.load_all(ins);

  std::cout << "A with B: " << (int)pwh.get(A_goal->id, B_goal->id, A_start->id, B_start->id) << std::endl;
  std::cout << "A with C: " << (int)pwh.get(A_goal->id, C_goal->id, A_start->id, C_start->id) << std::endl;
  std::cout << "B with C: " << (int)pwh.get(B_goal->id, C_goal->id, B_start->id, C_start->id) << std::endl;
  std::cout << "Done 2" << std::endl;
}
