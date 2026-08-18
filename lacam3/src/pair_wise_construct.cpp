//
// How to construct a DB from scratch:
// 1. The map size must be up to 32x32.
// 2. construct_for_instance - generate bin files
// 3. construct_for_instance_only_goals - in a differet name (folder)
// 4. merge_goal_folder(bin_folder, goals_folder).
//    this will merge the goals into the original bins
// 5. create bin2 files: { load_all(&ins); write_bin2_files(); }
// 6. delete the bin files leaving only the bin2 files
//

#include "../include/pair_wise_db.hpp"
#include "../include/dist_table.hpp"
#include "../include/pair_wise_bin.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>


std::unordered_set<PairKey> PairWiseDB::get_keys_from_files() {
  std::unordered_set<PairKey> keys;
  if (!std::filesystem::exists(DB_PATH + name)) return keys;
  for (const auto& entry : std::filesystem::directory_iterator(DB_PATH + name)) {
    const std::string fname = entry.path().filename().string();
    if (fname.rfind("tmp_", 0) == 0) continue;
    int lo, hi;
    if (std::sscanf(fname.c_str(), "%d_%d.bin", &lo, &hi) == 2)
      keys.insert(to_bin_key((uint16_t)lo, (uint16_t)hi));
  }
  return keys;
}

void PairWiseDB::construct() {
  const int num_vertices = G->V.size();

  long long count_evaluated = 0;
  long long count_interfering = 0;
  shared_zero_dh_gen = std::vector<uint32_t>(num_vertices * num_vertices, 0);

  const auto done_keys = get_keys_from_files();
  long long total_outer = (long long)num_vertices * (num_vertices - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  ThreadPool pool(NUM_OF_THREADS);
  for (int i_g = 0; i_g < num_vertices; ++i_g) {
    for (int j_g = i_g + 1; j_g < num_vertices; ++j_g, ++outer_idx) {
      if (done_keys.count(to_bin_key((uint16_t)i_g, (uint16_t)j_g))) continue;
      int pct = (int)(outer_idx * 100000 / total_outer);
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000;
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }

      populate_zero_dh_by_bfs(D, G->V[i_g], G->V[j_g]);

      std::string tmp_path   = DB_PATH + name + "/" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = DB_PATH + name + "/" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories(DB_PATH + name);
      { std::ofstream touch(tmp_path, std::ios::binary); }

      std::vector<std::future<ThreadResult>> futures;

      for (int i_s = 0; i_s < num_vertices; ++i_s) {
        if (i_s == i_g) continue;

        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          for (int j_s = 0; j_s < (int)G_ref.V.size(); ++j_s) {
            if (j_s == j_g) continue;
            if (i_s == j_s) continue;
            ++result.evaluated;

            int ek = i_s * D->K + j_s;
            if (shared_zero_dh_gen[ek] == shared_zero_current_gen) continue;

            if (can_interfere(D_ptr, i_s, i_g, j_s, j_g)) {
              Vertex* vi_s = G_ref.V[i_s];
              Vertex* vi_g = G_ref.V[i_g];
              Vertex* vj_s = G_ref.V[j_s];
              Vertex* vj_g = G_ref.V[j_g];

              int independent_cost = D_ptr->get(i_g, i_s) + D_ptr->get(j_g, j_s);
              int joint_cost = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g);
              int d_h = joint_cost - independent_cost;
              if (d_h > 0)
                result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)d_h});
            }
          }
          return result;
        }));
      }

      for (auto& fut : futures) {
        auto [entries, evaluated] = fut.get();
        count_evaluated += evaluated;
        count_interfering += entries.size();
        append_entries(tmp_path, entries);
      }

      std::filesystem::rename(tmp_path, final_path);
    }
  }

  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
  std::cout << "Total 4-tuples evaluated: " << count_evaluated << std::endl;
  std::cout << "Pairs flagged for A*:     " << count_interfering << std::endl;
}

void PairWiseDB::construct_for_instance(const Config& goals) {
  long long count_evaluated = 0;
  long long count_interfering = 0;
  const int num_vertices = G->V.size();
  const int K = (int)goals.size();
  shared_zero_dh_gen = std::vector<uint32_t>(num_vertices * num_vertices, 0);

  const auto done_keys = get_keys_from_files();
  long long total_outer = (long long)K * (K - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  ThreadPool pool(NUM_OF_THREADS);
  for (int agent1 = 0; agent1 < K; ++agent1) {
    for (int agent2 = agent1 + 1; agent2 < K; ++agent2, ++outer_idx) {
      int i_g = std::min(goals[agent1]->id, goals[agent2]->id);
      int j_g = std::max(goals[agent1]->id, goals[agent2]->id);
      if (done_keys.count(to_bin_key((uint16_t)i_g, (uint16_t)j_g))) continue;

      int pct = (int)(outer_idx * 100000 / total_outer);
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000;
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }

      populate_zero_dh_by_bfs(D, G->V[i_g], G->V[j_g]);

      std::string tmp_path   = DB_PATH + name + "/tmp_" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = DB_PATH + name + "/"     + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories(DB_PATH + name);
      { std::ofstream touch(tmp_path, std::ios::binary); }

      std::vector<std::future<ThreadResult>> futures;

      for (int i_s = 0; i_s < num_vertices; ++i_s) {
        if (i_s == i_g) continue;

        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          for (int j_s = 0; j_s < (int)G_ref.V.size(); ++j_s) {
            if (j_s == j_g) continue;
            if (i_s == j_s) continue;
            ++result.evaluated;

            int ek = i_s * D->K + j_s;

            if (shared_zero_dh_gen[ek] == shared_zero_current_gen) continue;

            if (can_interfere(D_ptr, i_s, i_g, j_s, j_g)) {
              Vertex* vi_s = G_ref.V[i_s];
              Vertex* vi_g = G_ref.V[i_g];
              Vertex* vj_s = G_ref.V[j_s];
              Vertex* vj_g = G_ref.V[j_g];

              int independent_cost = D_ptr->get(i_g, i_s) + D_ptr->get(j_g, j_s);
              int joint_cost = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g);
              int d_h = joint_cost - independent_cost;
              if (d_h > 0)
                result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)d_h});
            }
          }
          return result;
        }));
      }

      for (auto& fut : futures) {
        auto [entries, evaluated] = fut.get();
        count_evaluated += evaluated;
        count_interfering += entries.size();
        append_entries(tmp_path, entries);
      }

      std::filesystem::rename(tmp_path, final_path);
    }
  }

  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
  std::cout << "Total 4-tuples evaluated: " << count_evaluated << std::endl;
  std::cout << "Pairs flagged for A*:     " << count_interfering << std::endl;
}

void PairWiseDB::construct_for_instance_only_goals(const Config& goals) {
  const int K = (int)goals.size();

  long long total_outer = (long long)K * (K - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  ThreadPool pool(NUM_OF_THREADS);

  for (int agent1 = 0; agent1 < K; ++agent1) {
    for (int agent2 = agent1 + 1; agent2 < K; ++agent2, ++outer_idx) {
      int pct = (int)(outer_idx * 100000 / total_outer);
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000;
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }
      int i_g = std::min(goals[agent1]->id, goals[agent2]->id);
      int j_g = std::max(goals[agent1]->id, goals[agent2]->id);

      std::string tmp_path   = DB_PATH + name + "/tmp_" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = DB_PATH + name + "/"     + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories(DB_PATH + name);
      { std::ofstream touch(tmp_path, std::ios::binary); }

      std::vector<std::future<ThreadResult>> futures;

      for (int j_s = 0; j_s < (int)G->V.size(); ++j_s) {
        if (j_s == j_g || j_s == i_g) continue;
        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          int i_s = i_g;
          if (D_ptr->get(j_s, i_g) + D_ptr->get(i_g, j_g) != D_ptr->get(j_s, j_g)) return result;
          Vertex* vi_s = G_ref.V[i_s];
          Vertex* vi_g = G_ref.V[i_g];
          Vertex* vj_s = G_ref.V[j_s];
          Vertex* vj_g = G_ref.V[j_g];
          if (has_alternative_path(D_ptr, vi_g, vj_s, vj_g)) return result;
          int dh = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g) - D_ptr->get(j_s, j_g) - D_ptr->get(i_s, i_g);
          if (dh != 0)
            result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)dh});
          return result;
        }));
      }

      for (int i_s = 0; i_s < (int)G->V.size(); ++i_s) {
        if (i_s == i_g || i_s == j_g) continue;
        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          int j_s = j_g;
          if (D_ptr->get(i_s, j_g) + D_ptr->get(j_g, i_g) != D_ptr->get(i_s, i_g)) return result;
          Vertex* vi_s = G_ref.V[i_s];
          Vertex* vi_g = G_ref.V[i_g];
          Vertex* vj_s = G_ref.V[j_s];
          Vertex* vj_g = G_ref.V[j_g];
          if (has_alternative_path(D_ptr, vj_g, vi_s, vi_g)) return result;
          int dh = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g) - D_ptr->get(j_s, j_g) - D_ptr->get(i_s, i_g);
          if (dh != 0)
            result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)dh});
          return result;
        }));
      }

      std::vector<BinEntry> all_entries;
      for (auto& fut : futures) {
        auto [entries, evaluated] = fut.get();
        all_entries.insert(all_entries.end(), entries.begin(), entries.end());
      }
      std::sort(all_entries.begin(), all_entries.end(), [](const BinEntry& a, const BinEntry& b) {
        if (a.i_start != b.i_start) return a.i_start < b.i_start;
        return a.j_start < b.j_start;
      });
      append_entries(tmp_path, all_entries);
      std::filesystem::rename(tmp_path, final_path);
    }
  }
  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
}
