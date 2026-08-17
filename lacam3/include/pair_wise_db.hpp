#pragma once

#include "graph.hpp"
#include "dist_table.hpp"
#include "instance.hpp"
#include "pair_wise_bin.hpp"
#include "thread_pool.hpp"

#include <absl/container/flat_hash_map.h>
#include <array>
#include <cstdint>
#include <unordered_set>
#include <mutex>
#include <vector>

constexpr int NUM_OF_THREADS = 7;
constexpr char* DB_PATH = "/home/galko/dev/mapf_db/";

// Shared state for construct algorithms (defined in pair_wise_construct_algo.cpp)
extern thread_local std::vector<int> t_closed_g;
extern thread_local std::vector<uint32_t> t_closed_gen;
extern thread_local uint32_t t_current_gen;
extern std::vector<uint32_t> shared_zero_dh_gen;
extern uint32_t shared_zero_current_gen;
extern std::vector<std::pair<Vertex*, Vertex*>> shared_bfs_q;

// Free functions defined in pair_wise_construct_algo.cpp
int joint_astar(DistTable* D, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal);
void populate_zero_dh_by_bfs(DistTable* D, Vertex* i_goal, Vertex* j_goal);

// 40-bit key: 10 bits each for i_goal, j_goal, i_start, j_start (MSB to LSB)
using JointKey = uint64_t;

inline JointKey to_joint_key(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) {
  return ((JointKey)(i_goal  & 0x3FF) << 30) |
         ((JointKey)(j_goal  & 0x3FF) << 20) |
         ((JointKey)(i_start & 0x3FF) << 10) |
         ((JointKey)(j_start & 0x3FF));
}

struct PairWiseHeuristic {
  Graph *G;
  std::string name;
  DistTable* D;
  std::array<absl::flat_hash_map<uint32_t, uint8_t>, 256> pair_data;
  std::array<std::mutex, 256> pair_data_mtx;

  PairWiseHeuristic(Graph *G, std::string _name);
  void construct();
  void construct_for_instance(const Config& goals);
  void load_all(Instance* ins);
  void load_some(Instance* ins, int margin, int num_of_threads);
  void load_bin_file(uint16_t lo, uint16_t hi, const std::string& path);
  void load_bin_file_filtered(uint16_t lo, uint16_t hi, const std::string& path,
                               bool nearest_is_lo, const std::unordered_set<int>& vertex_set);
  std::unordered_set<int> bfs_with_margin(Vertex* start, Vertex* goal, int margin) const;
  uint8_t get(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) const;
  void integration_test();
  bool can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal);
  bool has_alternative_path(DistTable* D, Vertex* blocked, Vertex* v_s, Vertex* v_g);
  std::unordered_set<PairKey> get_keys_from_files();
  static void test();
  void construct_for_instance_only_goals(const Config& goals);

};