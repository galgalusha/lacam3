#pragma once

#include "graph.hpp"
#include "dist_table.hpp"
#include "instance.hpp"
#include "pair_wise_db.hpp"

#include <absl/container/flat_hash_map.h>
#include <array>
#include <cstdint>
#include <unordered_set>

struct PairWiseHeuristic {
  Graph *G;
  std::string name;
  DistTable* D;
  std::array<absl::flat_hash_map<uint32_t, uint8_t>, 256> pair_data;

  PairWiseHeuristic(Graph *G, std::string _name);
  void construct();
  void construct_for_instance(const Config& goals);
  void load_all(Instance* ins);
  void load_some(Instance* ins, int margin);
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