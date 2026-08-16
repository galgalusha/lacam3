#pragma once

#include "graph.hpp"
#include "dist_table.hpp"
#include "instance.hpp"
#include "pair_wise_db.hpp"

#include <absl/container/flat_hash_map.h>
#include <unordered_set>

struct PairWiseHeuristic {
  Graph *G;
  std::string name;
  absl::flat_hash_map<PairKey, absl::flat_hash_map<PairKey, uint16_t>> pair_data;

  PairWiseHeuristic(Graph *G, std::string _name);
  void construct();
  void construct_for_instance(const Config& goals);
  void load(Instance* ins);
  PairDHTable get_pair(int g1, int g2);
  bool can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal);
  bool has_alternative_path(DistTable* D, Vertex* blocked, Vertex* v_s, Vertex* v_g);
  std::unordered_set<PairKey> get_keys_from_files();
  static void test();
  void construct_for_instance_only_goals(const Config& goals);
  void construct_for_instance_only_goals_debug();

};