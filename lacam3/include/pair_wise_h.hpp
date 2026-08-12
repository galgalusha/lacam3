#pragma once

#include "graph.hpp"
#include "dist_table.hpp"
#include "instance.hpp"
#include "pair_wise_db.hpp"

#include <absl/container/flat_hash_map.h>

struct PairWiseHeuristic {
  Graph *G;
  absl::flat_hash_map<PairKey, absl::flat_hash_map<PairKey, uint16_t>> pair_data;

  PairWiseHeuristic(Graph *G);
  void construct();
  void load(Instance* ins);
  PairDHTable get_pair(int g1, int g2);
  static bool can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal);
  static void test();

};