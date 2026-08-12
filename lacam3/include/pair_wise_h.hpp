#pragma once

#include "graph.hpp"
#include "dist_table.hpp"

struct PairWiseHeuristic {
  Graph *G;

  PairWiseHeuristic(Graph *G);
  void construct();
  static bool can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal);
  static void test();

};