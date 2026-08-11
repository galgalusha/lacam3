#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"

struct PairWiseHeuristic {
  const Instance *ins;
  Graph *G;
  DistTable D_goals;
  DistTable D_starts;

  PairWiseHeuristic(const Instance &ins);
  void construct();
  bool can_interfere(int i, int j);
  static void test();

};