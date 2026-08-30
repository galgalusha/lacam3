/*
 * heuristic definition
 */

#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"

struct Heuristic {
  const Instance *ins;
  DistTable *D;

  Heuristic(const Instance *_ins, DistTable *_D);
  int get(const Config &C);
  int get(const Config &C, const std::vector<bool> *modes);  // phase 2 agents contribute 0
};
