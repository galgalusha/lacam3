/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

const bool TO_GOALS = true;
const bool TO_STARTS = false;

struct DistTable {
  const int K;  // number of vertices
  std::vector<std::vector<int>>
      table;  // distance table, index: agent-id & vertex-id
  std::vector<std::queue<Vertex *>> OPEN;  // search queue

  int get(const int i, const int v_id);   // agent, vertex-id
  int get(const int i, const Vertex *v);  // agent, vertex

  DistTable(const Instance &ins, bool toward_goal = true);
  DistTable(const Instance *ins, bool toward_goal = true);

  void setup(const Instance *ins, bool toward_goal = true);  // initialization
};
