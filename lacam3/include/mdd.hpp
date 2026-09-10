#pragma once

#include "../include/graph.hpp"
#include "../include/dist_table.hpp"
#include "../include/instance.hpp"

struct MDD {
  int agent_id;

  MDD(int _agent_id) : agent_id(_agent_id) {}

  // an index in frontiers represents time so that frontiers[t]
  // is an MDD frontier, meaning, all the vertices that are on one of the
  // agents shortests paths at time t where frontiers[0] is the contains only
  // only one vertex, the vertex the agent is at time 0.
  // The frontiers only go on in time until a specific horizon which is
  // frontiers.size(), so they do not necessarilly reach the goal of the agent.
  std::vector<std::vector<Vertex*>> frontiers;

  void populate(DistTable* D, Vertex* v_i, Vertex* v_g, int horizon);

  void render(Instance* ins);
  
};

