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

  virtual int get(const int i, const int v_id);   // agent, vertex-id
  virtual int get(const int i, const Vertex *v);  // agent, vertex
  virtual ~DistTable() = default;

  DistTable(const Instance &ins, bool toward_goal = true);
  DistTable(const Instance *ins, bool toward_goal = true);

  void setup(const Instance *ins, bool toward_goal = true);  // initialization

protected:
  explicit DistTable(int K);  // for subclasses: skips BFS setup
};

// Dispatches get() to D_prefix (phase 1) or D_real (phase 2) based on agent_modes.
struct DoubleModeDistTable : DistTable {
  DistTable *D_prefix;
  DistTable *D_real;
  const std::vector<bool> *agent_modes;

  DoubleModeDistTable(int K, DistTable *d_prefix, DistTable *d_real);
  void set_active_modes(const std::vector<bool> *modes);
  int get(const int i, const int v_id) override;
  int get(const int i, const Vertex *v) override;
};
