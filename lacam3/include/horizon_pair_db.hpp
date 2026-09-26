#pragma once

#include "mdd.hpp"


struct HorizonPairDB {
  static const int HORIZON;
  Graph* G;
  DistTable* D;

  HorizonPairDB(Graph* _G);

  uint32_t mdd_count = 0;

  // from id to MDD
  std::vector<MDD> mdd_by_id;

  // agent_id = v_i * V_SIZE + g_i;
  std::vector<uint32_t> mdd_id_by_agent;

  // This is an index from a tuple (int time, int vertex_id) to an MDD
  // where the index is t * (G->V.size()) + vertex_id.
  std::vector<std::vector<uint32_t>> mdd_by_t_s;

  void generate_mdds();

  void generate_conflicting_pairs();

  // Prompts the user for (v_i, g_i), renders the resulting MDD, and repeats
  // indefinitely until the process is killed.
  void interactive_mdd_test();

};
