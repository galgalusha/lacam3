#pragma once

#include "mdd.hpp"
#include <absl/container/flat_hash_map.h>


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

  // This maps an MDD id to its conflicting MDDs so that conflicts[id1][id2]
  // only contains an entry if there is a conclict with a penalty > 0. The
  // value of the entry is the penalty.
  // It is assumed that id1 < id2 so we don't need to store conflicts[50][10] which
  // should have the same value as conflicts[10][50]
  std::vector<absl::flat_hash_map<uint32_t, uint8_t>> conflicts;

  void generate_mdds();

  void generate_conflicts();

  uint8_t get_conflict_penalty(MDD& mdd1, MDD& mdd2);

  // Prompts the user for (v_i, g_i), renders the resulting MDD, and repeats
  // indefinitely until the process is killed.
  void interactive_mdd_test();

};
