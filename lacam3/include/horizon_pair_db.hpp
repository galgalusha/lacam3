#pragma once

#include "mdd.hpp"


struct HorizonPairDB {
  static const int HORIZON;
  Graph* G;
  DistTable* D;

  HorizonPairDB(Graph* _G);

  std::vector<MDD> all_mdds;

  // This is an index from a tuple (int time, int vertex_id) to an MDD
  // where the index is t * (G->V.size()) + vertex_id.
  std::vector<std::vector<MDD*>> idx_mdds;

  void generate_mdds();

  void generate_conflicting_pairs();

};
