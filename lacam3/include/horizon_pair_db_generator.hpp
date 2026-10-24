#pragma once

#include "mdd.hpp"
#include "horizon_pair_db.hpp"
#include "horizon_pair_db_file.hpp"
#include <fstream>


// Offline builder for the Horizon Pair DB. Generates MDDs for every
// (start, goal) pair, computes pairwise conflict penalties, and serializes
// the result via generate_and_save() to be loaded at runtime by HorizonPairDB.
struct HorizonPairDBGenerator {

  // Number of steps ahead we are aware of
  int HORIZON;
  // Max detour MDD allows beyond optimal paths
  int MARGIN;

  // MDD::frontiers[0] is the start vertex (time 0), hence the +1.
  inline int MDD_SIZE() const { return HORIZON + 1; }

  Graph* G;
  DistTable* D;
  size_t V_SIZE;
  std::string name;

  HorizonPairDBGenerator(Graph* _G, std::string _name, int _horizon, int _margin);

  uint32_t mdd_count = 0;

  // The index is the MDD id.
  std::vector<MDD> mdd_by_id;

  //Index is v->id * V_SIZE + g->id;
  std::vector<uint32_t> mdd_id_by_v_g;

  // This is an index from a tuple (int time, int vertex_id) to an MDD
  // where the index is t * (G->V.size()) + vertex_id.
  std::vector<std::vector<uint32_t>> mdd_by_t_s;

  MDD_Penalty calculate_penalty(MDD& mdd1, uint32_t mdd2_id);

  void generate_mdds();

  // Processes MDDs in memory-bounded bulks. For each MDD it discovers
  // overlapping candidates and computes both sync penalties (only for
  // other_mdd_id > mdd_id) and async (time-shifted) penalties, streaming
  // each bulk to <name>_sync_conflicts.tmp and <name>_async_conflicts.tmp.
  void generate_all_conflicts_in_bulks(int bulk_size = 10000);

  void write_header(std::ofstream& out, const HorizonPairDBFileHeader& header);

  void write_mdd_by_id_section(std::ofstream& out);

  void write_mdd_id_by_v_g_section(std::ofstream& out);

  bool generate_and_save();

};
