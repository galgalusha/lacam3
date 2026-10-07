#pragma once

#include "mdd.hpp"
#include "horizon_pair_db.hpp"
#include "horizon_pair_db_file.hpp"
#include <fstream>


// Offline builder for the Horizon Pair DB. Generates MDDs for every
// (start, goal) pair, computes pairwise conflict penalties, and serializes
// the result via save_to_file() to be loaded at runtime by HorizonPairDB.
struct HorizonPairDBGenerator {
  using MDD_Penalty = HorizonPairDB::MDD_Penalty;

  static int HORIZON;
  static int MDD_SIZE;
  Graph* G;
  DistTable* D;
  size_t V_SIZE;
  std::string name;

  HorizonPairDBGenerator(Graph* _G, std::string _name);

  uint32_t mdd_count = 0;

  // The index is the MDD id.
  std::vector<MDD> mdd_by_id;

  //Index is v->id * V_SIZE + g->id;
  std::vector<uint32_t> mdd_id_by_v_g;

  // This is an index from a tuple (int time, int vertex_id) to an MDD
  // where the index is t * (G->V.size()) + vertex_id.
  std::vector<std::vector<uint32_t>> mdd_by_t_s;

  // Stores only positive-penalty pairs with mdd_id < entry.mdd_id.
  std::vector<std::vector<MDD_Penalty>> penalties;

  // Same as penalties, only that the agent of MDD1 already made its move
  // from anywhere (we don't know where from) to MDD1.frontiers[0][0] and
  // MDD2 did not yet made a move.  
  std::vector<std::vector<MDD_Penalty>> time_shifted_penalties;

  // flagged_for_conflict[mdd_id] holds the ids (> mdd_id) of other MDDs whose
  // frontiers overlap in space-time with mdd_id's, as discovered by
  // flag_mdds_for_conflicts(). No penalties are computed at this stage.
  std::vector<std::vector<uint32_t>> flagged_for_conflict;

  // For unit tests
  uint8_t get_penalty(Vertex* v1, Vertex* g1, Vertex* v2, Vertex* g2);
  uint8_t get_constrained_move_penalty(Vertex* v1_0, Vertex* v1_1, Vertex* g1, Vertex* v2, Vertex* g2);

  void generate_mdds();

  // Time-shifts over MDD frontiers to discover potential overlaps and
  // populates flagged_for_conflict. Does not compute any penalties.
  void flag_mdds_for_conflicts();

  // Consumes flagged_for_conflict, computing the actual penalty for each
  // flagged pair via check_joint_mdd_conflict and populating penalties.
  void generate_sync_time_conflicts();

  void generate_constrained_move_conflicts();

  uint8_t calculate_sync_time_penalty(MDD& mdd1, MDD& mdd2);
  uint8_t calculate_time_shifted_penalty(MDD& constrained_mdd, MDD& other_mdd);

  // Prompts the user for (v_i, g_i), renders the resulting MDD, and repeats
  // indefinitely until the process is killed.
  void interactive_mdd_test();

  // Serializes mdd_by_id, mdd_id_by_v_g and penalties to
  // ROOT_FOLDER + name + ".mdd_db". Returns false on I/O failure.
  // mdd_by_t_s and flagged_for_conflict are not persisted since they are only
  // temporary build utilities.
  bool save_to_file();

  void write_header(std::ofstream& out, const HorizonPairDBFileHeader& header);

  void write_mdd(std::ofstream& out, const MDD& mdd);

  void write_mdd_by_id_section(std::ofstream& out);

  void write_mdd_id_by_v_g_section(std::ofstream& out);

  void write_conflicts_section(std::ofstream& out);

  void write_constrained_move_conflicts_section(std::ofstream& out);

  static void test_db_1();
  static void test_db_time_shift();
  static void test_db_time_shift_2();
};
