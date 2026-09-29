#pragma once

#include "mdd.hpp"
#include <absl/container/flat_hash_map.h>
#include <fstream>


struct HorizonPairDB {

  struct MDD_Penalty {
    uint32_t mdd_id;
    uint8_t penalty;
  };


  static const int HORIZON;
  Graph* G;
  DistTable* D;
  size_t V_SIZE;
  std::string name;

  HorizonPairDB(Graph* _G, std::string _name);

  uint32_t mdd_count = 0;

  // The index is the MDD id.
  std::vector<MDD> mdd_by_id;

  //Index is v->id * V_SIZE + g->id;
  std::vector<uint32_t> mdd_id_by_v_g;

  // This is an index from a tuple (int time, int vertex_id) to an MDD
  // where the index is t * (G->V.size()) + vertex_id.
  std::vector<std::vector<uint32_t>> mdd_by_t_s;

  // This maps an MDD id to its conflicting MDDs so that conflicts[id1][id2]
  // only contains an entry if there is a conclict with a penalty > 0. The
  // value of the entry is the penalty.
  // It is assumed that id1 < id2 so we don't need to store conflicts[50][10] which
  // should have the same value as conflicts[10][50]
  std::vector<absl::flat_hash_map<uint32_t, uint8_t>> conflicts;

  // Similar to confclits, but per MDD, its a vector of entries rather than a map.
  // Also, it should be symmetrical so if mdd1 contains mdd2, mdd2 shall contain mdd1.
  std::vector<std::vector<MDD_Penalty>> penalties;

  void generate_mdds();

  void generate_conflicts();

  uint8_t get_conflict_penalty(MDD& mdd1, MDD& mdd2);

  // Prompts the user for (v_i, g_i), renders the resulting MDD, and repeats
  // indefinitely until the process is killed.
  void interactive_mdd_test();

  // Serializes mdd_by_id, mdd_id_by_v_g and conflicts to
  // ROOT_FOLDER + name + ".mdd_db". Returns false on I/O failure.
  // mdd_by_t_s is not persisted since it is only a temporary build utility.
  bool save_to_file();

  // Populates mdd_by_id, mdd_id_by_v_g and conflicts from the file written by
  // save_to_file(), running sanity checks along the way. Returns false on
  // I/O failure or if a sanity check fails.
  bool load_from_file();

  // Header of the .mdd_db file: MDD count plus the byte offset of each section.
  struct FileHeader {
    uint32_t num_mdds;
    uint64_t offset_mdd_by_id;
    uint64_t offset_mdd_id_by_v_g;
    uint64_t offset_conflicts;
  };

  void write_header(std::ofstream& out, const FileHeader& header);
  bool read_header(std::ifstream& in, FileHeader& header);

  void write_mdd(std::ofstream& out, const MDD& mdd);
  bool read_mdd(std::ifstream& in, MDD& mdd);

  void write_mdd_by_id_section(std::ofstream& out);
  bool read_mdd_by_id_section(std::ifstream& in);

  void write_mdd_id_by_v_g_section(std::ofstream& out);
  bool read_mdd_id_by_v_g_section(std::ifstream& in);

  void write_conflicts_section(std::ofstream& out);
  bool read_conflicts_section(std::ifstream& in);

  inline uint8_t get_penalty(uint32_t mdd_id1, uint32_t mdd_id2) {
    uint32_t min_mdd_id = mdd_id1 < mdd_id2 ? mdd_id1 : mdd_id2;
    uint32_t max_mdd_id = mdd_id1 > mdd_id2 ? mdd_id1 : mdd_id2;
    auto& map = conflicts[min_mdd_id];
    auto entry = map.find(max_mdd_id);
    return entry == map.end() ? 0 : entry->second;
  }

  inline uint8_t get_penalty(int v1, int g1, int v2, int g2) {
    uint32_t agent1 = v1 * V_SIZE + g1;
    uint32_t agent2 = v2 * V_SIZE + g2;
    uint32_t mdd_id1 = mdd_id_by_v_g[agent1];
    uint32_t mdd_id2 = mdd_id_by_v_g[agent2];
    return get_penalty(mdd_id1, mdd_id2);
  }

  static void test_db_1();
};
