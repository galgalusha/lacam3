#pragma once

#include "mdd.hpp"
#include "horizon_pair_db_file.hpp"
#include <fstream>


// Runtime view of the Horizon Pair DB. Populated by load_from_file() from a
// file produced offline by HorizonPairDBGenerator::save_to_file(), and used
// by PIBT to look up MDD ids and conflict penalties. Holds no generation
// state (no DistTable, no temporary build indices).
struct HorizonPairDB {

  struct MDD_Penalty {
    uint32_t mdd_id;
    uint8_t penalty;
  };

  Graph* G;
  size_t V_SIZE;
  std::string name;

  HorizonPairDB(Graph* _G, std::string _name);

  uint32_t mdd_count = 0;

  // The index is the MDD id. Only frontiers are needed at runtime (e.g. PIBT
  // reads frontiers[1] to enumerate an agent's next-step options).
  std::vector<MDD> mdd_by_id;

  //Index is v->id * V_SIZE + g->id;
  std::vector<uint32_t> mdd_id_by_v_g;

  // Similar to confclits, but per MDD, its a vector of entries rather than a map.
  // Also, it should be symmetrical so if mdd1 contains mdd2, mdd2 shall contain mdd1.
  std::vector<std::vector<MDD_Penalty>> penalties;

  // 
  std::vector<std::vector<MDD_Penalty>> constrained_move_penalties;

  // Populates mdd_by_id, mdd_id_by_v_g and conflicts from the file written by
  // HorizonPairDBGenerator::save_to_file(), running sanity checks along the
  // way. Returns false on I/O failure or if a sanity check fails.
  bool load_from_file();

  bool read_header(std::ifstream& in, HorizonPairDBFileHeader& header);

  bool read_mdd(std::ifstream& in, MDD& mdd);

  bool read_mdd_by_id_section(std::ifstream& in);

  bool read_mdd_id_by_v_g_section(std::ifstream& in);

  bool read_conflicts_section(std::ifstream& in);

  bool read_move_constrained_conflicts_section(std::ifstream& in);

  inline uint8_t get_penalty(uint32_t mdd_id1, uint32_t mdd_id2) {
    for (auto& entry : penalties[mdd_id1]) {
      if (entry.mdd_id == mdd_id2) return entry.penalty;
    }
    return 0;
  }

  inline uint8_t get_penalty(int v1, int g1, int v2, int g2) {
    uint32_t agent1 = v1 * V_SIZE + g1;
    uint32_t agent2 = v2 * V_SIZE + g2;
    uint32_t mdd_id1 = mdd_id_by_v_g[agent1];
    uint32_t mdd_id2 = mdd_id_by_v_g[agent2];
    return get_penalty(mdd_id1, mdd_id2);
  }

  static void integration_test1();
};
