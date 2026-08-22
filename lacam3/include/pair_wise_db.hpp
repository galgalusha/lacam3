#pragma once

#include "graph.hpp"
#include "dist_table.hpp"
#include "instance.hpp"
#include "pair_wise_bin.hpp"
#include "thread_pool.hpp"

#include <absl/container/flat_hash_map.h>
#include <array>
#include <cstdint>
#include <unordered_set>
#include <mutex>
#include <vector>


constexpr int NUM_OF_THREADS = 8;
constexpr char* DB_PATH = "/home/galko/dev/mapf_db/";
constexpr size_t MAX_VERTICES = 1024;


// Shared state for construct algorithms (defined in pair_wise_construct_algo.cpp)
extern thread_local std::vector<int> t_closed_g;
extern thread_local std::vector<uint32_t> t_closed_gen;
extern thread_local uint32_t t_current_gen;
extern std::vector<uint32_t> shared_zero_dh_gen;
extern uint32_t shared_zero_current_gen;
extern std::vector<std::pair<Vertex*, Vertex*>> shared_bfs_q;


// Free functions defined in pair_wise_construct_algo.cpp
int joint_astar(DistTable* D, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal);
void populate_zero_dh_by_bfs(DistTable* D, Vertex* i_goal, Vertex* j_goal);

using GoalsKey = uint32_t;

inline GoalsKey to_goals_key(uint16_t lo, uint16_t hi) {
  return ((uint32_t)lo << 16) | hi;
}


#pragma pack(push, 1)
struct RangeEntry {
  uint16_t j_start;
  uint8_t  range;
  uint8_t  dh;
};
#pragma pack(pop)


using StartArrays = std::array<std::vector<RangeEntry>, MAX_VERTICES>;
using PairData = absl::flat_hash_map<GoalsKey, StartArrays>;

/**
 * This is a database for multi agent path finding (MAPF) instance
 * that for each pair of agents (i, j) where each agent has a fixed
 * goal vertex id so that (i_goal, j_goal) never changes during the execution
 * of an MAPF planner and (i_start, j_start) are the current locations of the agents
 * which changes.
 * The goal of the DB is to provide a dh value for a given (i_goal, j_goal, i_start, j_start)
 * arguments.
 * 
 * ## DB size limitation 
 * This DB is limited to a map of size up to 32x32, meaning, vertex id is 
 * between 0 to 1023.
 * 
 * ## bin files
 * The construct methods generate bin files in a folder (DB_PATH + name).
 * This is a one time generation of the DB on the disk.
 * The name of a bin file is "{lo}_{hi}.bin" where lo is the min of (i_goal, j_goal)
 * and hi is the max of (i_goal, j_goal).
 * The bin file content is a series of BinEntry entries ordered lexicographically by
 * (i_start, j_start). Example of bin file content viewed as a table:
 * 
 * i_start  j_start   dh
 * 2        32        1
 * 4        0         2
 * 4        1         2
 * 4        2         2
 * 4        3         2
 * 4        29        2
 * 4        30        2
 * 4        31        2
 *
 * ## storage in RAM (pair_data)
 * The load method reads all the bin files into memory in a compacted way.
 * Each bin file, represented by (lo, hi) is a 32 bit GoalsKey.
 * pair_data is a map from each "file" (GoalsKey) to the file content StartArrays.
 * For every i_start, StartArray has a vector RangeEntry entries where a range entry
 * combines a sequence of BinEntry items that share the same i_start and same dh
 * and their j_start falls into a range that begins in RangeEntry::j_start to 
 * (RangeEntry::jstart + RangeEntry::range).
 * Example of the previos bin file content as pair_data:
 * 
 * pair_data[goals_key][2]
 *   { j_start=32, range=1, dh=1 }
 * 
 * pair_data[goals_key][4]
 *   { j_start=0 , range=4, dh=2 }
 *   { j_start=29, range=3, dh=2 }
 * 
 * It is important that each vector of RangeEntry items will be sorted by j_start
 * to allow O(log(n)) lookup when looking for a j_start entry.
 */
struct PairWiseDB {
  static int RADIUS;
  Graph *G;
  std::string name;
  DistTable* D;
  PairData pair_data;

  PairWiseDB(Graph *G, std::string _name);
  void construct();
  void construct_for_instance(const Config& goals);
  void load_all(Instance* ins);
  void load_bin_file(uint16_t lo, uint16_t hi);
  uint8_t get(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) const;
  void integration_test();
  bool can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal);
  bool has_alternative_path(DistTable* D, Vertex* blocked, Vertex* v_s, Vertex* v_g);
  std::unordered_set<PairKey> get_keys_from_files();
  static void test();
  void construct_for_instance_only_goals(const Config& goals);
  std::string bin_file_name(uint16_t lo, uint16_t hi) {
    return DB_PATH + name + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin";
  };
  void write_bin2_files();
  void load_all2(Instance* ins);
  void load_bin2_file(uint16_t lo, uint16_t hi);
  void test_interactive(Instance* ins);
};