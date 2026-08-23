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

using Kernel24 = std::array<uint8_t, 24>;

using Bin2Content = std::array<std::vector<RangeEntry>, MAX_VERTICES>;
using PairData = absl::flat_hash_map<GoalsKey, Bin2Content>;

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
 * ## bin2 files (compacted)
 * Each bin file (and respectedly bin2 file), represented by (lo, hi) is a 32 bit GoalsKey.
 * The content of each bin2 file is Bin2Content: an array of size MAX_VERTICES mapping
 * each i_start value (a vertex) to a vector of RangeEntry.
 * A single RangeEntry comresses a sequence of BinEntry items that share the same i_start and same dh
 * and their j_start falls into a range that begins in RangeEntry::j_start to 
 * (RangeEntry::jstart + RangeEntry::range).
 * Example of the previos bin file content as Bin2Content:
 * 
 * Bin2Content[goals_key][2]
 *   { j_start=32, range=1, dh=1 }
 * 
 * Bin2Content[goals_key][4]
 *   { j_start=0 , range=4, dh=2 }
 *   { j_start=29, range=3, dh=2 }
 * 
 * Each vector<RangeEntry> is sorted by j_start.
 * 
 * ## RAM storage
 * For compaction, rather than storing GoalsKey as a pair of goal vertices, we will store
 * them as pair of agent IDs because there are less agents than vertices and an agent
 * goal vertex never changes during MAPF execution.
 * Rather than using GoalsKey, we use KernelKey, a uint32_t calculated as follows:
 * kernel_key(i, j, i_s, j_s) = (lo_agent * N + hi_agent)) * MAX_VERTICES + lo_s
 * where i, j are agent IDs and i_s, j_s are their current vertex IDs (start),
 * lo_agent is the id of the agent with smaller goal vertex id. hi_agent is the other agent.
 * lo_s is the start vertex id of lo_agent and hi_s is the start vertex of the other agent.
 * We have a vector kernel_offsets of size N * N * MAX_VERTICES that is 
 * pre-allocated upon initialization. Since the DB is sparse, not every entry in kernel_offsets
 * will actually point to a real kernel. This is another compaction optimization.
 * Another vector, kernels, with an unknown size upon initialization, will store the actual
 * loaded kernels while kernel_offsets will contain the offsets in kernels for each kernel key,
 * so we simulate a vector of kernels where we can fetch a kernel as follows:
 * kernel = kernels[kernel_offsets[kernel_key]]
 * Given a kernel key, we want to load only the dh values for j_start vertices in radius of 3
 * from i_start. There are up to 24 such neighbor j_start vertices that we abstract as a
 * kernel: an array of uint_8 items of size 24.
 * While in the bin files, j_start is an absolute vertex id, in a kernel, an absolute j_start
 * is translated to one of 24 possible offsets from i_start (i_start is still an absolute vertex id).
 * Remember that a Vertex has an id and also x,y members.
 * We use get_offset_index(i_v, j_v) to get an index from 0 to 23.
 * The get method that returns dh is as follows (assuming N and ins are accessible):
 * uint8_t get(int i, int j, const Vertex* vi, const Vertex* vj) const {
 *   int i_g = ins->goals[i]->id;
 *   int j_g = ins->goals[j]->id;
 *   int lo       = i_g < j_g ? i    : j;
 *   int hi       = i_g < j_g ? j    : i;
 *   const Vertex* lo_v = i_g < j_g ? vi   : vj;
 *   const Vertex* hi_v = i_g < j_g ? vj   : vi;
 *   
 *   uint8_t idx_in_kernel = get_offset_index(lo_v, hi_v);
 *   if (idx_in_kernel == 255) return 0;
 *   
 *   uint32_t kernel_offset = kernel_offsets[(lo * N + hi) * MAX_VERTICES + lo_v->id];
 *   if (kernel_offset == UINT32_MAX) return 0;
 *   
 *   return kernels[kernel_offset][idx_in_kernel];
 * }
 */
struct PairWiseDB {
  static int RADIUS;
  Instance* ins;
  Graph *G;
  std::string name;
  DistTable* D;
  PairData pair_data; // deprecated. only used when loading bin files before writing bin2 files.
  std::vector<Kernel24> kernels;
  std::vector<uint32_t> kernel_offsets;
  int N = 0;

  PairWiseDB(Graph *G, std::string _name);
  void construct();
  void construct_for_instance(const Config& goals);
  void load_all(Instance* ins);
  void load_bin_file(uint16_t lo, uint16_t hi);
  uint8_t get_from_map(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) const;
  void load_kernels(Instance* ins);

  inline uint8_t get(int i, int j, Vertex* vi, Vertex* vj) const {
    int i_g = ins->goals[i]->id;
    int j_g = ins->goals[j]->id;
    int lo        = i_g < j_g ? i  : j;
    int hi        = i_g < j_g ? j  : i;
    Vertex* lo_v  = i_g < j_g ? vi : vj;
    Vertex* hi_v  = i_g < j_g ? vj : vi;
    uint8_t idx_in_kernel = get_offset_index(lo_v, hi_v);
    if (idx_in_kernel == 255) return 0;
    uint32_t kernel_offset = kernel_offsets[((size_t)lo * N + hi) * MAX_VERTICES + lo_v->id];
    if (kernel_offset == UINT32_MAX) return 0;
    return kernels[kernel_offset][idx_in_kernel];
  }
  
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

  inline uint8_t get_offset_index(Vertex* vi, Vertex* vj) const {
      static const uint8_t offset_map[7][7] = {
          {255, 255, 255,   0, 255, 255, 255},
          {255, 255,   1,   2,   3, 255, 255},
          {255,   4,   5,   6,   7,   8, 255},
          {  9,  10,  11, 255,  12,  13,  14}, // Center (3,3) is 255 (i == j)
          {255,  15,  16,  17,  18,  19, 255},
          {255, 255,  20,  21,  22, 255, 255},
          {255, 255, 255,  23, 255, 255, 255}
      };
      int dy = vj->y - vi->y + 3;
      int dx = vj->x - vi->x + 3;    
      if (dx < 0 || dx > 6 || dy < 0 || dy > 6) return 255;
      return offset_map[dy][dx];
  };
};

