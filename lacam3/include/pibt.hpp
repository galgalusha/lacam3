/*
 * implementation of PIBT
 *
 * references:
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. Keisuke Okumura, Manao Machida, Xavier Défago & Yasumasa Tamura.
 * Artificial Intelligence (AIJ). 2022.
 */
#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "scatter.hpp"
#include "utils.hpp"
#include "horizon_pair_db.hpp"
#include <unordered_map>

struct PIBT {

  static HorizonPairDB* pair_db;

  Config goals;
  const Graph* G;

  std::mt19937 MT;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTable *D;

  // specific to PIBT
  const int NO_AGENT;
  std::vector<int> occupied_now;                // for quick collision checking
  std::vector<int> occupied_next;               // for quick collision checking
  std::vector<std::array<Vertex *, 5>> C_next;  // next location candidates
  std::vector<double> tie_breakers;              // random values, used in PIBT
  std::vector<double> dh_values;

  // swap, used in the LaCAM* paper
  bool flg_swap;

  // scatter
  Scatter *scatter;

  // for fill_dh_values
  std::vector<uint32_t> visited_token; // Initialize with size N, filled with 0
  uint32_t current_evaluation_token;   // Initialize to 0  
  // The key is t * V_size + v->id and the value is a vector of agent IDs
  // whose MDDs overlap vertex v at time t.
  std::vector<std::vector<int>> agents_by_t_v; 
  std::vector<size_t> dirty_t_v_vectors;

  PIBT(const Graph *_G, Config _goals, DistTable *_D, int seed = 0, bool _flg_swap = true,
       Scatter *_scatter = nullptr);
  ~PIBT();

  void setup_pair_db();
  bool set_new_config(const Config &Q_from, Config &Q_to,
                      const std::vector<int> &order);
  bool set_new_config_internal(const Config &Q_from, Config &Q_to,
                      const std::vector<int> &order);
  bool funcPIBT(const int i, const Config &Q_from, Config &Q_to);
  int is_swap_required_and_possible(const int ai, const Config &Q_from,
                                    Config &Q_to);
  bool is_swap_required(const int pusher, const int puller,
                        Vertex *v_pusher_origin, Vertex *v_puller_origin);
  bool is_swap_possible(Vertex *v_pusher_origin, Vertex *v_puller_origin);

  void fill_dh_values(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start = 0);


  inline int pair_key(int i, int j) { 
    int lo = std::min(i, j), hi = std::max(i, j);
    return hi * N + lo;
  }

  inline int64_t dh_cache_key(int i, int j) const { return (int64_t)i * N + j; }

  inline uint32_t get_mdd_id(int v, int goal) {
    uint32_t agent_id_for_db = V_size * v + goal;
    return pair_db->mdd_id_by_agent[agent_id_for_db];
  }

  void register_agent_mdd(int agent_id, uint32_t mdd_id);

  template <typename Func>
  void for_each_other_agent_overlapping_with_mdd(uint32_t mdd_id, Func callback);

};
