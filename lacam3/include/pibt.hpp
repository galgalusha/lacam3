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
#include "pair_wise_db.hpp"
#include <unordered_map>

struct PIBT {

  struct FuturePenalty {
    double dh1;
    double dh2;
    double dh3;
    double dh4;
    double dh5;
    double dh6;
    double dh7;
    double dh_min;

    double certainty;

    FuturePenalty(double dh) { dh_min = dh1 = dh2 = dh3 = dh4 = dh5 = dh6 = dh7 = dh; certainty = 1.0; }    
    FuturePenalty() { dh_min = dh1 = dh2 = dh3 = dh4 = dh5 = dh6 = dh7 = 0.0; }    
  };


  static PairWiseDB* pair_db;
  static bool FIXED_TIE;

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
  // per-vertex candidates within RADIUS: non-null, in-bounds, Manhattan <= RADIUS
  std::vector<std::vector<Vertex*>> radial_neighbors;

  // swap, used in the LaCAM* paper
  bool flg_swap;

  // scatter
  Scatter *scatter;


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
};
