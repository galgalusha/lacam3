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
  std::vector<std::unordered_map<int, double>> oracle_tie_breakers;  // per-agent recorded tie-breakers: [agent_id][vertex_id]
  std::vector<double> dh_values;
  std::vector<int> pair_distances;
  // per-vertex candidates within RADIUS: non-null, in-bounds, Manhattan <= RADIUS
  std::vector<std::vector<Vertex*>> radial_neighbors;

  // swap, used in the LaCAM* paper
  bool flg_swap;
  bool record_dh_errors;

  // scatter
  Scatter *scatter;

  // --- dh prediction accuracy tracking ---
  // a speculative dh estimate for agent i's candidate move u_i, w.r.t. neighbor agent j
  static constexpr int NUM_DH_VARIANTS = 7;  // dh1..dh7

  struct DHPredictionRecord {
    Vertex *u_i = nullptr;
    bool has_future = false;
    double future_dh[NUM_DH_VARIANTS] = {};  // dh1..dh7, from get_future_penalty (radius == 1)
    bool has_wait = false;
    int wait_dh = 0;         // from direct pair_db lookup (radius != 1)
  };
  // keyed by (i, j) ordered agent pair, one slot per agent i's candidate move index k
  std::unordered_map<int64_t, std::array<DHPredictionRecord, 5>> dh_prediction_cache;

  static constexpr int NUM_WAIT_ERROR_BUCKETS = 12;    // exact errors 0..10, plus one overflow (>10)
  static constexpr int NUM_FUTURE_ERROR_BUCKETS = 13;  // 0-.25,.25-.5,.5-1,1-2,...,9-10, plus overflow (>=10)
  long long wait_error_buckets[NUM_WAIT_ERROR_BUCKETS] = {};
  long long future_error_buckets[NUM_FUTURE_ERROR_BUCKETS] = {};

  // breakdown of the first 4 future error buckets: [was_wait][error_bucket][congestion_bucket]
  static constexpr int NUM_BREAKDOWN_ERROR_BUCKETS = 4;       // 0-.25,.25-.5,.5-1,1-2
  static constexpr int NUM_CONGESTION_BUCKETS = 4;            // 0-.25,.25-.5,.5-.75,.75-1
  long long congestion_table[2][NUM_BREAKDOWN_ERROR_BUCKETS][NUM_CONGESTION_BUCKETS] = {};

  // breakdown of wait_dh prediction success/failure by congestion: [is_error][congestion_bucket]
  long long wait_congestion_table[2][NUM_CONGESTION_BUCKETS] = {};

  // breakdown of future_dh (dhN) prediction success/failure by congestion: [dh_variant][is_error][congestion_bucket]
  // is_error threshold is 0.25 for the first table, 0.5 for the second
  long long future_congestion_table_025[NUM_DH_VARIANTS][2][NUM_CONGESTION_BUCKETS] = {};
  long long future_congestion_table_05[NUM_DH_VARIANTS][2][NUM_CONGESTION_BUCKETS] = {};

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
  FuturePenalty get_future_penalty(const int i, const int j, Vertex* u_i, Vertex* u_j, const Config& Q_from, const Config& Q_to);
  // cheap pre-check: true iff get_future_penalty(i, j, u_i, u_j, ...) is guaranteed to be 0
  bool is_surely_zero(const int i, const int j, Vertex* u_i, Vertex* u_j, const Config& Q_from, const Config& Q_to);

  int get_move_risk(int j, Vertex* u_j, const Config& Q_to, int agent_i);

  // fraction of v's radial neighbors that are currently occupied
  double get_congestion(Vertex* v);

  void evaluate_dh_predictions(const Config &Q_from, const Config &Q_to);
  void print_dh_error_buckets();

  inline int pair_key(int i, int j) { 
    int lo = std::min(i, j), hi = std::max(i, j);
    return hi * N + lo;
  }

  inline int64_t dh_cache_key(int i, int j) const { return (int64_t)i * N + j; }
};
