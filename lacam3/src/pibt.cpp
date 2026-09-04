#include "../include/pibt.hpp"

#include <cmath>

PairWiseDB* PIBT::pair_db = nullptr;
bool PIBT::FIXED_TIE = false;

const double DH_WEIGHT = 0.01;
const double RANDOM_TIE_WEIGHT = 0.1;
const double TRAPPED_PENALTY = INT_MAX;

const double WEIGHT_PRIORITIZED = 2.50;
const double WEIGHT_GOOD_MOVE   = 2.00;
const double WEIGHT_WAIT        = 1.50;
const double WEIGHT_BAD_MOVE    = 1.00;

static const double RADIUS_DECAY[] = {
    1.0,                // r = 0 (failsafe)
    1.0,                // r = 1
    0.75,               // r = 2
    0.5,                // r = 3
    0.25,               // r = 4
};

const double RISK_NUM_TO_PROB[6] = { 1.0, 0.8, 0.6, 0.4, 0.2, 0.0 };

// avg SoC=76.3 with gamma^2 and these params
// const double WEIGHT_PRIORITIZED = 2.50;
// const double WEIGHT_GOOD_MOVE   = 2.00;
// const double WEIGHT_WAIT        = 1.50;
// const double WEIGHT_BAD_MOVE    = 1.00;

const double BASE_DISCOUNT      = 0.50;


PIBT::PIBT(const Instance *_ins, DistTable *_D, int seed, bool _flg_swap,
           Scatter *_scatter)
    : ins(_ins),
      MT(std::mt19937(seed)),
      N(ins->N),
      V_size(ins->G->size()),
      D(_D),
      NO_AGENT(N),
      occupied_now(V_size, NO_AGENT),
      occupied_next(V_size, NO_AGENT),
      C_next(N, std::array<Vertex *, 5>()),
      tie_breakers(V_size, 0),
      oracle_tie_breakers(N),
      dh_values(V_size, 0),
      flg_swap(_flg_swap),
      scatter(_scatter),
      pair_distances(N*N),
      radial_neighbors(ins->G->V.size())
{
  if (pair_db) setup_pair_db();
}

PIBT::~PIBT() {
  if (pair_db != nullptr) print_dh_error_buckets();
}

void PIBT::setup_pair_db() {
  const int width = ins->G->width;
  const int height = ins->G->height;
  const int r = PairWiseDB::RADIUS;
  for (Vertex* u_i : ins->G->V) {
    auto& vec = radial_neighbors[u_i->id];
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) + std::abs(dy) > r) continue;
        const int nx = u_i->x + dx;
        const int ny = u_i->y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        Vertex* u_j = ins->G->U[ny * width + nx];
        if (u_j == nullptr || pair_db->D->get(u_i->id, u_j->id) > r) continue;
        vec.push_back(u_j);
      }
    }
  }
}


bool PIBT::set_new_config(const Config &Q_from, Config &Q_to,
                          const std::vector<int> &order)
{
  bool success = true;
  pair_distances.assign(N * N,  -1);
  dh_prediction_cache.clear();
  // setup cache & constraints check
  for (auto i = 0; i < N; ++i) {
    // set occupied now
    occupied_now[Q_from[i]->id] = i;

    // set occupied next
    if (Q_to[i] != nullptr) {
      // vertex collision
      if (occupied_next[Q_to[i]->id] != NO_AGENT) {
        success = false;
        break;
      }
      // swap collision
      auto j = occupied_now[Q_to[i]->id];
      if (j != NO_AGENT && j != i && Q_to[j] == Q_from[i]) {
        success = false;
        break;
      }
      occupied_next[Q_to[i]->id] = i;
    }
  }

  if (FIXED_TIE) {
    for (auto i = 0; i < N; i++) {
      auto neighbors = Q_from[i]->neighbor;
      oracle_tie_breakers[i].clear();
      for (Vertex* u : neighbors) {
        oracle_tie_breakers[i][u->id] = RANDOM_TIE_WEIGHT * get_random_float(MT);
      }      
    }
  }

  if (success) {
    for (auto i : order) {
      if (Q_to[i] == nullptr && !funcPIBT(i, Q_from, Q_to)) {
        success = false;
        break;
      }
    }
  }

  if (success) evaluate_dh_predictions(Q_to);

  // cleanup
  for (auto i = 0; i < N; ++i) {
    occupied_now[Q_from[i]->id] = NO_AGENT;
    if (Q_to[i] != nullptr) occupied_next[Q_to[i]->id] = NO_AGENT;
  }

  return success;
}

int PIBT::get_move_risk(int j, Vertex* u_j, const Config& Q_to, int agent_i) {
  int risk = 0;
  
  // Check the cell itself
  int k = occupied_now[u_j->id];
  if (k != NO_AGENT && k != j && k != agent_i && Q_to[k] == nullptr) risk++; 
  
  // Check neighbors
  for (auto u_k : u_j->neighbor) {
    k = occupied_now[u_k->id];
    if (k != NO_AGENT && k != j && k != agent_i && Q_to[k] == nullptr) risk++;
  }
  return risk;
}

double PIBT::get_future_penalty(const int agent_i, const int j, Vertex* u_i, Vertex* u_j, const Config& Q_from, const Config& Q_to)
{
  // Scatter
  Vertex *j_scatter = nullptr;
  if (scatter != nullptr) {
    auto itr_s = scatter->scatter_data[j].find(Q_from[j]->id);
    if (itr_s != scatter->scatter_data[j].end()) {
      j_scatter = itr_s->second;
    }
  }

  int K = 0; // num of neighbors

  // wait is possible if agent j is not pushed by agent i
  if (u_i != u_j) C_next[j][K++] = u_j;

  for (auto u_j_maybe : Q_from[j]->neighbor) {
    ///
    /// filtering invalid moves
    /// 
    // 1. Vertex collision with i: j cannot move to the cell i is claiming
    if (u_j_maybe == u_i) continue; 
    // 2. True Swap Conflict: only applies if they are exchanging places
    if (u_i == u_j && u_j_maybe == Q_from[agent_i]) continue; 
    // vertex collision
    if (occupied_next[u_j_maybe->id] != NO_AGENT) continue; 
    // swap conflict with agent k
    int agent_k = occupied_now[u_j_maybe->id];
    if (agent_k != NO_AGENT && Q_to[agent_k] == u_j) continue; 
    ///
    /// the move is valid
    ///
    C_next[j][K++] = u_j_maybe;
  }

  // j is forced to wait
  if (K == 0) {
    return TRAPPED_PENALTY;
  }

  if (K == 1) {
    return pair_db->get(agent_i, j, u_i, C_next[j][0]);    
  }

  std::sort(C_next[j].begin(), C_next[j].begin() + K,
            [&](Vertex *const v, Vertex *const u) {
              if (v == j_scatter) return true;
              if (u == j_scatter) return false;
              return D->get(j, v) + oracle_tie_breakers[j][v->id] <
                     D->get(j, u) + oracle_tie_breakers[j][u->id];
            });

  double expected_penalty = 0.0;
  double remaining_prob = 1.0;  

  for (int v_idx = 0; v_idx < K; v_idx++) {
    Vertex* v = C_next[j][v_idx];
    // Calculate risk
    int risk = get_move_risk(j, v, Q_to, agent_i); 
    double success_rate = RISK_NUM_TO_PROB[std::min(risk, 5)];
    
    // Actual probability of ending up here
    double p_v = remaining_prob * success_rate;
    
    // Add to expected penalty
    if (p_v > 0) {
        expected_penalty += p_v * pair_db->get(agent_i, j, u_i, v);
        remaining_prob -= p_v;
    }
    
    // Early exit if we've exhausted all probability
    if (remaining_prob <= 0.01) break; 
  }

  if (remaining_prob > 0.01) {
      if (u_i == u_j) {
          expected_penalty += BASE_DISCOUNT * remaining_prob * pair_db->get(agent_i, j, Q_from[agent_i], u_j);
      } else {
          expected_penalty += BASE_DISCOUNT * remaining_prob * pair_db->get(agent_i, j, u_i, u_j);
      }
  }  
  return expected_penalty;           
}

void PIBT::fill_dh_values(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start)
{
  for (int k = start; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    double max_penalty = 0;
    double total_penalty = 0;

    for (Vertex* u_j : radial_neighbors[u_i->id]) {
      int r = std::max(1, pair_db->D->get(u_i->id, u_j->id));
      double r_scale = RADIUS_DECAY[r];

      // Check for agents moving TO this cell (PART 1)
      int j = occupied_next[u_j->id];
      if (j != NO_AGENT && j != i) {
        double current_penalty = r_scale * pair_db->get(i, j, u_i, u_j);
        if (current_penalty > max_penalty) max_penalty = current_penalty;
        total_penalty += current_penalty;
      }

      // Check for agents CURRENTLY AT this cell (PART 2)
      j = occupied_now[u_j->id];
      if (j != NO_AGENT && j != i && Q_to[j] == nullptr) {
        double expected_penalty;
        auto& rec = dh_prediction_cache[dh_cache_key(i, j)][k];
        rec.u_i = u_i;
        if (r == 1) {
          expected_penalty = get_future_penalty(i, j, u_i, u_j, Q_from, Q_to);
          int wait_penalty = (u_i == u_j)
                  ? pair_db->get(i, j, Q_from[i], u_j)
                  : pair_db->get(i, j, u_i      , u_j);
          rec.has_future = true;
          rec.future_dh = expected_penalty;
          rec.has_wait = true;
          rec.wait_dh = wait_penalty;
        } else {
          int wait_penalty = (u_i == u_j)
                  ? pair_db->get(i, j, Q_from[i], u_j)
                  : pair_db->get(i, j, u_i      , u_j);
          expected_penalty = r_scale * wait_penalty * BASE_DISCOUNT;
        }
        if (expected_penalty > max_penalty) max_penalty = expected_penalty;
        total_penalty += expected_penalty;
      }
    }
    double final_penalty = max_penalty + (total_penalty - max_penalty) * 0.25;
    // Write the final maximum penalty to the pre-allocated array
    dh_values[u_i->id] = DH_WEIGHT * final_penalty;
  }
}

bool PIBT::funcPIBT(const int i, const Config &Q_from, Config &Q_to)
{
  const auto K = Q_from[i]->neighbor.size();

  // exploit scatter data
  Vertex *prioritized_vertex = nullptr;
  if (scatter != nullptr) {
    auto itr_s = scatter->scatter_data[i].find(Q_from[i]->id);
    if (itr_s != scatter->scatter_data[i].end()) {
      prioritized_vertex = itr_s->second;
    }
  }

  // set C_next
  auto get_tb = [&](Vertex* v) -> double {
    if (FIXED_TIE) 
      return (v == Q_from[i]) ? 0 : oracle_tie_breakers[i][v->id];
    double tb = RANDOM_TIE_WEIGHT * get_random_float(MT);
    tie_breakers[v->id] = tb;
    return tb;
  };
  for (size_t k = 0; k < K; ++k) {
    auto u = Q_from[i]->neighbor[k];
    C_next[i][k] = u;
    tie_breakers[u->id] = get_tb(u);
    dh_values[u->id] = 0;
  }
  C_next[i][K] = Q_from[i];
  tie_breakers[Q_from[i]->id] = 0; // get_tb(Q_from[i]);
  dh_values[Q_from[i]->id] = 0;


  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex *const v, Vertex *const u) {
              if (v == prioritized_vertex) return true;
              if (u == prioritized_vertex) return false;
              return D->get(i, v) + tie_breakers[v->id] <
                     D->get(i, u) + tie_breakers[u->id];
            });

  auto swap_agent = NO_AGENT;
  if (flg_swap) {
    swap_agent = is_swap_required_and_possible(i, Q_from, Q_to);
  }

  // emulate swap
  if (swap_agent != NO_AGENT) {
    // reverse vertex scoring
    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
  }

  auto swap_operation = [&]() {
    if (swap_agent != NO_AGENT &&                 // swap_agent exists
        Q_to[swap_agent] == nullptr &&            // not decided
        occupied_next[Q_from[i]->id] == NO_AGENT  // free
    ) {
      // pull swap_agent
      occupied_next[Q_from[i]->id] = swap_agent;
      Q_to[swap_agent] = Q_from[i];
    }
  };

  // dh_values applied lazily on first detected tie in the main loop
  bool dh_filled = (pair_db == nullptr || swap_agent != NO_AGENT);

  // main loop
  for (size_t k = 0; k < K + 1; ++k) {
    // 1. Evaluate ties and sort FIRST
    if (!dh_filled && k < K && C_next[i][k] != prioritized_vertex &&
        D->get(i, C_next[i][k]->id) == D->get(i, C_next[i][k + 1]->id)) {
      
      fill_dh_values(i, C_next[i], K + 1, Q_from, Q_to, k);
      
      std::sort(C_next[i].begin() + k, C_next[i].begin() + K + 1,
              [&](Vertex *const v, Vertex *const u) {
                if (v == prioritized_vertex) return true;
                if (u == prioritized_vertex) return false;
                double hv = D->get(i, v) + dh_values[v->id];
                double hu = D->get(i, u) + dh_values[u->id];
                if (hv != hu) return hv < hu;
                return tie_breakers[v->id] < tie_breakers[u->id];
              });
      
      dh_filled = true;
    }

    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != NO_AGENT) continue;

    auto j = occupied_now[u->id];
    // avoid swap conflicts with constraints
    if (j != NO_AGENT && Q_to[j] == Q_from[i]) continue;

    // reserve next location
    occupied_next[u->id] = i;
    Q_to[i] = u;

    // priority inheritance
    if (j != NO_AGENT && u != Q_from[i] && Q_to[j] == nullptr &&
        !funcPIBT(j, Q_from, Q_to))
      continue;

    // success to plan next one step
    if (flg_swap && k == 0) swap_operation();
    return true;
  }

  // failed to secure node
  occupied_next[Q_from[i]->id] = i;
  Q_to[i] = Q_from[i];
  return false;
}

void PIBT::evaluate_dh_predictions(const Config &Q_to)
{
  static const double FUTURE_BUCKET_EDGES[] = {0.25, 0.5, 1.0, 2.0, 3.0, 4.0,
                                                5.0,  6.0, 7.0, 8.0, 9.0, 10.0};
  constexpr int NUM_EDGES = sizeof(FUTURE_BUCKET_EDGES) / sizeof(FUTURE_BUCKET_EDGES[0]);

  for (auto &kv : dh_prediction_cache) {
    const int i = (int)(kv.first / N);
    const int j = (int)(kv.first % N);
    for (auto &rec : kv.second) {
      if (rec.u_i == nullptr || rec.u_i != Q_to[i]) continue;
      const double actual = pair_db->get(i, j, Q_to[i], Q_to[j]);

      if (rec.has_future) {
        const double err = std::abs(actual - rec.future_dh);
        int b = NUM_EDGES;  // overflow bucket (>= 10)
        for (int e = 0; e < NUM_EDGES; ++e) {
          if (err < FUTURE_BUCKET_EDGES[e]) { b = e; break; }
        }
        future_error_buckets[b]++;
      }

      if (rec.has_wait) {
        const int err = std::abs((int)actual - rec.wait_dh);
        const int b = (err <= 10) ? err : NUM_WAIT_ERROR_BUCKETS - 1;
        wait_error_buckets[b]++;
      }
    }
  }
}

void PIBT::print_dh_error_buckets()
{
  static const double FUTURE_BUCKET_EDGES[] = {0.0,  0.25, 0.5, 1.0, 2.0, 3.0, 4.0,
                                                5.0,  6.0,  7.0, 8.0, 9.0, 10.0};
  constexpr int NUM_EDGES = sizeof(FUTURE_BUCKET_EDGES) / sizeof(FUTURE_BUCKET_EDGES[0]);

  std::cout << "=== future_dh prediction |error| buckets ===" << std::endl;
  for (int b = 0; b < NUM_EDGES - 1; ++b) {
    std::cout << "  [" << FUTURE_BUCKET_EDGES[b] << ", " << FUTURE_BUCKET_EDGES[b + 1]
               << "): " << future_error_buckets[b] << std::endl;
  }
  std::cout << "  [" << FUTURE_BUCKET_EDGES[NUM_EDGES - 1] << ", inf): "
             << future_error_buckets[NUM_FUTURE_ERROR_BUCKETS - 1] << std::endl;

  std::cout << "=== wait_dh prediction |error| buckets ===" << std::endl;
  for (int b = 0; b <= 10; ++b) {
    std::cout << "  " << b << ": " << wait_error_buckets[b] << std::endl;
  }
  std::cout << "  >10: " << wait_error_buckets[NUM_WAIT_ERROR_BUCKETS - 1] << std::endl;
}

int PIBT::is_swap_required_and_possible(const int i, const Config &Q_from,
                                        Config &Q_to)
{
  // agent-j occupying the desired vertex for agent-i
  const auto j = occupied_now[C_next[i][0]->id];
  if (j != NO_AGENT && j != i &&  // j exists
      Q_to[j] == nullptr &&       // j does not decide next location
      is_swap_required(i, j, Q_from[i], Q_from[j]) &&  // swap required
      is_swap_possible(Q_from[j], Q_from[i])           // swap possible
  ) {
    return j;
  }

  // for clear operation, c.f., push & swap
  if (C_next[i][0] != Q_from[i]) {
    for (auto u : Q_from[i]->neighbor) {
      const auto k = occupied_now[u->id];
      if (k != NO_AGENT &&              // k exists
          C_next[i][0] != Q_from[k] &&  // this is for clear operation
          is_swap_required(k, i, Q_from[i],
                           C_next[i][0]) &&  // emulating from one step ahead
          is_swap_possible(C_next[i][0], Q_from[i])) {
        return k;
      }
    }
  }
  return NO_AGENT;
}

bool PIBT::is_swap_required(const int pusher, const int puller,
                            Vertex *v_pusher_origin, Vertex *v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex *tmp = nullptr;
  while (D->get(pusher, v_puller) < D->get(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      const auto i = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && i != NO_AGENT && ins->goals[i] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap at v_l
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (D->get(puller, v_pusher) < D->get(puller, v_puller)) &&
         (D->get(pusher, v_pusher) == 0 ||
          D->get(pusher, v_puller) < D->get(pusher, v_pusher));
}

bool PIBT::is_swap_possible(Vertex *v_pusher_origin, Vertex *v_puller_origin)
{
  // simulate pull
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex *tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      const auto i = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && i != NO_AGENT && ins->goals[i] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;  // able to swap at v_next
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}
