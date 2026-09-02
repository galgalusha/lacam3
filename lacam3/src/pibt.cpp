#include "../include/pibt.hpp"

PairWiseDB* PIBT::pair_db = nullptr;
bool PIBT::FIXED_TIE = false;

const double DH_WEIGHT = 0.1;
const double RANDOM_TIE_WEIGHT = 0.001;
const double TRAPPED_PENALTY = INT_MAX;

const double WEIGHT_PRIORITIZED = 2.50;
const double WEIGHT_GOOD_MOVE   = 2.00;
const double WEIGHT_WAIT        = 1.50;
const double WEIGHT_BAD_MOVE    = 1.00;

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

PIBT::~PIBT() {}

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

  // cleanup
  for (auto i = 0; i < N; ++i) {
    occupied_now[Q_from[i]->id] = NO_AGENT;
    if (Q_to[i] != nullptr) occupied_next[Q_to[i]->id] = NO_AGENT;
  }

  return success;
}

std::pair<double, double> PIBT::get_future_penalty(const int i, const int j, Vertex* u_i, Vertex* u_j, const Config& Q_from, const Config& Q_to)
{
  double total_weight = 0;
  double expected_penalty_sum = 0;
  double expected_sq_penalty_sum = 0;
  int D_wait = D->get(j, u_j);

  // Scatter
  Vertex *j_prioritized = nullptr;
  if (scatter != nullptr) {
    auto itr_s = scatter->scatter_data[i].find(Q_from[i]->id);
    if (itr_s != scatter->scatter_data[i].end()) {
      j_prioritized = itr_s->second;
    }
  }

  for (auto u_j_maybe : Q_from[j]->neighbor) {
    //
    // filter conflicting maybes
    // 

    // 1. Vertex collision with i: j cannot move to the cell i is claiming
    if (u_j_maybe == u_i) continue; 
    // 2. True Swap Conflict: only applies if they are exchanging places
    if (u_i == u_j && u_j_maybe == Q_from[i]) continue; 
    // vertex collision
    if (occupied_next[u_j_maybe->id] != NO_AGENT) continue; 
    // swap conflict with agent k
    int agent_k = occupied_now[u_j_maybe->id];
    if (agent_k != NO_AGENT && Q_to[agent_k] == u_j) continue; 

    // --- CONSERVATIVE WEIGHT LOGIC ---
    double weight = WEIGHT_BAD_MOVE;
    
    double dh = pair_db->get(i, j, u_i, u_j_maybe);
    int D_maybe = D->get(j, u_j_maybe);

    if (u_j_maybe == j_prioritized) {
      weight = WEIGHT_PRIORITIZED;    
    } else if (D_maybe < D_wait) {
      weight = WEIGHT_GOOD_MOVE;      
    }
    
    total_weight += weight;
    expected_penalty_sum += (weight * dh);
    expected_sq_penalty_sum += (weight * dh * dh);
  }

  // j is forced to wait
  if (total_weight == 0) {
    if (u_i == u_j) return {TRAPPED_PENALTY, 1.0}; 
    return {pair_db->get(i, j, u_i, u_j), 1.0};    
  }

  // add the wait "move" if possible (if agent i is not pushing agent j)
  if (u_i != u_j && occupied_next[u_j->id] == NO_AGENT) {
    double dh = pair_db->get(i, j, u_i, u_j);
    double weight = WEIGHT_WAIT;
    total_weight += weight;
    expected_penalty_sum += (weight * dh);
    expected_sq_penalty_sum += (weight * dh * dh);
  }

  // Mean and Variance
  double future_dh = expected_penalty_sum / total_weight;
  double mean_sq = expected_sq_penalty_sum / total_weight;
  double variance = mean_sq - (future_dh * future_dh);
  if (variance < 0) variance = 0;  
 
  // Dynamic Gamma
  double gamma = 1.0 / (1.0 + variance);
  return {future_dh, gamma};
}

void PIBT::fill_dh_values(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start)
{
  for (int k = start; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    double max_penalty = 0;

    for (Vertex* u_j : radial_neighbors[u_i->id]) {
      // Check for agents moving TO this cell (PART 1)
      int j = occupied_next[u_j->id];
      if (j != NO_AGENT && j != i) {
        double current_penalty = pair_db->get(i, j, u_i, u_j);
        if (current_penalty > max_penalty) max_penalty = current_penalty;
      }

      // Check for agents CURRENTLY AT this cell (PART 2)
      j = occupied_now[u_j->id];
      if (j != NO_AGENT && j != i && Q_to[j] == nullptr) {
        auto [future_dh, gamma] = get_future_penalty(i, j, u_i, u_j, Q_from, Q_to);
        if (future_dh >= TRAPPED_PENALTY) { 
          max_penalty = future_dh; 
          continue; 
        }
        double base_penalty = (u_i == u_j)
                ? pair_db->get(i, j, Q_from[i], u_j)
                : pair_db->get(i, j, u_i      , u_j);  

        if (u_i != u_j) gamma *= gamma;

        double expected_penalty = (base_penalty * BASE_DISCOUNT * (1.0 - gamma)) + (future_dh * gamma);
        // double expected_penalty = gamma > threshold
        //        ? future_dh * gamma
        //        : base_penalty * BASE_DISCOUNT;

        if (expected_penalty > max_penalty) max_penalty = expected_penalty;
      }
    }

    // Write the final maximum penalty to the pre-allocated array
    dh_values[u_i->id] = DH_WEIGHT * max_penalty;
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
