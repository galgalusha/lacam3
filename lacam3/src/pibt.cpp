#include "../include/pibt.hpp"

#include <cmath>

HorizonPairDB* PIBT::pair_db = nullptr;

const double RANDOM_TIE_WEIGHT = 0.1;
const double UNMOVED_DISCOUNTING = 0.5;
const double PELANTY_MAX_FACTOR = 0.75;
const double PENALTY_SUM_FACTOR = 1 - PELANTY_MAX_FACTOR;

const int MAX_TIME_DISCOUNTING = 7;
const double TIME_DISCOUNTING[] = 
  { 1.0, 1.0, 1.0 , 0.9, 0.45, 0.2, 0.0, 0, 0};
// const double TIME_DISCOUNTING[] = 
//   { 1.0, 1.0, 1.0 , 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };


PIBT::PIBT(const Graph *_G, Config _goals, DistTable *_D, int seed, bool _flg_swap,
           Scatter *_scatter)
    : G(_G),
      goals(_goals),
      MT(std::mt19937(seed)),
      N(goals.size()),
      V_size(G->size()),
      D(_D),
      NO_AGENT(N),
      occupied_now(V_size, NO_AGENT),
      occupied_next(V_size, NO_AGENT),
      C_next(N, std::array<Vertex *, 5>()),
      tie_breakers(V_size, 0),
      move_penalties(V_size),
      flg_swap(_flg_swap),
      scatter(_scatter),
      mdd_registry(N, pair_db != nullptr ? pair_db->mdd_count : 1)
{
}


PIBT::~PIBT() {
}


bool PIBT::set_new_config(const Config &Q_from, Config &Q_to,
                          const std::vector<int> &order)
{
  // std::cout << "[1] set_new_config" << std::endl;
  bool success = true;

  if (pair_db != nullptr) { 
    mdd_registry.clear();
  }

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


  if (success) {

    // Perform initial registration for all agents
    if (pair_db != nullptr) { 
      for (int j = 0; j < N; ++j) {
        mdd_registry.register_agent_now(j, get_mdd_id(Q_from[j]->id, goals[j]->id));
        Vertex* v = Q_to[j];
        if (v != nullptr) {
          mdd_registry.register_agent_next(j, get_mdd_id(v->id, goals[j]->id));
        }
      } 
    }

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


void PIBT::fill_move_penalties(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start)
{
  for (int k = start; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    uint32_t mdd_u_i = get_mdd_id(u_i->id, goals[i]->id);
    // uint32_t mdd_i = get_mdd_id(Q_from[i]->id, goals[i]->id);
    double penalty_sum = 0;
    double penalty_max = 0;

    for (const auto& conflict : pair_db->penalties[mdd_u_i]) {
      uint32_t mdd_j = conflict.mdd_id;
      int j;

      double time_discount = conflict.time <= MAX_TIME_DISCOUNTING ? TIME_DISCOUNTING[conflict.time] : 0;
      double penalty = conflict.penalty * time_discount;
      if (!(penalty > 0)) continue;

      //
      // Scenario 1: Agent j already moved
      //
      j = mdd_registry.mdd_id_to_agent_next[mdd_j];
      if (j != mdd_registry.NO_AGENT && j != i && Q_to[j] != nullptr) {
        penalty_max = std::max(penalty_max, penalty);
        penalty_sum += penalty;
      }
      //
      // Scenario 2: Agent j did not yet move
      //
      j = mdd_registry.mdd_id_to_agent_now[mdd_j];
      if (j != mdd_registry.NO_AGENT && j != i && Q_to[j] == nullptr) {
        double discounted_penalty = penalty * UNMOVED_DISCOUNTING;
        penalty_max = std::max(penalty_max, discounted_penalty);
        penalty_sum += discounted_penalty;
      }
    }

    //
    // Scenario 3: Agent j is going to be pushed by agent i so that u_i==u_j
    //             therefore, mdd_j does not exist in conflicts[mdd_i]
    //
    int j = occupied_now[u_i->id];
    if (j != NO_AGENT && j != i && Q_to[j] == nullptr) {
      uint32_t mdd_j = get_mdd_id(u_i->id, goals[j]->id);
      const auto &conflict = pair_db->get_penalty(mdd_u_i, mdd_j);
      double time_discount = conflict.time <= MAX_TIME_DISCOUNTING ? TIME_DISCOUNTING[conflict.time] : 0;
      double penalty = conflict.penalty * time_discount * UNMOVED_DISCOUNTING;
      if (penalty > 0) {
        penalty_max = std::max(penalty_max, penalty);
        penalty_sum += penalty;
      }
    }
    move_penalties[u_i->id] = PELANTY_MAX_FACTOR * (penalty_max) + PENALTY_SUM_FACTOR * (penalty_sum);
  }
}


bool PIBT::funcPIBT(const int i, const Config &Q_from, Config &Q_to)
{
  const int K = Q_from[i]->neighbor.size();

  // exploit scatter data
  Vertex *prioritized_vertex = nullptr;
  if (scatter != nullptr) {
    auto itr_s = scatter->scatter_data[i].find(Q_from[i]->id);
    if (itr_s != scatter->scatter_data[i].end()) {
      prioritized_vertex = itr_s->second;
    }
  }

  for (size_t k = 0; k < K; ++k) {
    auto u = Q_from[i]->neighbor[k];
    C_next[i][k] = u;
    tie_breakers[u->id] = RANDOM_TIE_WEIGHT * get_random_float(MT);
  }
  C_next[i][K] = Q_from[i];
  tie_breakers[Q_from[i]->id] = 0;


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

      if (pair_db != nullptr) {
        mdd_registry.register_agent_next(swap_agent, get_mdd_id(Q_from[i]->id, goals[swap_agent]->id));
      }      
    }
  };

  // dh_values applied lazily on first detected tie in the main loop
  bool pair_db_disabled = (pair_db == nullptr || swap_agent != NO_AGENT);
  int k_of_last_sorting = prioritized_vertex != nullptr ? 0 : -1;
  auto& C_i_next = C_next[i];

  // if (!pair_db_disabled) {
  //   fill_move_penalties(i, C_i_next, K + 1, Q_from, Q_to);
  //   std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
  //           [&](Vertex *const v, Vertex *const u) {
  //             if (v == prioritized_vertex) return true;
  //             if (u == prioritized_vertex) return false;
  //             int Dv = D->get(i, v);
  //             int Du = D->get(i, u);
  //             if (Dv != Du) return Dv < Du;
  //             double hv = move_penalties[v->id];
  //             double hu = move_penalties[u->id];
  //             if (hv != hu) return hv < hu;
  //             return tie_breakers[v->id] < tie_breakers[u->id];
  //           });    
  // }
  
  // main loop
  for (int k = 0; k < K + 1; ++k) {
    // Lazy dh values evaluation (and re-sorting)
    if (!pair_db_disabled && k < K && k > k_of_last_sorting &&
      D->get(i, C_i_next[k]->id) == D->get(i, C_i_next[k + 1]->id)) {
      
      k_of_last_sorting = k + 1;
      while (k_of_last_sorting + 1 <= K && 
            D->get(i, C_i_next[k]->id) == D->get(i, C_i_next[k_of_last_sorting + 1]->id)) {
        k_of_last_sorting++;
      }
      fill_move_penalties(i, C_i_next, k_of_last_sorting + 1, Q_from, Q_to, k);
      std::sort(C_i_next.begin() + k, C_i_next.begin() + k_of_last_sorting + 1,
              [&](Vertex *const v, Vertex *const u) {
                double hv = move_penalties[v->id];
                double hu = move_penalties[u->id];
                if (hv != hu) return hv < hu;
                return tie_breakers[v->id] < tie_breakers[u->id];
              });
    }

    auto u = C_i_next[k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != NO_AGENT) continue;

    auto j = occupied_now[u->id];
    // avoid swap conflicts with constraints
    if (j != NO_AGENT && Q_to[j] == Q_from[i]) continue;

    // reserve next location
    occupied_next[u->id] = i;
    Q_to[i] = u;

    if (pair_db != nullptr) mdd_registry.register_agent_next(i, get_mdd_id(u->id, goals[i]->id));

    // priority inheritance
    if (j != NO_AGENT && u != Q_from[i] && Q_to[j] == nullptr && !funcPIBT(j, Q_from, Q_to)) {
      continue;
    }

    // success to plan next one step
    if (flg_swap && k == 0) swap_operation();
    return true;
  }

  // failed to secure node
  occupied_next[Q_from[i]->id] = i;
  Q_to[i] = Q_from[i];
  if (pair_db != nullptr) {
    mdd_registry.register_agent_next(i, get_mdd_id(Q_from[i]->id, goals[i]->id));
  }  
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
          (u->neighbor.size() == 1 && i != NO_AGENT && goals[i] == u)) {
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
          (u->neighbor.size() == 1 && i != NO_AGENT && goals[i] == u)) {
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
