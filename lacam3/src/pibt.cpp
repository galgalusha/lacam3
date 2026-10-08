#include "../include/pibt.hpp"

#include <cmath>

HorizonPairDB* PIBT::pair_db = nullptr;

const double DH_WEIGHT         = 0.01;
const double RANDOM_TIE_WEIGHT = 0.1;
const double DISCOUNT_FACTOR   = 0.50;


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
      dh_values(V_size, 0),
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


void PIBT::fill_dh_values(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start)
{
  for (int k = start; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    if (u_i == Q_from[i]) continue;

    double max_penalty = 0;
    double total_penalty = 0;

    uint32_t mdd_u_i = get_mdd_id(u_i->id, goals[i]->id);

    //
    // Scenario 1: Agent j already moved  (Q_to[j] != nullptr)
    //
    for (const auto& conflict : pair_db->penalties[mdd_u_i]) {
      uint32_t mdd_j = conflict.mdd_id;
      double penalty = conflict.penalty;
      int j = mdd_registry.mdd_id_to_agent_next[mdd_j];
      if (j != mdd_registry.NO_AGENT && j != i && Q_to[j] != nullptr) {
        if (penalty > max_penalty) max_penalty = penalty;
        total_penalty += penalty;
      }
    }

    //
    // Scenario 2: Agent j did not yet move  (Q_to[j] == nullptr)
    //
    for (const auto& conflict : pair_db->time_shifted_penalties[mdd_u_i]) {
      uint32_t mdd_j = conflict.mdd_id;
      double penalty = conflict.penalty;
      int j = mdd_registry.mdd_id_to_agent_now[mdd_j];
      if (j != mdd_registry.NO_AGENT && j != i && Q_to[j] == nullptr && Q_from[j] != u_i) {
        double speculated_penalty = penalty * DISCOUNT_FACTOR;
        if (speculated_penalty > max_penalty) max_penalty = speculated_penalty;
        total_penalty += speculated_penalty;
      }
    }

    //
    // Scenario 3: Agent j is going to be pushed by agent i so that u_i==Q_from[j]
    //             therefore, mdd_j does not exist in conflicts[mdd_i]
    //
    int j = occupied_now[u_i->id];
    if (j != NO_AGENT && j != i && Q_to[j] == nullptr) {
      // uint32_t mdd_i = get_mdd_id(Q_from[i]->id, goals[i]->id);
      double penalty = get_mdd_panelaty_for_push(i, u_i, mdd_u_i, j, Q_from) * DISCOUNT_FACTOR;
      if (penalty > max_penalty) max_penalty = penalty;
      total_penalty += penalty;
    } 

    double final_penalty = max_penalty + (total_penalty - max_penalty) * 0.25;
    // Write the final maximum penalty to the pre-allocated array
    dh_values[u_i->id] = DH_WEIGHT * final_penalty;
  }
}


uint8_t PIBT::get_mdd_panelaty_for_push(int i, Vertex* u_i, uint32_t mdd_i_id, int j, const Config& Q_from) {
  static const uint32_t MAX_PENALTY = 2;
  if (goals[j]->id == u_i->id) return MAX_PENALTY;
  uint8_t penalty = MAX_PENALTY;
  auto& j_frontiers = pair_db->mdd_by_id[get_mdd_id(u_i->id, goals[j]->id)].frontiers[1];
  for (auto u_j : j_frontiers) {
    if (u_j->id == Q_from[i]->id) continue; // swap conflict with i.
    if (occupied_next[u_j->id] != NO_AGENT) continue; // vertex conflict
    uint32_t mdd_u_j = get_mdd_id(u_j->id, goals[j]->id);
    uint8_t u_j_penalty = pair_db->get_penalty(mdd_i_id, mdd_u_j);
    if (u_j_penalty == 0) return 0;
    if (u_j_penalty < penalty) penalty = u_j_penalty;
  }
  return penalty;
}


bool PIBT::funcPIBT(const int i, const Config &Q_from, Config &Q_to)
{
  // std::cout << "[2] funcPIBT" << std::endl;

  const auto K = Q_from[i]->neighbor.size();

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
    dh_values[u->id] = 0;
  }
  C_next[i][K] = Q_from[i];
  tie_breakers[Q_from[i]->id] = 0;
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

      if (pair_db != nullptr) {
        mdd_registry.register_agent_next(swap_agent, get_mdd_id(Q_from[i]->id, goals[swap_agent]->id));
      }      
    }
  };

  // dh_values applied lazily on first detected tie in the main loop
  bool dh_filled = (pair_db == nullptr || swap_agent != NO_AGENT);

  // main loop
  for (size_t k = 0; k < K + 1; ++k) {
    // Lazy dh values evaluation (and re-sorting)
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
