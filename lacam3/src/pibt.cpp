#include "../include/pibt.hpp"

#include <cmath>

HorizonPairDB* PIBT::pair_db = nullptr;

const double RANDOM_TIE_WEIGHT = 0.1;

const int MAX_TIME_DISCOUNTING = 7;

const double NO_TIME_DISCOUNTING[] = 
  {1, 1, 1, 1, 1, 1, 1, 1};

const double LATE_COMMITMENT_TIME_DISCOUNTING[] = 
  {1.00, 1.00, 1.00, 0.75, 0.25, 0, 0, 0, 0};

const double PENALTY_3_TIME_DISCOUNTING[] = 
  {1.00, 1.00, 0.85, 0.45, 0.20, 0.05, 0, 0, 0};

const double SIGMOID_TIME_DISCOUNTING[] = 
  {1.00, 1.00, 0.85, 0.45, 0.20, 0.05, 0, 0, 0};

const double GAL_TIME_DISCOUNTING[] = 
  { 1.0, 1.0, 1.0 , 0.9, 0.45, 0.2, 0.0, 0, 0};

const double* TIME_DISCOUNTING = GAL_TIME_DISCOUNTING;


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
      move_scores(V_size),
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


void PIBT::fill_move_scores(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start)
{
  for (int k = start; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    MoveScore& score = move_scores[u_i->id];
    score.reset();
    uint32_t mdd_u_i = get_mdd_id(u_i->id, goals[i]->id);

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
        score.max_penalty = std::max(score.max_penalty, penalty);
        score.sum_penalty += penalty;
      }
      //
      // Scenario 2: Agent j did not yet move
      //
      j = mdd_registry.mdd_id_to_agent_now[mdd_j];
      if (j != mdd_registry.NO_AGENT && j != i && Q_to[j] == nullptr) {
        score.discounted_max = std::max(score.discounted_max, penalty);
        score.discounted_sum += penalty;
      }
    }

    //
    // Scenario 3: Agent j is going to be pushed by agent i so that u_i==u_j
    //             therefore, mdd_j does not exist in conflicts[mdd_i]
    //
    int j = occupied_now[u_i->id];
    if (j != NO_AGENT && j != i && Q_to[j] == nullptr) {
      uint32_t mdd_i = get_mdd_id(Q_from[i]->id, goals[i]->id);
      uint32_t mdd_j = get_mdd_id(u_i->id, goals[j]->id);
      const auto &conflict = pair_db->get_penalty(mdd_u_i, mdd_j);
      double time_discount = conflict.time <= MAX_TIME_DISCOUNTING ? TIME_DISCOUNTING[conflict.time] : 0;
      double penalty = conflict.penalty * time_discount;
      if (penalty > 0) {
        score.discounted_max = std::max(score.discounted_max, penalty);
        score.discounted_sum += penalty;
      }
    }
  }
}


void PIBT::lexicographic_sort(int i, std::array<Vertex *, 5>& C_i_next, int K, Vertex* prioritized_vertex) {
  std::sort(C_i_next.begin(), C_i_next.begin() + K + 1,
      [&](Vertex *const v1, Vertex *const v2) {
        if (v1 == prioritized_vertex) return true;
        if (v2 == prioritized_vertex) return false;
        int D1 = D->get(i, v1);
        int D2 = D->get(i, v2);
        auto& score1 = move_scores[v1->id];
        auto& score2 = move_scores[v2->id];

        double D1_plus = (double)D1 + 0.7 * score1.max_penalty;
        double D2_plus = (double)D2 + 0.7 * score2.max_penalty;

        if (D1_plus != D2_plus) 
          return D1_plus < D2_plus;

        if (D1 != D2) 
          return D1 < D2;

        // max penalty
        if (score1.max_penalty != score2.max_penalty) 
          return score1.max_penalty < score2.max_penalty;
        // sum penalty
        if (score1.sum_penalty != score2.sum_penalty) 
          return score1.sum_penalty < score2.sum_penalty;
        // max discounted
        if (score1.discounted_max != score2.discounted_max) 
          return score1.discounted_max < score2.discounted_max;
        // sum discounted
        if (score1.discounted_sum != score2.discounted_sum) 
          return score1.discounted_sum < score2.discounted_sum;
        return tie_breakers[v1->id] < tie_breakers[v2->id];
      });
}


void PIBT::weighted_sort(int i, std::array<Vertex *, 5>& C_i_next, int K, Vertex* prioritized_vertex) {
 std::sort(C_i_next.begin(), C_i_next.begin() + K + 1,
      [&](Vertex *const v1, Vertex *const v2) {
        if (v1 == prioritized_vertex) return true;
        if (v2 == prioritized_vertex) return false;
        int D1 = D->get(i, v1);
        int D2 = D->get(i, v2);
        if (D1 != D2) return D1 < D2;

        auto& score1 = move_scores[v1->id];
        auto& score2 = move_scores[v2->id];
        double weight1 = 0.75 * (score1.max_penalty + 0.5 * score1.discounted_max) + 0.25 * (score1.sum_penalty + 0.5 * score1.discounted_sum);
        double weight2 = 0.75 * (score2.max_penalty + 0.5 * score2.discounted_max) + 0.25 * (score2.sum_penalty + 0.5 * score2.discounted_sum);
        if (weight1 != weight2) return weight1 < weight2;

        return tie_breakers[v1->id] < tie_breakers[v2->id];
      });
}


bool PIBT::funcPIBT(const int i, const Config &Q_from, Config &Q_to)
{
  // std::cout << "[2] funcPIBT" << std::endl;

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

  // // dh_values applied lazily on first detected tie in the main loop
  // bool dh_filled = (pair_db == nullptr || swap_agent != NO_AGENT);
  // int k_of_last_sorting = prioritized_vertex != nullptr ? 0 : -1;
  auto& C_i_next = C_next[i];

  if (pair_db != nullptr && swap_agent == NO_AGENT) {
    fill_move_scores(i, C_i_next, K + 1, Q_from, Q_to, 0);
    // lexicographic_sort(i, C_i_next, K, prioritized_vertex);
    weighted_sort(i, C_i_next, K, prioritized_vertex);
  }
  // main loop
  for (int k = 0; k < K + 1; ++k) {
    // // Lazy dh values evaluation (and re-sorting)
    // if (!dh_filled && k < K && k > k_of_last_sorting &&
    //   D->get(i, C_i_next[k]->id) == D->get(i, C_i_next[k + 1]->id)) {
      
    //   k_of_last_sorting = k + 1;
    //   while (k_of_last_sorting + 1 <= K && 
    //         D->get(i, C_i_next[k]->id) == D->get(i, C_i_next[k_of_last_sorting + 1]->id)) {
    //     k_of_last_sorting++;
    //   }
    //   fill_move_scores(i, C_i_next, k_of_last_sorting + 1, Q_from, Q_to, k);
      
    //   std::sort(C_i_next.begin() + k, C_i_next.begin() + k_of_last_sorting + 1,
    //           [&](Vertex *const v, Vertex *const u) {
    //             double hv = move_scores[v->id];
    //             double hu = move_scores[u->id];
    //             if (hv != hu) return hv < hu;
    //             return tie_breakers[v->id] < tie_breakers[u->id];
    //           });
    // }

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
