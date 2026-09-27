#include "../include/pibt.hpp"

#include <cmath>

HorizonPairDB* PIBT::pair_db = nullptr;

const double DH_WEIGHT = 0.01;
const double RANDOM_TIE_WEIGHT = 0.1;
const double TRAPPED_PENALTY = INT_MAX;

static const double RADIUS_DECAY[] = {
    1.0,                // r = 0 (failsafe)
    1.0,                // r = 1
    0.75,               // r = 2
    0.5,                // r = 3
    0.25,               // r = 4
};

const double BASE_DISCOUNT      = 0.50;


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
      visited_token(N, 0),
      current_evaluation_token(0),
      agents_by_t_v((HorizonPairDB::HORIZON + 1) * _G->V.size(), std::vector<int>())   
{
  if (pair_db) setup_pair_db();
}

PIBT::~PIBT() {
}


void PIBT::setup_pair_db() {
  const int width = G->width;
  const int height = G->height;
}


bool PIBT::set_new_config(const Config &Q_from, Config &Q_to,
                          const std::vector<int> &order)
{
  bool success = true;

  // Clear previous agent t_v registrations
  for (size_t t_v : dirty_t_v_vectors) {
    agents_by_t_v[t_v].clear();
  }
  dirty_t_v_vectors.clear();  

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

  // Perform initial registration for all agents
  for (int j = 0; j < N; ++j) {
    Vertex* v = Q_to[j] != nullptr ? Q_to[j] : Q_from[j];
    uint32_t mdd_id = get_mdd_id(v->id, goals[j]->id);
    register_agent_mdd(j, mdd_id);
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


void PIBT::fill_dh_values(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to, const int start)
{
  for (int k = start; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    double max_penalty = 0;
    double total_penalty = 0;

    uint32_t mdd_i = get_mdd_id(u_i->id, goals[i]->id);
    current_evaluation_token++; // Unique ID for this specific (i, u_i) evaluation

    // this method supposed to invoke the lamda for each agent j that is registered in any (t,s) ot (t+1,s) in mdd_i
    //
    for_each_other_agent_overlapping_with_mdd(mdd_i, [&](int agent_j) {
      if (i == agent_j) return;
      if (visited_token[agent_j] == current_evaluation_token) return;
      visited_token[agent_j] = current_evaluation_token;      
      //
      // Setting up agent j
      //
      Vertex* u_j = Q_to[agent_j];
      bool agent_j_already_moved = true;
      if (u_j == nullptr) { 
        u_j = Q_from[agent_j]; 
        agent_j_already_moved = false; 
      }
      //
      // Scenario 1: Agent j moved
      //
      if (agent_j_already_moved) {
        double current_penalty = pair_db->get_penalty(u_i->id, goals[i]->id, u_j->id, goals[agent_j]->id);
        if (current_penalty > max_penalty) max_penalty = current_penalty;
        total_penalty += current_penalty;
      }
      //
      // Scenario 2: Agent j did not yet move
      //
      else {
        double expected_penalty = (u_i == u_j)
                ? pair_db->get_penalty(Q_from[i]->id, goals[i]->id, u_j->id, goals[agent_j]->id)
                : pair_db->get_penalty(u_i->id      , goals[i]->id, u_j->id, goals[agent_j]->id);
        expected_penalty *= BASE_DISCOUNT;
        if (expected_penalty > max_penalty) max_penalty = expected_penalty;
        total_penalty += expected_penalty;
      }      
    });

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
    register_agent_mdd(i, get_mdd_id(u->id, goals[i]->id));

    // priority inheritance
    if (j != NO_AGENT && u != Q_from[i] && Q_to[j] == nullptr && !funcPIBT(j, Q_from, Q_to)) {
      occupied_next[u->id] = NO_AGENT;
      Q_to[i] = nullptr;
      continue;
    }

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

void PIBT::register_agent_mdd(int agent_id, uint32_t mdd_id) {
  MDD& mdd = pair_db->mdd_by_id[mdd_id];
  for (int t = 0; t <= HorizonPairDB::HORIZON; t++) {
    for (Vertex* v : mdd.frontiers[t]) {
      size_t t_v = static_cast<size_t>(t) * V_size + v->id;
      auto& bucket = agents_by_t_v[t_v];
      // Prevent duplicate agent registrations in the same bucket
      if (std::find(bucket.begin(), bucket.end(), agent_id) != bucket.end()) {
        continue;
      }      
      // If this bucket is empty, mark it to be cleared later
      if (bucket.empty()) {
        dirty_t_v_vectors.push_back(t_v);
      }
      bucket.push_back(agent_id);
    }
  }
}


template <typename Func>
void PIBT::for_each_other_agent_overlapping_with_mdd(uint32_t mdd_id, Func callback) {
  MDD& mdd = pair_db->mdd_by_id[mdd_id];
  
  for (int t = 0; t <= HorizonPairDB::HORIZON; ++t) {
    for (Vertex* v : mdd.frontiers[t]) {
      
      // 1. Trigger on shared vertices at time t
      size_t t_v = static_cast<size_t>(t) * V_size + v->id;
      for (int agent_j : agents_by_t_v[t_v]) {
        callback(agent_j);
      }
      
      // 2. Trigger on time t+1 to catch swap conflicts and follows
      if (t + 1 <= HorizonPairDB::HORIZON) {
        size_t t_plus_1_v = static_cast<size_t>(t + 1) * V_size + v->id;
        for (int agent_j : agents_by_t_v[t_plus_1_v]) {
          callback(agent_j);
        }
      }
      
    }
  }
}
