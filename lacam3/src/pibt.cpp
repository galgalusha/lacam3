#include "../include/pibt.hpp"

PairWiseDB* PIBT::pair_db = nullptr;
float PIBT::GAMMA = 0.5;

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
      dh_values(V_size, 0),
      flg_swap(_flg_swap),
      scatter(_scatter),
      pair_distances(N*N)
{
}

PIBT::~PIBT() {}

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

void PIBT::fill_dh_values(const int i, const std::array<Vertex*, 5>& neighbors, const int num_neighbors, const Config& Q_from, const Config& Q_to)
{
  for (int k = 0; k < num_neighbors; ++k) {
    Vertex* u_i = neighbors[k];
    int max_penalty = 0;

    int r = PairWiseDB::RADIUS + 0;

    // Spatial scan within RADIUS=3 Manhattan distance around u_i
    const int cx = u_i->x;
    const int cy = u_i->y;
    const int width = ins->G->width;
    const int height = ins->G->height;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        // if (std::abs(dx) + std::abs(dy) > PairWiseDB::RADIUS) continue;
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        Vertex* u_j = ins->G->U[ny * width + nx];
        if (u_j == nullptr) continue;
        if (pair_db->D->get(u_i->id, u_j->id) > r) continue;

        // Check for agents moving TO this cell (PART 1)
        int j_next = occupied_next[u_j->id];
        if (j_next != NO_AGENT && j_next != i) {
          int current_penalty = pair_db->get(i, j_next, u_i, u_j);
          // int current_penalty = pair_db->get_from_map(ins->goals[i]->id, ins->goals[j_next]->id, u_i->id, u_j->id);
          if (current_penalty > max_penalty) max_penalty = current_penalty;
        }

        // Check for agents CURRENTLY AT this cell (PART 2)
        int j_now = occupied_now[u_j->id];
        if (j_now != NO_AGENT && j_now != i) {
          // Only apply PART 2 if the agent hasn't planned a move yet.
          // If Q_to is not null, they are moving somewhere else and will be evaluated there.
          if (Q_to[j_now] == nullptr) {
            int wait_penalty = pair_db->get(i, j_now, u_i, u_j);
            // int wait_penalty = pair_db->get_from_map(ins->goals[i]->id, ins->goals[j_now]->id, u_i->id, u_j->id);
            int current_penalty = wait_penalty * GAMMA;
            if (current_penalty > max_penalty) max_penalty = current_penalty;
          }
        }
      }
    }

    // Write the final maximum penalty to the pre-allocated array
    dh_values[u_i->id] = max_penalty;
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
  for (size_t k = 0; k < K; ++k) {
    auto u = Q_from[i]->neighbor[k];
    C_next[i][k] = u;
    tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
    dh_values[u->id] = 0;
  }
  C_next[i][K] = Q_from[i];
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
  } else if (pair_db != nullptr) {
    fill_dh_values(i, C_next[i], K + 1, Q_from, Q_to);
    std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex *const v, Vertex *const u) {
              if (v == prioritized_vertex) return true;
              if (u == prioritized_vertex) return false;
              return D->get(i, v) + 0.001 * tie_breakers[v->id] + 0.01 * (float)dh_values[v->id] <
                     D->get(i, u) + 0.001 * tie_breakers[u->id] + 0.01 * (float)dh_values[u->id];
            });    
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

  // main loop
  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != NO_AGENT) continue;

    const auto j = occupied_now[u->id];

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
