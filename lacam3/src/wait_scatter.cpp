#include "../include/wait_scatter.hpp"
#include "../include/metrics.hpp"

// HYPER PARAMETERS
const int COLLISION_WEIGHT = 12;
const float ASTAR_WEIGHT = 1;
const int MAX_COLLISION_TIME = 50;
const int MAX_WAIT = 10;

inline int get_collision_factor(int time) {
  return 1;
  // int time_factor = 10;
  // if (time <= 5) {
  //   time_factor = 100;
  // } else if (time <= 15) {
  //   time_factor = 30;
  // } 
  // return time_factor;
}

WaitScatter::WaitScatter(const Instance *_ins, DistTable *_D,
                         Deadline *_deadline, const int seed,
                         int _verbose, int _cost_margin)
    : ins(_ins),
      deadline(_deadline),
      MT(std::mt19937(seed)),
      verbose(_verbose),
      N(ins->N),
      V_size(ins->G->size()),
      T(get_makespan_lower_bound(*ins, *_D) + _cost_margin),
      D(_D),
      cost_margin(_cost_margin),
      sum_of_path_length(0),
      paths(N),
      scatter_data(N),
      CT(ins)
{
}

void WaitScatter::construct(int iterations)
{
  info(1, verbose, deadline, "wait_scatter", "\tinvoked");

  // metrics
  auto collision_cnt_last = 0;
  auto paths_prev = std::vector<Path>();

  auto CLOSED_cost = std::vector<int>(V_size, 0);
  auto CLOSED_gen = std::vector<int>(V_size, 0);
  int current_gen = 0;

  // main loop
  auto loop = 0;
  while (loop < iterations) {
    ++loop;
    collision_cnt_last = CT.collision_cnt;

    // randomize planning order
    auto order_to_agent = std::vector<int>(N, 0);
    std::iota(order_to_agent.begin(), order_to_agent.end(), 0);
    std::shuffle(order_to_agent.begin(), order_to_agent.end(), MT);

    // single-agent path finding for agent-i
    for (int agent_order = 0; agent_order < N; ++agent_order) {
      current_gen++;

      if (is_expired(deadline)) break;

      const auto i = order_to_agent[agent_order];

      if (!paths[i].empty()) sum_of_path_length -= (paths[i].size() - 1);

      // clear cache
      CT.clearPath(i, paths[i]);

      // A*
      auto new_path = astar(i, CLOSED_cost, CLOSED_gen, current_gen, MT());
      if (!new_path.empty()) paths[i] = std::move(new_path);

      // register to CT & update collision count
      CT.enrollPath(i, paths[i]);
      sum_of_path_length += paths[i].size() - 1;

    } // agent loop

    paths_prev = paths;
    info(4, verbose, deadline, "wait_scatter", "\titer:", loop,
         "\tcollision_cnt:", CT.collision_cnt);

    // if (CT.collision_cnt == 0) break;
    if (is_expired(deadline)) break;

  } // epoch loop

  paths = paths_prev;

  // set scatter data (time-aware)
  for (auto i = 0; i < N; ++i) {
    if (paths[i].empty()) continue;
    for (auto t = 0; t < (int)paths[i].size() - 1; ++t) {
      scatter_data[i][{paths[i][t]->id, t}] = paths[i][t + 1];
    }
  }

  info(1, verbose, "wait_scatter", "\tcompleted");
}

Path WaitScatter::astar(int i,
                        std::vector<int>& CLOSED_cost, std::vector<int>& CLOSED_gen,
                        int& current_gen, uint32_t fast_seed,
                        int override_time, Vertex* override_start)
{
  const auto cost_ub = override_start == nullptr
      ? D->get(i, ins->starts[i]) + cost_margin
      : std::max(D->get(i, ins->starts[i]) + cost_margin, 
                 override_time + D->get(i, override_start));

  const auto s_i = override_start == nullptr ? ins->starts[i] : override_start;

  auto fast_rand = [&fast_seed]() {
    fast_seed ^= fast_seed << 13;
    fast_seed ^= fast_seed >> 17;
    fast_seed ^= fast_seed << 5;
    return fast_seed;
  };

  auto calc_cost = [](int c, int g, int d) {
    return c;
    // return c * COLLISION_WEIGHT + g + d * ASTAR_WEIGHT;
  };

  auto cmp = [&](const Node *a, const Node *b) {
    int cost_a = calc_cost(a->collisions, a->g, a->d);
    int cost_b = calc_cost(b->collisions, b->g, b->d);
    if (cost_a != cost_b) return cost_a > cost_b;
    if (a->collisions != b->collisions) return a->collisions > b->collisions;
    auto f_a = a->g + a->d;
    auto f_b = b->g + b->d;
    if (f_a != f_b) return f_a > f_b;
    return a->v->id < b->v->id;
  };

  arena.clear();
  auto OPEN = std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)>(cmp);


  Node startNode({
    s_i,              // start vertex
    override_time,    // time
    D->get(i, s_i),   // heuristic
    0,                // collisions
    nullptr,          // parent
    fast_rand()
  });

  arena.push_back(startNode);
  OPEN.push(&arena.back());
  const int NOT_VISITED = 0;
  const int VISITED = 1;

  while (!OPEN.empty() && !is_expired(deadline)) {
    auto node = OPEN.top();
    OPEN.pop();

    const auto v = node->v;
    int current_cost = calc_cost(node->collisions, node->g, node->d);
    bool is_wait = (node->waiting_for != nullptr);

    if (CLOSED_gen[v->id] != current_gen) {
      CLOSED_cost[v->id] = NOT_VISITED;
      CLOSED_gen[v->id] = current_gen;
    }

    if (CLOSED_cost[v->id] == VISITED && !is_wait) continue;
    CLOSED_cost[v->id] = VISITED;

    if (v == ins->goals[i] ) {
      Path result;
      auto cur = node;
      while (cur != nullptr) {
        result.push_back(cur->v);
        cur = cur->parent;
      }
      std::reverse(result.begin(), result.end());
      return result;
    }

    expand(node, i, s_i, cost_ub, arena, OPEN, fast_rand);
  }

  return {};
}

template<typename OpenQueue, typename RandFunc>
void WaitScatter::expand(Node* node, int i, Vertex* s_i, int cost_ub,
                         std::deque<Node>& arena, OpenQueue& OPEN, RandFunc& fast_rand)
{
  const auto v = node->v;
  const auto time = node->g;
  const auto c_v = node->collisions;
  const auto d_v = D->get(i, v);

  // --- Wait node: restricted expansion ---
  if (node->waiting_for != nullptr) {
    if (node->wait_depth > MAX_WAIT) return;
        
    Vertex* target = node->waiting_for;
    int new_wait_depth = node->wait_depth;

    auto d_target = D->get(i, target);
    if (d_target + time + 1 > cost_ub) { // margin exceeded
      return;
    }

    // target is free, move there
    if (CT.getCollisionCost(v, target, time) == 0) {
      arena.push_back({target, time + 1, d_target,
                       c_v,
                       node, fast_rand()});
      OPEN.push(&arena.back());
      return;
    }

    // target still occupied, continue waiting
    else { 
      arena.push_back({v, time + 1, d_v,
                        c_v + CT.getCollisionCost(v, v, time) * get_collision_factor(time),
                        node, fast_rand(),
                        target, new_wait_depth + 1});
      OPEN.push(&arena.back());
      return;
    }
  } // END of wait

  // --- Normal node: expand spatial neighbors + per-collision wait nodes ---
  for (auto u : v->neighbor) {
    auto d_u = D->get(i, u);
    if (u != s_i && d_u + time + 1 <= cost_ub) {
      int step_collisions = CT.getCollisionCost(v, u, time);

      if (step_collisions > 0) {
        // Create a wait node targeted at this colliding neighbor
        if (d_u + time + 1 <= cost_ub) {
          arena.push_back({v, time + 1, d_v,
                           c_v + CT.getCollisionCost(v, v, time) * get_collision_factor(time),
                           node, fast_rand(),
                           u, 1});
          OPEN.push(&arena.back());
        }
      }

      arena.push_back({u, time + 1, d_u,
                       c_v + step_collisions * get_collision_factor(time),
                       node, fast_rand()});

      OPEN.push(&arena.back());
    }
  }
}


