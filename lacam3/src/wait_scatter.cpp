#include "../include/wait_scatter.hpp"
#include "../include/metrics.hpp"

// HYPER PARAMETERS
const int MAX_WAIT = 2;

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

  auto CLOSED_cost = std::vector<VisitState>(V_size, VisitState::UNVISITED);
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
                        std::vector<VisitState>& CLOSED_cost, std::vector<int>& CLOSED_gen,
                        int& current_gen, uint32_t fast_seed,
                        int override_time, Vertex* override_start)
{
  const auto cost_ub = override_start == nullptr
      ? D->get(i, ins->starts[i]) + cost_margin
      : std::max(D->get(i, ins->starts[i]) + cost_margin, 
                 override_time + D->get(i, override_start));

  const auto s_i = override_start == nullptr ? ins->starts[i] : override_start;

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
    false             // was waiting
  });

  arena.push_back(startNode);
  OPEN.push(&arena.back());

  while (!OPEN.empty() && !is_expired(deadline)) {
    auto node = OPEN.top();
    OPEN.pop();

    const auto v = node->v;

    if (CLOSED_gen[v->id] != current_gen) {
      CLOSED_cost[v->id] = UNVISITED;
      CLOSED_gen[v->id] = current_gen;
    }

    // The Two-Lane Pareto Filter
    if (node->was_waiting) {
      // A wait node can only enter if the vertex is completely untouched
      if (CLOSED_cost[v->id] != UNVISITED) continue;
      CLOSED_cost[v->id] = VISITED_BY_WAIT;
    } else {
      // A move node can overwrite a wait node, but not another move node
      if (CLOSED_cost[v->id] == VISITED_BY_MOVE) continue;
      CLOSED_cost[v->id] = VISITED_BY_MOVE;
    }

    if (v == ins->goals[i] ) {
      Path result;
      auto cur = node;
      while (cur != nullptr) {
        result.push_back(cur->v);

        // Fill in the time gaps created by the wait macro-actions
        if (cur->parent != nullptr) {
          int time_gap = cur->g - cur->parent->g;
          for (int w = 1; w < time_gap; ++w) {
            result.push_back(cur->parent->v);
          }
        }

        cur = cur->parent;
      }
      std::reverse(result.begin(), result.end());
      return result;
    }

    expand(node, i, s_i, cost_ub, arena, OPEN);
  }
  std::cout << "failed to find path" << std::endl;
  return {};
}

template<typename OpenQueue>
void WaitScatter::expand(Node* node, int i, Vertex* s_i, int cost_ub,
                         std::deque<Node>& arena, OpenQueue& OPEN)
{
  const auto v = node->v;
  const auto time = node->g;
  const auto c_v = node->collisions;

  for (auto u : v->neighbor) {
    auto d_u = D->get(i, u);
    if (u != s_i && d_u + time + 1 <= cost_ub) {
      int step_collisions = CT.getCollisionCost(v, u, time);
      if (step_collisions > 0) {
        // std::cout << "considering wait for vertex " << v->id << " at time " << time << std::endl;
        // look-ahead: find earliest safe future time to move to u
        for (int wait_steps = 1; wait_steps <= MAX_WAIT; ++wait_steps) {
          int t_move = time + wait_steps;
          if (CT.getCollisionCost(v, v, t_move - 1) > 0) break; // hit while idling
          if (CT.getCollisionCost(v, u, t_move) == 0) {
            if (d_u + t_move + 1 <= cost_ub) {
              // std::cout << "Added wait for vertex " << v->id << " at time " << time << std::endl;
              arena.push_back({u, t_move + 1, d_u,
                               c_v,
                               node, true});
              arena.back().was_waiting = true;
              OPEN.push(&arena.back());
            }
            break;
          }
        }
      }
      arena.push_back({u, time + 1, d_u,
                        c_v + step_collisions,
                        node, false});
      OPEN.push(&arena.back());
    }
  }
}


