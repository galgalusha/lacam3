#pragma once

#include "scatter.hpp"
#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "utils.hpp"

struct WaitScatter : IScatter {

  enum VisitState {
    UNVISITED = 0,
    VISITED_BY_WAIT = 1,
    VISITED_BY_MOVE = 2
  };

  struct Node {
    Vertex *v;
    int g;           // cost-to-come
    int d;           // cost-to-go
    int collisions;
    Node *parent;
    uint32_t tie_breaker;
    bool was_waiting;
  };

  const Instance *ins;
  Deadline *deadline;
  std::mt19937 MT;
  const int verbose;
  const int N;
  const int V_size;
  const int T;  // makespan lower bound
  DistTable *D;
  std::deque<Node> arena = std::deque<Node>();

  const int cost_margin;
  int sum_of_path_length;

  // outcome
  std::vector<Path> paths;
  // agent, (vertex-id, time) -> next vertex
  struct PairHash {
    std::size_t operator()(const std::pair<int,int>& p) const {
      return std::hash<long long>()(((long long)p.first << 32) | (unsigned)p.second);
    }
  };
  std::vector<std::unordered_map<std::pair<int,int>, Vertex *, PairHash>> scatter_data;

  Vertex* get_neighbor(int time, int agent, int vertex_id) const override
  {
    const auto& map = scatter_data[agent];
    auto itr = map.find({vertex_id, time});
    if (itr != map.end()) return itr->second;
    // fall back to the entry with the same vertex_id and closest time
    Vertex* best = nullptr;
    int best_dt = std::numeric_limits<int>::max();
    // int max_time = std::numeric_limits<int>::min();
    for (const auto& [key, v] : map) {
      if (key.first != vertex_id) continue;
      // if (key.second > max_time) max_time = key.second;
      int dt = std::abs(key.second - time);
      bool is_wait = v->id == vertex_id;
      if (dt < best_dt && !is_wait) { best_dt = dt; best = v; }
    }
    // if (max_time != std::numeric_limits<int>::min() && time > max_time) return nullptr;
    return best;
  }

  const std::vector<Path>& get_paths() const override { return paths; }

  // collision data
  CollisionTable CT;

  void construct(int iterations) override;

  template<typename OpenQueue>
  void expand(Node* node, int i, Vertex* s_i, int cost_ub,
              std::deque<Node>& arena, OpenQueue& OPEN);

  Path astar(int i, std::vector<VisitState>& CLOSED_cost, std::vector<int>& CLOSED_gen,
             int& current_gen, uint32_t fast_seed,
             int override_time = 0, Vertex* override_start = nullptr);

  WaitScatter(const Instance *_ins, DistTable *_D, Deadline *_deadline,
              const int seed = 0, int _verbose = 0, int _cost_margin = 2);

};
