/*
 * Implementation of LaCAM*
 *
 * references:
 * LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding.
 * Keisuke Okumura.
 * Proc. AAAI Conf. on Artificial Intelligence (AAAI). 2023.
 *
 * Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding.
 * Keisuke Okumura.
 * Proc. Int. Joint Conf. on Artificial Intelligence (IJCAI). 2023.
 *
 * Engineering LaCAM*: Towards Real-Time, Large-Scale, and Near-Optimal
 * Multi-Agent Pathfinding. Keisuke Okumura. Proc. Int. Conf. on Autonomous
 * Agents and Multiagent Systems. 2024.
 */
#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "heuristic.hpp"
#include "hnode.hpp"
#include "instance.hpp"
#include "pibt.hpp"
#include "refiner.hpp"
#include "scatter.hpp"
#include "translator.hpp"
#include "utils.hpp"

struct RandomLNodeGen {
  std::mt19937* MT;
  DistTable* D;
  int t = 0;
  LNode empty;
  LNode* L = new LNode();

  RandomLNodeGen() : MT(nullptr), D(nullptr) {}
  RandomLNodeGen(std::mt19937* mt, DistTable* d) : MT(mt), D(d) {}

  LNode* generate(HNode* H) {
    int depth = 0;
    for (auto* node = H; node != nullptr; node = node->parent) depth++;
    if (depth > 1) return &empty;

    delete L;
    L = new LNode();

    int n = 0;
    while (n < 10 && get_random_float(MT) < 0.5) ++n;
    int pool_size = std::min((int)H->order.size(), get_random_int(MT, n, 50));
    auto pool = std::vector<int>(H->order.begin(), H->order.begin() + pool_size);
    std::shuffle(pool.begin(), pool.end(), *MT);
    n = std::min(n, pool_size);

    for (int p = 0; p < n; ++p) {
      int agent = pool[p];
      Vertex* v = H->C[agent];
      std::vector<Vertex*> neighbors;
      neighbors.reserve(v->neighbor.size() + 1);
      neighbors.push_back(v);
      for (auto u : v->neighbor) neighbors.push_back(u);
      int idx = get_random_int(MT, 0, static_cast<int>(neighbors.size()) - 1);
      L->who.push_back(agent);
      L->where.push_back(neighbors[idx]);
      L->depth++;
    }
    return L;
  }
};

struct RolloutPlanner {
  const Instance *ins;
  const Deadline *deadline;
  const int seed;
  std::mt19937 MT;
  const int verbose;
  const int depth;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTable *D;
  bool delete_dist_table_after_used;

  // heuristic
  Heuristic *heuristic;

  // scatter (SUO)
  Scatter *scatter;

  // configuration generator
  std::vector<PIBT *> pibts;

  // for refiner
  int seed_refiner;
  std::list<std::future<Solution>> refiner_pool;

  // for search utils
  HNode *H_init;  // start node
  HNode *H_goal;  // goal node

   // for logging
  static std::string MSG;

  int search_iter;
  int time_initial_solution;
  int cost_initial_solution;
  RandomLNodeGen lnode_gen;

  RolloutPlanner(const Instance *_ins, int _verbose = 0,
          const Deadline *_deadline = nullptr, int _seed = 0,
          int _depth = 0,          // used in recursive LaCAM
          DistTable *_D = nullptr  // used in recursive LaCAM
  );
  ~RolloutPlanner();
  Solution solve();
  bool set_new_config(HNode *S, LNode *M, Config &Q_to);
  HNode *create_highlevel_node(const Config &Q, HNode *parent);
  int get_edge_cost(const Config &C1, const Config &C2);
  Solution backtrack(HNode *H);
  void set_scatter();
  void set_pibt();
  void set_refiner(Solution& solution);
  Solution get_refined_plan(const Solution &plan_origin);
  void logging();
};
