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
#include "restarter.hpp"

struct ASHA_Planner {
  struct LaCAM_Res {
    HNode* H;
    int iterations;
    bool is_goal;
    bool is_success;
  };

  const Instance *ins;
  const Deadline *deadline;
  const int seed;
  std::mt19937 MT;
  const int verbose;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTable *D;
  PIBT* pibt;
  bool delete_dist_table_after_used;

  // heuristic
  Heuristic *heuristic;

  // scatter (SUO)
  Scatter *scatter;

  // for refiner
  int seed_refiner;

  // for search utils
  std::unordered_map<Config, HNode *, ConfigHasher> EXPLORED;
  HNode *H_init;  // start node
  HNode *H_goal;  // goal node

  ASHA_Planner(const Instance *_ins, int _verbose = 0,
          const Deadline *_deadline = nullptr, int _seed = 0,
          DistTable *_D = nullptr  // used in recursive LaCAM
  );
  ~ASHA_Planner();
  Solution solve();
  LaCAM_Res run_lacam(HNode* H_from, int max_iterations=INT_MAX, int upper_bound=INT_MAX, int max_depth=INT_MAX);
  HNode *create_highlevel_node(const Config &Q, HNode *parent);
  HNode* rewrite(HNode *H_from, HNode *H_to);
  int get_edge_cost(const Config &C1, const Config &C2);
  Solution backtrack(HNode *H);
  void set_scatter();
  void set_pibt();
  void logging();
};
