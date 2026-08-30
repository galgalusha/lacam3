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

  // prefix refinement
  bool FLG_PREFIX_REFINEMENT;
  Config prefix_goals;       // intermediate goals (phase 1 targets)
  DoubleModeDistTable *dmt;  // non-null when FLG_PREFIX_REFINEMENT is true

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
          DistTable *_D = nullptr,
          const Config *_prefix_goals = nullptr  // non-null enables prefix refinement mode
  );
  ~ASHA_Planner();
  Solution solve();
  LaCAM_Res run_lacam(HNode* H_from, int max_iterations=INT_MAX, int upper_bound=INT_MAX, int max_depth=INT_MAX);
  HNode *create_highlevel_node(const Config &Q, HNode *parent);
  HNode* rewrite(HNode *H_from, HNode *H_to);
  int get_edge_cost(const Config &C1, const Config &C2, const std::vector<bool> *modes = nullptr);
  Solution backtrack(HNode *H);
  void set_scatter();
  void set_pibt();
  void logging();
  HNode* refine_prefix(LaCAM_Res& res_init);
};
