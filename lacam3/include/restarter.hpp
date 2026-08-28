#pragma once

#include <vector>
#include "hnode.hpp"


class Restarter {
public:
  virtual ~Restarter() = default;
  virtual HNode* get_node_for_restart(HNode* H, HNode* H_goal) = 0;
  virtual bool should_restart_now(HNode* H) = 0;
  virtual bool should_prune(HNode* H, HNode* H_goal) {
    if (H_goal == nullptr) return false;
    return H->f >= H_goal->g;
  }
  virtual void on_cost_update(HNode* H_goal) = 0;
  virtual void on_found_goal(HNode* H_goal) = 0;
};


class SPD : public Restarter {
public:
  SPD() : restart_count(0) {}
  int restart_count;
  HNode* get_node_for_restart(HNode* H, HNode* H_goal);
  inline bool should_restart_now(HNode* H) { return false; }
  inline void on_cost_update(HNode* H_goal) {}
  inline void on_found_goal(HNode* H_goal) {}
};


class ASHA : public Restarter {
private:
    // Hyperparameters
    const int    NUM_CANDIDATES     = 4;
    const int    EVALUATION_COUNT   = 5;
    const double BASE_HORIZON       = 0.35;
    const double EVAL_HORIZON       = 0.70;
    const double PRUNING_RELAXATION = 1.5;

    enum Mode { INIT, GEN, EVAL, SPD_MODE };
    Mode mode = INIT;

    struct BaseCandidate {
        HNode* end_node;         // The restart anchor at depth 0.35 * makespan
        HNode* best_eval_node;   // The best node found at depth 0.70 * makespan
        
        int makespan;            // Frozen target depth (e.g., 100)
        double delta_error;      // Frozen calibration value
        bool has_delta;

        double total_costs;
        int eval_count;
        int success_count;
        int failure_count;
    };

    std::vector<BaseCandidate> pool;
    HNode* best_global_goal = nullptr;
    HNode* H_init = nullptr;
    
    int  active_idx = 0;
    int  makespan = 0;
    int  winner_idx = 0;
    bool expecting_success_restart = false;

    // SPD Mode variables
    const std::vector<double> spd_ratios = {0.4, 0.45, 0.55, 0.65, 0.7};
    int spd_restart_count = 0;

public:
  HNode* get_node_for_restart(HNode* H, HNode* H_goal);
  bool should_restart_now(HNode* H);
  void on_cost_update(HNode* H_goal);
  void on_found_goal(HNode* H_goal);
};

