#include "../include/restarter.hpp"

HNode* ASHA::get_node_for_restart(HNode* H, HNode* H_goal) {
  return nullptr;
}

bool ASHA::should_restart_now(HNode* H) {
  return false;
}

void ASHA::on_cost_update(HNode* H_goal) {}

void ASHA::on_found_goal(HNode* H_goal) {
  // Vault the best global solution to avoid amnesia
  if (best_global_goal == nullptr || H_goal->g < best_global_goal->g) {
      best_global_goal = H_goal;
  }

  if (mode == INIT) {
      H_init = H_goal;
      while (H_init->parent != nullptr) H_init = H_init->parent;
      makespan = H_goal->depth;

      // Extract Candidate 1
      int target_end = static_cast<int>(makespan * BASE_HORIZON);
      int target_eval = static_cast<int>(makespan * EVAL_HORIZON);
      
      HNode* H_eval = H_goal;
      for (int i = 0; i < makespan - target_eval && H_eval->parent; ++i) H_eval = H_eval->parent;
      
      HNode* H_end = H_eval;
      for (int i = 0; i < target_eval - target_end && H_end->parent; ++i) H_end = H_end->parent;

      BaseCandidate cand = {H_init, H_end, H_eval, 0.0, true, (double)H_goal->g, 1, 1, 0};
      cand.delta_error = H_goal->g - H_eval->f;
      pool.push_back(cand);

      std::cout << "[ASHA] INIT Mode: Found initial plan! Makespan: " << makespan << "\n";
      std::cout << "[ASHA] Transitioning to GEN mode. Blinding planner.\n";
      
      mode = GEN;
      expecting_success_restart = true; 
      H_goal = nullptr; // Blind the planner to find diverse full plans
  }
  else if (mode == EVAL) {
      BaseCandidate& cand = pool[active_idx];
      if (!cand.has_delta) {
          // First successful full run for this candidate -> Calculate Delta
          int target_eval = static_cast<int>(makespan * EVAL_HORIZON);
          HNode* H_eval = H_goal;
          
          if (H_goal->depth >= target_eval) {
              for (int i = 0; i < H_goal->depth - target_eval && H_eval->parent; ++i) H_eval = H_eval->parent;
              cand.delta_error = H_goal->g - H_eval->f;
              cand.best_eval_node = H_eval;
          } else {
              cand.delta_error = 0;
              cand.best_eval_node = H_goal;
          }
          
          cand.has_delta = true;
          cand.total_costs += H_goal->g;
          std::cout << "[ASHA] EVAL Mode: Cand " << active_idx + 1 << " full calibration success. Delta: " << cand.delta_error << "\n";
          expecting_success_restart = true;
      }
  }

}
