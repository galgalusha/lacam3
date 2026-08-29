#include "../include/asha_planner.hpp"
#include "../include/planner.hpp" // for the flags
#include <algorithm>
#include <iostream>


const int PIBT_DEADLOCK_ATTEMPTS = 500;
constexpr auto TIME_ZERO = std::chrono::seconds(0);
const std::vector<double> SPD_RATIOS = {0.0, 0.25, 0.35, 0.45, 0.55, 0.6};

const double BASE_HORIZON = 0.25;
const double EVAL_ERROR   = 0.03;
const int NUM_OF_CANDIDATES = 8;

ASHA_Planner::ASHA_Planner(const Instance *_ins, int _verbose, const Deadline *_deadline,
                 int _seed, DistTable *_D)
    : ins(_ins),
      deadline(_deadline),
      seed(_seed),
      MT(std::mt19937(seed)),
      verbose(_verbose),
      N(ins->N),
      V_size(ins->G->size()),
      D((_D == nullptr) ? new DistTable(ins) : _D),
      heuristic(new Heuristic(ins, D)),
      scatter(nullptr),
      seed_refiner(0),
      EXPLORED(),
      H_init(nullptr),
      H_goal(nullptr)
{
}

ASHA_Planner::~ASHA_Planner()
{
  if (heuristic != nullptr) delete heuristic;
  if (scatter != nullptr) delete scatter;
  delete pibt;
}

Solution ASHA_Planner::solve()
{
  // setup
  set_scatter();
  set_pibt();

  H_init = create_highlevel_node(ins->starts, nullptr);

  std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Finding initial solution from H_init...\n";
  auto res_init = run_lacam(H_init, INT_MAX, INT_MAX, INT_MAX);
  
  if (!res_init.is_goal) {
    std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Failed to find initial solution.\n";
    for (auto p : EXPLORED) delete p.second;
    return Solution();
  }

  int end_iters = res_init.iterations;
  int best_cost = H_goal->g;
  int end_depth = res_init.H->depth;

  HNode* best_H_candidate = nullptr;
  int best_candidate_score = INT_MAX;
  int best_candidate_iters = end_iters;

  // 4. Repeat for 4 candidates (Candidate 0 uses the initial path)
  for (int cand_idx = 0; cand_idx < NUM_OF_CANDIDATES; ++cand_idx) {
    std::cout << "\n[ASHA] t=" << elapsed_ms(deadline) << "ms === Generating Candidate " << cand_idx << " ===\n";
    
    HNode* H_end = nullptr;

    //
    // generation
    //    
    int cand_gen_iters = end_iters;
    if (cand_idx == 0) {
      H_end = H_goal;
    } else {
      // Reuse H_init for candidates 1, 2, and 3
      int iter_budget = end_iters * 3;
      int ub_budget = static_cast<int>(best_cost * 1.5);
      auto res = run_lacam(H_init, iter_budget, ub_budget, end_depth * 2);
      if (!res.is_goal || !res.is_success) {
        std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Candidate " << cand_idx << " generation failed. Skipping.\n";
        continue;
      } else {
        H_end = res.H;
        cand_gen_iters = res.iterations;
        std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Candidate " << cand_idx << " generated. Depth=" << H_end->depth << ", cost=" << H_end->g << "\n";
      }
    }

    //
    // evaluation
    //
    int makespan = H_end->depth;
    int base_horizon = static_cast<int>(makespan * BASE_HORIZON);
    
    HNode* H_candidate = H_end;
    while (H_candidate != nullptr && H_candidate->depth > base_horizon) {
      H_candidate = H_candidate->parent;
    }

    std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Cand " << cand_idx << " | Makespan: " << makespan 
              << " | H_mid depth: " << (H_candidate ? H_candidate->depth : -1) << "\n";

    if (H_candidate == nullptr) continue;

    int eval_iter_budget = end_iters; // Example budget
    int eval_ub = static_cast<int>(best_cost * 1.5);
    HNode *H_error_next = H_end, *H_error;
    while (((float)(H_end->g - H_error_next->f))/((float)H_end->g) < EVAL_ERROR) {
      H_error = H_error_next;
      H_error_next = H_error_next->parent;
    }
    int eval_depth = H_error->depth;
    int f_error = H_end->g - H_error->f;
    std::cout << "Eval depth: " << eval_depth << ", ratio: " << ((float)eval_depth)/((float)H_end->depth) << ", error: " << f_error << std::endl;

    // 3. run_lacam 4 more times from H_mid with budgets
    int eval_f_error_sum = 0;
    int eval_successes = 0;
    for (int eval_idx = 0; eval_idx < 4; ++eval_idx) {
      auto eval_res = run_lacam(H_candidate, eval_iter_budget, eval_ub, eval_depth);

      std::cout << "  -> [EVAL " << eval_idx << "] t=" << elapsed_ms(deadline) << "ms Cand " << cand_idx << " (Run from H_mid) "
                << "| Success: " << eval_res.is_success
                << "| Iters: " << eval_res.iterations
                << "| Final Depth: " << (eval_res.H ? eval_res.H->depth : -1)
                << "| f+error: " << (eval_res.H ? eval_res.H->f + f_error : -1)
                << "\n";

      if (eval_res.is_success && eval_res.H != nullptr) {
        eval_f_error_sum += eval_res.H->f + f_error;
        ++eval_successes;
      }
    }

    if (eval_successes > 0) {
      int score = eval_f_error_sum / eval_successes;
      std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Cand " << cand_idx << " score=" << score << "\n";
      if (score < best_candidate_score) {
        best_candidate_score = score;
        best_H_candidate = H_candidate;
        best_candidate_iters = cand_gen_iters;
      }
    }
  }

  //
  // Part 2: exploitation
  //
  int exploit_max_iters = static_cast<int>(best_candidate_iters * 1.5);
  std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Starting exploitation from best candidate (score=" << best_candidate_score << ", iters_budget=" << exploit_max_iters << ")\n";
  int restart_count = 0;
  while (!is_expired(deadline)) {
    double ratio = SPD_RATIOS[restart_count % SPD_RATIOS.size()];
    int depth = (H_goal->depth - best_H_candidate->depth) * ratio;
    HNode* H_start = H_goal;
    while (H_start->depth > depth) H_start = H_start->parent;
    std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Exploiting from ratio: " << ratio << ", depth: " << depth << "\n";
    auto exploit_res = run_lacam(H_start, exploit_max_iters, H_goal->g, H_goal->depth * 1.5);
    restart_count++;
  }

  //
  // extract solution from H_goal
  //

  Solution solution = backtrack(H_goal);
  for (auto p : EXPLORED) delete p.second;
  return solution;
}

ASHA_Planner::LaCAM_Res ASHA_Planner::run_lacam(HNode* H_from, int max_iterations, int upper_bound, int max_depth)
{
  HNode* H = H_from;
  int search_iter = 0;

  while (search_iter <= max_iterations && !is_expired(deadline) && H->depth <= max_depth && H->f <= upper_bound) {
    search_iter += 1;

    // low level search
    auto L = H->get_next_lowlevel_node(MT);
    if (L == nullptr) {
      H = H->parent;
      continue;
    }

    // create successors at the high-level search
    auto Q_to = Config(N, nullptr);
    for (auto d = 0; d < L->depth; ++d) Q_to[L->who[d]] = L->where[d];
    auto res = pibt->set_new_config(H->C, Q_to, H->order);
    delete L;

    // failed? retry
    if (!res) continue;

    if (is_same_config(Q_to, ins->goals)) {
      HNode* H_new = create_highlevel_node(Q_to, H);
      if (H_goal == nullptr) H_goal = H_new;
      if (H_new->g < H_goal->g) { 
        H_goal->g = H_new->g; 
        H_goal->h = H_new->h; 
        H_goal->f = H_new->f; 
        H_goal->parent = H_new->parent; 
        H_goal->depth = H_new->depth;
      }
      info(1, verbose, deadline, "found goal, cost: ", H_goal->g);
      return { H, search_iter, true, true };
    }

    // check explored list
    auto iter = EXPLORED.find(Q_to);
    if (iter != EXPLORED.end()) {
      auto g = H->g + get_edge_cost(H->C, Q_to);
      auto h = heuristic->get(Q_to);
      auto f = g + h;
      if (f >= iter->second->f && H_goal != nullptr) {
        // We found a worse (or equal) path to a known configuration.
        // Deterministically drop this branch. 
        // By continuing without pushing, OPEN.front() remains H, 
        // forcing H to generate a novel low-level node on the next iteration.
        continue;
      }
      // known configuration
      HNode* H_best = rewrite(H, iter->second);
      if (H_best != nullptr) {
        // 1. Surpassed or hit the depth budget
        if (H_best->depth >= max_depth) {
            while (H_best != nullptr && H_best->depth > max_depth) {
                H_best = H_best->parent;
            }
            // If it happens to be the goal exactly at max_depth
            bool is_goal = (H_best == H_goal); 
            return { H_best, search_iter, is_goal, true };
        }
        
        // 2. Goal found within depth budget
        if (H_best == H_goal) {
            return { H_best, search_iter, true, true };
        }
        
        // 3. Not at horizon, not the goal -> Teleport to the new frontier
        H = H_best;
        continue; 
      }      
      H = iter->second;
    } else {
      auto H_new = create_highlevel_node(Q_to, H);
      H = H_new;
    }
  }

  if (H->depth - 1 > max_depth || H->f > upper_bound)
    return { H, search_iter, false, false };

  return  { H, search_iter, false, true };
}

HNode *ASHA_Planner::create_highlevel_node(const Config &Q, HNode *parent)
{
  auto g_val =
      (parent == nullptr) ? 0 : parent->g + get_edge_cost(parent->C, Q);
  auto h_val = heuristic->get(Q);
  auto H_new = new HNode(Q, D, parent, g_val, h_val);
  EXPLORED[Q] = H_new;
  return H_new;
}

Solution ASHA_Planner::backtrack(HNode *H)
{
  std::vector<Config> plan;
  auto _H = H;
  while (_H != nullptr) {
    plan.push_back(_H->C);
    _H = _H->parent;
  }
  std::reverse(plan.begin(), plan.end());
  return plan;
}


HNode* ASHA_Planner::rewrite(HNode *H_from, HNode *H_to)
{
  H_from->neighbor.insert(H_to);
  
  HNode* H_best = nullptr;
  std::queue<HNode *> Q({H_from});  
  
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      
      if (g_val < n_to->g) {
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        n_to->depth = n_from->depth + 1;
        
        // Track the deepest updated node (tie-break with f-value)
        if (H_best == nullptr || n_to->depth > H_best->depth || 
           (n_to->depth == H_best->depth && n_to->f < H_best->f)) {
            H_best = n_to;
        }
        
        if (n_to == H_goal) {
          info(2, verbose, deadline, "cost update: ", g_val);
          return H_goal;
        }
        Q.push(n_to);
      }
    }
  }
  return H_best; // Could be nullptr if graph was somehow locked
}

int ASHA_Planner::get_edge_cost(const Config &C1, const Config &C2)
{
  auto cost = 0;
  for (uint i = 0; i < N; ++i) {
    if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
      cost += 1;
    }
  }
  return cost;
}

void ASHA_Planner::set_scatter()
{
  if (!Planner::FLG_SCATTER) return;
  info(1, verbose, deadline, "start computing SUO");
  auto scatter_deadline =
      Deadline(deadline == nullptr
                   ? INT_MAX
                   : (deadline->time_limit_ms - elapsed_ms(deadline)) / 2);
  auto margin = Planner::SCATTER_MARGIN < 0 ? get_random_int(MT, 0, 30) : Planner::SCATTER_MARGIN;
  scatter = new Scatter(ins, D, &scatter_deadline, 3, verbose - 4, margin);
  scatter->construct();
  info(1, verbose, deadline, "finish computing SUO",
       ", collision count: ", scatter->CT.collision_cnt,
       ", scatter margin: ", scatter->cost_margin,
       ", sum_of_path_length: ", scatter->sum_of_path_length);
}

void ASHA_Planner::set_pibt()
{
  pibt = new PIBT(ins, D, seed, Planner::FLG_SWAP, scatter);
}

void ASHA_Planner::logging()
{
}

