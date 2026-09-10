#include "../include/asha_planner.hpp"
#include "../include/planner.hpp" // for the flags
#include <algorithm>
#include <iostream>

// int DEBUG_AGENT = 50;
int DEBUG_AGENT = 51;


const int PIBT_DEADLOCK_ATTEMPTS = 500;
constexpr auto TIME_ZERO = std::chrono::seconds(0);
const std::vector<double> SPD_RATIOS = {0.0, 0.0, 0.25, 0.35, 0.45, 0.55, 0.6};

const double CANDIDATE_HORIZON = 0.3;
const int KILL_AFTER_STALE_TRIALS = 5;
const int NUM_OF_CANDIDATES = 3;

struct Candidate {
  static int next_id;
  // Candidate() { id = next_id++; }
  int    id;
  HNode* H;
  int    min_cost;
  int    trials_since_last_update;
};

int Candidate::next_id = 0;

static bool should_kill(const std::vector<Candidate>& candidates, int c);

ASHA_Planner::ASHA_Planner(const Instance *_ins, int _verbose, const Deadline *_deadline,
                 int _seed, DistTable *_D, const Config *_prefix_goals)
    : ins(_ins),
      deadline(_deadline),
      seed(_seed),
      MT(std::mt19937(seed)),
      verbose(_verbose),
      N(ins->N),
      V_size(ins->G->size()),
      D((_D == nullptr) ? new DistTable(ins) : _D),
      delete_dist_table_after_used(false),
      FLG_PREFIX_REFINEMENT(_prefix_goals != nullptr),
      prefix_goals(_prefix_goals != nullptr ? *_prefix_goals : Config{}),
      dmt((_prefix_goals != nullptr) ? static_cast<DoubleModeDistTable *>(_D) : nullptr),
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
  // if (heuristic != nullptr) delete heuristic;
  // if (scatter != nullptr) delete scatter;
  // delete pibt;
}

HNode* go_back_to_depth(HNode* H_from, int new_depth) {
  HNode* H = H_from;
  while (H->depth > new_depth) H = H->parent;
  return H;
}

inline HNode* go_back_to_depth_ratio(HNode* H_from, double ratio) {
  return go_back_to_depth(H_from, H_from->depth * ratio);
}

Solution ASHA_Planner::solve()
{
  // setup
  set_scatter();
  set_pibt();

  //
  // Find initial solution
  //

  H_init = create_highlevel_node(ins->starts, nullptr);

  std::cout << "[ASHA] elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  "
            << "Finding initial solution from H_init... " 
            << std::endl;

  auto res_init = run_lacam(H_init, INT_MAX, INT_MAX, INT_MAX);

  if (!res_init.is_goal) {
    std::cout << "[ASHA] elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  "
              << "Failed" 
              << std::endl;
    for (auto p : EXPLORED) delete p.second;
    return Solution();
  }

  //
  // Refine prefix
  //
  HNode* H_start = refine_prefix(res_init);
  if (H_start == nullptr) return backtrack(H_goal);

  //
  // Refine suffix
  //
  int iter = 0;
  H_goal = nullptr;
  while (!is_expired(deadline)) {
    std::cout << "[ASHA] elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  "
              << "Finding suffix. iter: " << iter
              << std::endl;
    int UB = H_goal == nullptr ? res_init.H->g * 1.25 : H_goal->g;
    int max_depth = H_goal == nullptr ? res_init.H->depth * 1.5 : H_goal->depth * 1.5;
    if (H_goal != nullptr) {
      int depth = SPD_RATIOS[iter % SPD_RATIOS.size()] * H_goal->depth;
      HNode* H_mid = go_back_to_depth(H_goal, depth);
      run_lacam(H_mid, res_init.iterations, UB, max_depth);
    }
    else run_lacam(H_start, res_init.iterations, UB, max_depth);
    iter++;
  }

  //
  // extract solution from H_goal
  //

  Solution solution = backtrack(H_goal);
  for (auto p : EXPLORED) delete p.second;
  return solution;
}

HNode* ASHA_Planner::refine_prefix(LaCAM_Res& res_init) {
  HNode* p_H_goal = go_back_to_depth_ratio(H_goal, 0.15);
  Config prefix_goals_cfg = p_H_goal->C;

  // D (toward real goals) already exists; build D_prefix toward prefix goals
  Instance p_ins_for_d(ins->G, ins->starts, prefix_goals_cfg, ins->N);
  // auto *D_prefix = new DistTable(&p_ins_for_d);
  // auto *p_dmt = new DoubleModeDistTable(ins->G->size(), D_prefix, D);

  ASHA_Planner planner(&p_ins_for_d, verbose, deadline,
                        get_random_int(MT, 0, 10000000));
  planner.scatter = scatter;
  // planner.set_scatter();
  planner.set_pibt();
  int MAX_FAILURES = 100;
  int trials_left = MAX_FAILURES;
  int max_depth =  p_H_goal->depth + 2;
  int best_cost = p_H_goal->g * 1.05;
  int upper_bound = best_cost * 1.05;
  int max_iters = res_init.iterations;
  HNode* H_best = nullptr;
  HNode* p_H_init = planner.create_highlevel_node(ins->starts, nullptr);

  std::cout << "[ASHA] elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  "
            << "Running prefix refinement. "
            << ", mid depth: " << p_H_goal->depth << ", mid g: " << p_H_goal->g
            << ", UB: " << upper_bound
            << std::endl;

  while (!is_expired(deadline) && trials_left--) {
    //p_H_init->reset_tree();
    auto res = planner.run_lacam(p_H_init, max_iters, upper_bound, max_depth);
    if (res.is_goal && res.H->g < best_cost) {
      std::cout << "Cost update: " << res.H->g
                << "\t at depth: " << res.H->depth
                << std::endl;
      best_cost = res.H->g;
      H_best = res.H;
      trials_left = MAX_FAILURES;
      upper_bound = best_cost - 1;
    } else {
      // int num_reached = std::count(res.H->agent_modes.begin(), res.H->agent_modes.end(), true);
      std::cout << "[ASHA] elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  "
                << "is_success: " << res.is_success 
                << ", depth: " << res.H->depth << ", max_depth: " << max_depth
                << ", iters: " << res.iterations << ", max_iters: " << max_iters 
                << ", g: " << res.H->g << ", UB: " << upper_bound
//                << ", reached_mid_goals: " << num_reached << "/" << N
                << "\t FAILED" << std::endl;
    }
  }
  // delete D_prefix;
  // delete p_dmt;
  if (H_best == nullptr) return nullptr;
  H_best = go_back_to_depth(H_best, p_H_goal->depth);

  // Collect original plan nodes indexed by depth
  std::vector<HNode *> orig_by_depth(p_H_goal->depth + 1, nullptr);
  for (HNode *H = p_H_goal; H != nullptr; H = H->parent)
    orig_by_depth[H->depth] = H;

  // Collect refined plan nodes indexed by depth
  std::vector<HNode *> refined_by_depth(H_best->depth + 1, nullptr);
  for (HNode *H = H_best; H != nullptr; H = H->parent)
    refined_by_depth[H->depth] = H;

  // Recalculate g, h, f for each refined plan node bottom-up from root
  for (int d = 0; d <= H_best->depth; ++d) {
    HNode *H = refined_by_depth[d];
    if (H == nullptr) continue;
    if (d == 0) {
      H->g = 0;
    } else {
      HNode *par = refined_by_depth[d - 1];
      H->g = par->g + get_edge_cost(par->C, H->C, nullptr);
    }
    H->h = heuristic->get(H->C);
    H->f = H->g + H->h;
  }

  // Find depth where f_refined - f_orig is maximal (only where both exist)
  HNode *best_start = H_best;
  int max_delta = INT_MIN;
  int shared_depth = std::min((int)orig_by_depth.size(), (int)refined_by_depth.size()) - 1;
  for (int d = 0; d <= shared_depth; ++d) {
    if (orig_by_depth[d] == nullptr || refined_by_depth[d] == nullptr) continue;
    int delta = refined_by_depth[d]->f - orig_by_depth[d]->f;
    if (delta > max_delta) {
      max_delta = delta;
      best_start = refined_by_depth[d];
    }
  }

  std::cout << "[ASHA] elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  "
            << "Returning H_start at depth: " << best_start->depth << ", g: " << best_start->g
            << std::endl;  

  exit(0);            
  return best_start;
}


static bool should_kill(const std::vector<Candidate>& candidates, int c)
{
  const auto& cand = candidates[c];
  return cand.H == nullptr || cand.trials_since_last_update >= KILL_AFTER_STALE_TRIALS;
}

ASHA_Planner::LaCAM_Res ASHA_Planner::run_lacam(HNode* H_from, int max_iterations, int upper_bound, int max_depth)
{
  HNode* H = H_from;
  int search_iter = 0;

  while (search_iter <= max_iterations && !is_expired(deadline) && H->depth <= max_depth && H->g <= upper_bound) {
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

    bool pibt_res;
    if (FLG_PREFIX_REFINEMENT) {
      dmt->set_active_modes(&H->agent_modes);
      const Config& orig_goals = ins->goals;
      for (int i = 0; i < N; ++i)
        pibt->goals[i] = H->agent_modes[i] ? orig_goals[i] : prefix_goals[i];
      pibt_res = pibt->set_new_config(H->C, Q_to, H->order);

      // auto v_str = [&](Vertex* v) -> std::string {
      //   return "(" + std::to_string(v->x) + ", " + std::to_string(v->y) + ")";
      // };

      // std::cout << "[DEBUG agent=" << DEBUG_AGENT << "]"
      //           << " depth: " << H->depth
      //           << ", reached mid: " << H->agent_modes[DEBUG_AGENT]
      //           << ", pibt.goal: " << v_str(pibt->goals[DEBUG_AGENT])
      //           << ", real goal: " << v_str(ins->goals[DEBUG_AGENT])
      //           << ", mid goal: " << v_str(prefix_goals[DEBUG_AGENT])
      //           << ", D: " << dmt->get(DEBUG_AGENT, H->C[DEBUG_AGENT])
      //           << std::endl;

      pibt->goals = orig_goals;

    } else {
      pibt_res = pibt->set_new_config(H->C, Q_to, H->order);
    }
    delete L;

    // failed? retry
    if (!pibt_res) continue;

    // compute new modes to check success condition
    const bool is_goal = [&]() -> bool {
      if (!FLG_PREFIX_REFINEMENT) return is_same_config(Q_to, ins->goals);
      // success when all agents reach their prefix goals
      for (int i = 0; i < N; ++i) {
        if (!H->agent_modes[i] && Q_to[i] != prefix_goals[i]) return false;
      }
      return true;
    }();

    if (is_goal) {
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
      return { H_goal, search_iter, true, true };
    }

    // check explored list before creating a node, to avoid overwriting EXPLORED
    auto iter = EXPLORED.find(Q_to);
//    if (iter != EXPLORED.end() && !FLG_PREFIX_REFINEMENT) {
    if (iter != EXPLORED.end()) {
      auto* H_existing = iter->second;
      // compute tentative g for the path through H
      // (create_highlevel_node would do this; replicate cheaply here)
      auto tentative_g = H->g + get_edge_cost(H->C, Q_to,
                                              FLG_PREFIX_REFINEMENT ? &H_existing->agent_modes : nullptr);
      auto tentative_f = tentative_g + H_existing->h;
      if (tentative_f >= H_existing->f && H_goal != nullptr) {
        // We found a worse (or equal) path to a known configuration.
        // Deterministically drop this branch. 
        // By continuing without pushing, OPEN.front() remains H, 
        // forcing H to generate a novel low-level node on the next iteration.
        continue;
      }
      // known configuration
      HNode* H_best = rewrite(H, H_existing);
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
      H = H_existing;
    } else {
      auto H_new = create_highlevel_node(Q_to, H);
      H = H_new;
    }
  }

  if (H->depth - 1 > max_depth || H->g > upper_bound)
    return { H, search_iter, false, false };

  return  { H, search_iter, false, true };
}

HNode *ASHA_Planner::create_highlevel_node(const Config &Q, HNode *parent)
{
  std::vector<bool> new_modes;
  if (FLG_PREFIX_REFINEMENT) {
    new_modes = parent ? parent->agent_modes : std::vector<bool>(N, false);
    for (int i = 0; i < N; ++i)
      if (!new_modes[i] && Q[i] == prefix_goals[i]) new_modes[i] = true;
  }

  auto g_val = (parent == nullptr)
                   ? 0
                   : parent->g + get_edge_cost(parent->C, Q,
                                               FLG_PREFIX_REFINEMENT ? &new_modes : nullptr);
  auto h_val = FLG_PREFIX_REFINEMENT ? heuristic->get(Q, &new_modes) : heuristic->get(Q);
  auto H_new = new HNode(Q, D, parent, g_val, h_val,
                         FLG_PREFIX_REFINEMENT ? new_modes : std::vector<bool>());

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
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C,
                                              FLG_PREFIX_REFINEMENT ? &n_to->agent_modes : nullptr);
      
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

int ASHA_Planner::get_edge_cost(const Config &C1, const Config &C2,
                                const std::vector<bool> *modes)
{
  auto cost = 0;
  for (uint i = 0; i < N; ++i) {
    // if (modes && (*modes)[i]) continue;  // phase 2: free
    // const Vertex *goal = (modes && FLG_PREFIX_REFINEMENT) ? prefix_goals[i] : ins->goals[i];
    const Vertex *goal = ins->goals[i];
    if (C1[i] != goal || C2[i] != goal) cost += 1;
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
  pibt = new PIBT(ins->G, ins->goals, D, seed, Planner::FLG_SWAP, scatter);
}

void ASHA_Planner::logging()
{
}

