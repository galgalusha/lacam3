#include "../include/asha_planner.hpp"
#include "../include/planner.hpp" // for the flags
#include <algorithm>
#include <iostream>


const int PIBT_DEADLOCK_ATTEMPTS = 500;
constexpr auto TIME_ZERO = std::chrono::seconds(0);
const std::vector<double> SPD_RATIOS = {0.0, 0.0, 0.0, 0.0, 0.25, 0.35, 0.45, 0.55, 0.6};

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

  int max_iterations = res_init.iterations * 3;

  auto create_candidate = [&]() -> Candidate {
    int upper_bound = H_goal->g; // TODO: it should be a friction of H_goal->g, but we don't yet know how much
    auto res = run_lacam(H_init, max_iterations, upper_bound, static_cast<int>(H_goal->depth * CANDIDATE_HORIZON));
    HNode* H = (res.is_success && res.H != nullptr) ? res.H : nullptr;
    std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms create_candidate: depth="
              << (H ? H->depth : -1) << "\n";
    return { Candidate::next_id++, H, INT_MAX, 0};
  };

  std::vector<Candidate> candidates;
  candidates.reserve(NUM_OF_CANDIDATES);
  for (int i = 0; i < NUM_OF_CANDIDATES; ++i) candidates.push_back(create_candidate());

  //
  // Main loop: iterate over candidates, run all SPD_RATIOS, refresh stale ones
  //
  while (!is_expired(deadline)) {
    for (int c = 0; c < (int)candidates.size() && !is_expired(deadline); ++c) {
      if (should_kill(candidates, c)) {
        std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms Killing candidate " << c << " (stale), replacing.\n";
        candidates[c] = create_candidate();
      }

      auto& cand = candidates[c];
      if (cand.H == nullptr) continue;

      int cost_before_trial = H_goal->g;

      // for (double ratio : SPD_RATIOS) {
      //   if (is_expired(deadline)) break;
      //   int depth = cand.H->depth + static_cast<int>((H_goal->depth - cand.H->depth) * ratio);
      //   HNode* H_start = H_goal;
      //   while (H_start != nullptr && H_start->depth > depth) H_start = H_start->parent;
      //   if (H_start == nullptr) H_start = cand.H;

      //   std::cout << "[ASHA] t=" << elapsed_ms(deadline) << "ms cand=" << cand.id
      //             << " ratio=" << ratio << " depth=" << depth << " best_cost=" << H_goal->g << "\n";
      //   run_lacam(H_start, max_iterations, H_goal->g, H_goal->depth);
      // }
      run_lacam(cand.H, max_iterations, H_goal->g, H_goal->depth);

      if (H_goal->g < cost_before_trial) {
        cand.min_cost = H_goal->g;
        cand.trials_since_last_update = 0;
      } else {
        ++cand.trials_since_last_update;
      }
    }
  }

  //
  // extract solution from H_goal
  //

  Solution solution = backtrack(H_goal);
  for (auto p : EXPLORED) delete p.second;
  return solution;
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

