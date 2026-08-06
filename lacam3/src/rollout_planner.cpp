#include "../include/rollout_planner.hpp"
#include "../include/planner.hpp"

#include <algorithm>
#include <iostream>


std::string RolloutPlanner::MSG;

constexpr auto TIME_ZERO = std::chrono::seconds(0);


RolloutPlanner::RolloutPlanner(const Instance *_ins, int _verbose, const Deadline *_deadline,
                 int _seed, int _depth, DistTable *_D)
    : ins(_ins),
      deadline(_deadline),
      seed(_seed),
      MT(std::mt19937(seed)),
      verbose(_verbose),
      depth(_depth),
      N(ins->N),
      V_size(ins->G->size()),
      D((_D == nullptr) ? new DistTable(ins) : _D),
      delete_dist_table_after_used(_D == nullptr),
      heuristic(new Heuristic(ins, D)),
      scatter(nullptr),
      seed_refiner(0),
      refiner_pool(),
      H_init(nullptr),
      H_goal(nullptr),
      search_iter(0),
      time_initial_solution(-1),
      cost_initial_solution(-1),
      lnode_gen(&MT, D)
{
}

RolloutPlanner::~RolloutPlanner()
{
  if (heuristic != nullptr) delete heuristic;
  if (scatter != nullptr) delete scatter;
  for (auto &pibt : pibts) delete pibt;
  if (delete_dist_table_after_used) delete D;
}

Solution RolloutPlanner::solve()
{
  info(1, verbose, deadline, "RolloutPlanner: start search");

  // insert initial node
  H_init = create_highlevel_node(ins->starts, nullptr);
  HNode* H = H_init;
  LNode* EMPTY_LNODE = new LNode();
  Solution solution;
  int best_cost = 999999999;

  set_scatter();
  set_pibt();

  // search loop
  while (!is_expired(deadline)) {
    search_iter += 1;

    // check pooled procedures
    refiner_pool.remove_if([&](auto &proc) {
      if ((proc).wait_for(TIME_ZERO) != std::future_status::ready) return false;
      Solution s = proc.get();
      if (!s.empty()) solution = s;
      ++seed_refiner;
      refiner_pool.emplace_back(std::async(std::launch::async,
                                           &RolloutPlanner::get_refined_plan, this,
                                           backtrack(H_goal)));
      return true;
    });


    // check lower bounds
    if (!solution.empty() && H->f >= best_cost) {
      H = H_init;
      continue;
    }

    // check goal condition
    if (is_same_config(H->C, ins->goals)) {
      time_initial_solution = elapsed_ms(deadline);
      cost_initial_solution = H->g;
      H_goal = H;
      info(1, verbose, deadline, "found solution, cost: ", H_goal->g);
      solution = backtrack(H_goal);
      best_cost = H->g;
      if (!Params::FLG_STAR) break;  // finish search
      set_refiner(solution);          // refining start
      continue;
    }

    // create successors at the high-level search
    auto Q_to = Config(N, nullptr);
    auto res = set_new_config(H, lnode_gen.generate(H), Q_to);
    // auto res = set_new_config(H, EMPTY_LNODE, Q_to);
    if (!res) {
      H = H_init;
      continue;
    }

    auto g_val = H->g + get_edge_cost(H->C, Q_to);
    auto h_val = heuristic->get(Q_to);
    auto f_val = g_val + h_val;

    if (!solution.empty()) {
      if (f_val >= best_cost) {
        H = H_init;
        continue;
      }
    }
    auto H_new = create_highlevel_node(Q_to, H);
    H = H_new;
  }

  // clear pooled operaitons
  bool is_optimal = false;
  for (auto &proc : refiner_pool) proc.get();

  // end processing
  logging();
  return solution;
}

HNode *RolloutPlanner::create_highlevel_node(const Config &Q, HNode *parent)
{
  auto g_val =
      (parent == nullptr) ? 0 : parent->g + get_edge_cost(parent->C, Q);
  auto h_val = heuristic->get(Q);
  auto H_new = new HNode(Q, D, parent, g_val, h_val);
  return H_new;
}

Solution RolloutPlanner::backtrack(HNode *H)
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

bool RolloutPlanner::set_new_config(HNode *H, LNode *L, Config &Q_to)
{
  // int depth = 0;
  // for (auto* node = H; node != nullptr; node = node->parent) depth++;
  // if (depth < 10) {
  //   for (auto d = 0; d < L->depth; ++d) Q_to[L->who[d]] = L->where[d];
  //   return pibts[0]->set_new_config(H->C, Q_to, H->order);
  // }
  for (int t = 0; t < 10; t++) {
    for (auto d = 0; d < L->depth; ++d) Q_to[L->who[d]] = L->where[d];
    bool res = pibts[0]->set_new_config(H->C, Q_to, H->order);
    if (res) return true;
    L = lnode_gen.generate(H);
  }
  return false;

  // worker-id, time -> configuration
  auto Q_cands = std::vector<Config>(Params::PIBT_NUM, Config(N, nullptr));
  auto f_vals = std::vector<int>(Params::PIBT_NUM, INT_MAX);
  auto is_success = std::vector<bool>(Params::PIBT_NUM, false);

  // parallel monte carlo
  auto worker = [&](int k) {
    // set constraints
    for (auto d = 0; d < L->depth; ++d) Q_cands[k][L->who[d]] = L->where[d];
    // PIBT
    auto res = pibts[k]->set_new_config(H->C, Q_cands[k], H->order);
    if (res) {
      f_vals[k] = get_edge_cost(H->C, Q_cands[k]) + heuristic->get(Q_cands[k]);
      is_success[k] = true;
    }
  };
  if (Params::FLG_MULTI_THREAD && Params::PIBT_NUM > 1) {
    auto threads = std::vector<std::thread>();
    for (auto k = 0; k < Params::PIBT_NUM; ++k) threads.emplace_back(worker, k);
    for (auto &th : threads) th.join();
  } else {
    for (auto k = 0; k < Params::PIBT_NUM; ++k) worker(k);
  }

  // obtain the best score
  auto min_f_val = INT_MAX;
  auto min_f_val_idx = -1;
  for (auto k = 0; k < Params::PIBT_NUM; ++k) {
    if (!is_success[k]) continue;
    // min_f_val = f_vals[k];
    // min_f_val_idx = k;
    // break;
    if (f_vals[k] < min_f_val) {
      min_f_val = f_vals[k];
      min_f_val_idx = k;
    }
  }

  if (min_f_val < INT_MAX) {
    auto &Q_win = Q_cands[min_f_val_idx];
    std::copy(Q_win.begin(), Q_win.end(), Q_to.begin());
    return true;
  } else {
    return false;
  }
}

int RolloutPlanner::get_edge_cost(const Config &C1, const Config &C2)
{
  auto cost = 0;
  for (uint i = 0; i < N; ++i) {
    if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
      cost += 1;
    }
  }
  return cost;
}

void RolloutPlanner::set_scatter()
{
  if (!Params::FLG_SCATTER) return;
  info(1, verbose, deadline, "start computing SUO");
  auto scatter_deadline =
      Deadline(deadline == nullptr
                   ? INT_MAX
                   : (deadline->time_limit_ms - elapsed_ms(deadline)) / 2);
  auto margin = Params::SCATTER_MARGIN < 0 ? get_random_int(MT, 0, 30) : Params::SCATTER_MARGIN;
  scatter = new Scatter(ins, D, &scatter_deadline, 3, verbose - 4, margin);
  scatter->construct(5);
  info(1, verbose, deadline, "finish computing SUO",
       ", collision count: ", scatter->CT.collision_cnt,
       ", scatter margin: ", scatter->cost_margin,
       ", sum_of_path_length: ", scatter->sum_of_path_length);
}

void RolloutPlanner::set_pibt()
{
  for (auto k = 0; k < Params::PIBT_NUM; ++k) {
    pibts.emplace_back(new PIBT(ins, D, k + seed, Params::FLG_SWAP, scatter));
  }
}

void RolloutPlanner::set_refiner(Solution& solution)
{
  if (!Params::FLG_REFINER) return;
  if (!Params::FLG_MULTI_THREAD) return;
  info(2, verbose, deadline, "invoke refiners");
  for (auto k = 0; k < Params::REFINER_NUM; ++k) {
    ++seed_refiner;
    refiner_pool.emplace_back(
        std::async(std::launch::async, &RolloutPlanner::get_refined_plan, this, solution));
  }
}

Solution RolloutPlanner::get_refined_plan(const Solution &plan)
{
  auto MT_internal = std::mt19937(seed_refiner);
  return refine(ins, deadline, plan, D, seed_refiner, verbose - 4);
}

void RolloutPlanner::logging()
{
  if (depth > 0) return;
  MSG +=
      "\ncomp_time_initial_solution=" + std::to_string(time_initial_solution);
  MSG += "\ncost_initial_solution=" + std::to_string(cost_initial_solution);
  MSG += "\nsearch_iteration=" + std::to_string(search_iter);
  MSG += "\nnum_high_level_node=" + std::to_string(HNode::COUNT);
  MSG += "\nnum_low_level_node=" + std::to_string(LNode::COUNT);

if (H_goal != nullptr) {
    info(1, verbose, deadline, "solved sub-optimally, cost:", H_goal->g);
  } else {
    info(1, verbose, deadline, "timeout");
  }
  info(1, verbose, deadline, "search iteration:", search_iter);
}
