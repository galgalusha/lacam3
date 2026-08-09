#include "../include/planner.hpp"
#include "../include/arm.hpp"
#include "../include/metrics.hpp"
#include "../include/wait_scatter.hpp"

#include <algorithm>
#include <iostream>

bool Planner::FLG_SWAP = true;
bool Planner::FLG_STAR = true;
bool Planner::FLG_MULTI_THREAD = true;
int Planner::SCATTER_MARGIN = 10;
int Planner::PIBT_NUM = 10;
bool Planner::FLG_REFINER = true;
int Planner::REFINER_NUM = 4;
bool Planner::FLG_SCATTER = true;
float Planner::RANDOM_INSERT_PROB1 = 0.1;
float Planner::RANDOM_INSERT_PROB2 = 0.01;
bool Planner::FLG_RANDOM_INSERT_INIT_NODE = false;
float Planner::RECURSIVE_RATE = 0.2;
double Planner::RECURSIVE_TIME_LIMIT = 1000;

std::string Planner::MSG;
int Planner::CHECKPOINTS_DURATION = 5000;
constexpr int CHECKPOINTS_NIL = -1;

const int PIBT_DEADLOCK_ATTEMPTS = 150;
const int PIBT_LIVELOCK_ATTEMPTS = 150;

constexpr auto TIME_ZERO = std::chrono::seconds(0);


Planner::Planner(const Instance *_ins, int _verbose, const Deadline *_deadline,
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
      EXPLORED(),
      H_init(nullptr),
      H_goal(nullptr),
      arm_idx(-1),
      search_iter(0),
      time_initial_solution(-1),
      cost_initial_solution(-1),
      checkpoints()
{
}

Planner::~Planner()
{
  if (heuristic != nullptr) delete heuristic;
  if (scatter != nullptr) {
    auto &mab = (depth == 0) ? main_scatter_mab : refiners_scatter_mab;
    if (mab.get_cached(arm_idx, ins->starts) != scatter) delete scatter;
  }
  for (auto &pibt : pibts) delete pibt;
  if (delete_dist_table_after_used) delete D;
}

int get_depth(HNode *H) {
  int depth = 0;
  while (H->parent != nullptr) { H = H->parent; depth++; }
  return depth;
}

Solution Planner::solve()
{
  info(1, verbose, deadline, "start search");
  update_checkpoints();

  // insert initial node
  H_init = create_highlevel_node(ins->starts, nullptr);
  HNode* H = H_init;

  set_scatter();
  set_pibt();

  int pibt_deadlock_attempts = PIBT_DEADLOCK_ATTEMPTS;
  int pibt_livelock_attempts = PIBT_LIVELOCK_ATTEMPTS;
  int restart_count = 0;

  auto do_restart = [&]() {
    pibt_deadlock_attempts = PIBT_DEADLOCK_ATTEMPTS;
    pibt_livelock_attempts = PIBT_LIVELOCK_ATTEMPTS;
    restart_count++;
    H = H_init;
    int new_arm = main_scatter_mab.choose_ready_arm(ins->starts);
    if (new_arm >= 0) {
      arm_idx = new_arm;
      scatter = main_scatter_mab.get_cached(arm_idx, ins->starts);
      for (auto &pibt : pibts) pibt->scatter = scatter;
      main_scatter_mab.record_pull(arm_idx);
      info(2, verbose, deadline, "restart with arm", arm_idx,
           main_scatter_mab.arms[arm_idx].config.str());
    }
  };

  // search loop
  while (!is_expired(deadline)) {
    search_iter += 1;
    update_checkpoints();

    // check pooled procedures
    refiner_pool.remove_if([&](auto &proc) {
      if (proc.wait_for(TIME_ZERO) != std::future_status::ready) return false;
      apply_new_solution(proc.get());
      ++seed_refiner;
      refiner_pool.emplace_back(std::async(std::launch::async,
                                           &Planner::get_refined_plan, this,
                                           backtrack(H_goal)));
      return true;
    });

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      if (depth > 0) break;
      do_restart();
      continue;
    }

    // check goal condition
    if (H_goal == nullptr && is_same_config(H->C, ins->goals)) {
      time_initial_solution = elapsed_ms(deadline);
      cost_initial_solution = H->g;
      H_goal = H;
      info(1, verbose, deadline, "found initial solution, cost: ", H_goal->g);
      if (!FLG_STAR || depth > 0) break;  // finish search
      set_refiner();         // refining start
      continue;
    }

    // low level search
    auto L = H->get_next_lowlevel_node(MT);
    if (L == nullptr) {
      H = H->parent;
      continue;
    }

    // create successors at the high-level search
    auto Q_to = Config(N, nullptr);
    bool res = set_new_config(H, L, Q_to);

    delete L;

    // failed? retry
    if (!res) {
      if (--pibt_deadlock_attempts == 0) {
        if (depth > 0) break;
        do_restart();
      }
      continue;
    };

    // check explored list
    auto iter = EXPLORED.find(Q_to);

    if (iter != EXPLORED.end()) {
      auto g = (H->parent == nullptr) ? 0 : H->parent->g + get_edge_cost(H->parent->C, Q_to);
      auto h = heuristic->get(Q_to);
      auto f = g + h;
      if (f >= iter->second->f) {
        if (--pibt_livelock_attempts == 0) {
          if (depth > 0) break;
          do_restart();
          continue;
        }
      }
      // known configuration
      rewrite(H, iter->second, Refiner::LaCAM, arm_idx);
      H = iter->second;
    } else {
      // new one -> insert
      auto H_new = create_highlevel_node(Q_to, H);
      H = H_new;
    }
  }

  // clear pooled operaitons
  bool is_optimal = false;
  for (auto &proc : refiner_pool) apply_new_solution(proc.get());

  // end processing
  update_checkpoints();
  logging();
  auto solution = backtrack(H_goal);        // obtain solution
  for (auto p : EXPLORED) delete p.second;  // memory management
  return solution;
}

HNode *Planner::create_highlevel_node(const Config &Q, HNode *parent)
{
  auto g_val =
      (parent == nullptr) ? 0 : parent->g + get_edge_cost(parent->C, Q);
  auto h_val = heuristic->get(Q);
  auto H_new = new HNode(Q, D, parent, g_val, h_val);
  EXPLORED[Q] = H_new;
  return H_new;
}

void Planner::apply_new_solution(const RefinedPlan &result)
{
  const auto &plan = result.solution;
  const auto caller = result.caller;
  if (plan.empty()) return;
  info(3, verbose, deadline, "incorporate new solution");

  // forcibly insert configuration
  HNode *H_from = EXPLORED[plan[0]];
  HNode *H_to = nullptr;
  for (auto t = 1; t < plan.size(); ++t) {
    auto &&Q = plan[t];
    auto iter = EXPLORED.find(Q);
    if (iter != EXPLORED.end()) {
      // known
      H_to = iter->second;
      rewrite(H_from, H_to, caller, result.arm_idx);
    } else {
      // new
      auto g_val = H_from->g + get_edge_cost(H_from->C, Q);
      H_to = new HNode(Q, D, H_from, g_val, heuristic->get(Q));
      EXPLORED[Q] = H_to;
    }
    H_from = H_to;
  }
}

Solution Planner::backtrack(HNode *H)
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

bool Planner::set_new_config(HNode *H, LNode *L, Config &Q_to)
{
  // worker-id, time -> configuration
  auto Q_cands = std::vector<Config>(PIBT_NUM, Config(N, nullptr));
  auto f_vals = std::vector<int>(PIBT_NUM, INT_MAX);

  // parallel
  auto worker = [&](int k) {
    // set constraints
    for (auto d = 0; d < L->depth; ++d) Q_cands[k][L->who[d]] = L->where[d];
    // PIBT
    auto res = pibts[k]->set_new_config(H->C, Q_cands[k], H->order, get_depth(H));
    if (res)
      f_vals[k] = get_edge_cost(H->C, Q_cands[k]) + heuristic->get(Q_cands[k]);
  };
  if (FLG_MULTI_THREAD && PIBT_NUM > 1) {
    auto threads = std::vector<std::thread>();
    for (auto k = 0; k < PIBT_NUM; ++k) threads.emplace_back(worker, k);
    for (auto &th : threads) th.join();
  } else {
    for (auto k = 0; k < PIBT_NUM; ++k) worker(k);
  }

  // obtain the best score
  auto min_f_val = INT_MAX;
  auto min_f_val_idx = -1;
  for (auto k = 0; k < PIBT_NUM; ++k) {
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

void Planner::rewrite(HNode *H_from, HNode *H_to, Refiner caller, int arm_idx)
{
  // update neighbors
  H_from->neighbor.insert(H_to);

  // Dijkstra
  std::queue<HNode *> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      if (g_val < n_to->g) {
        if (n_to == H_goal) {
          const char *name = caller == Refiner::LaCAM          ? "LaCAM"
                             : caller == Refiner::RecursiveLaCAM ? "Recursive LaCAM"
                                                                  : "SIPP";
          info(2, verbose, deadline, "cost update [", name, "]: ", H_goal->g,
               " -> ", g_val);
          if (caller == Refiner::RecursiveLaCAM && arm_idx >= 0)
            refiners_scatter_mab.record_improvement(arm_idx);
          else if (caller == Refiner::LaCAM && arm_idx >= 0)
            main_scatter_mab.record_improvement(arm_idx);
        }
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        Q.push(n_to);
      }
    }
  }
}

int Planner::get_edge_cost(const Config &C1, const Config &C2)
{
  auto cost = 0;
  for (uint i = 0; i < N; ++i) {
    if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
      cost += 1;
    }
  }
  return cost;
}

void Planner::set_scatter()
{
  if (!FLG_SCATTER) return;
  info(1, verbose, deadline, "start computing SUO");
  auto scatter_deadline =
      new Deadline(deadline == nullptr
                   ? INT_MAX
                   : (deadline->time_limit_ms - elapsed_ms(deadline)) / 2);

  if (depth > 0) {
    auto &chosen_arm = refiners_scatter_mab.arms[arm_idx >= 0 ? arm_idx : 0];
    scatter = refiners_scatter_mab.get_cached(arm_idx, ins->starts);
    if (scatter == nullptr) {
      auto arm_margin = chosen_arm.config.margin;
      if (chosen_arm.config.scatter_type == ST_Scatter) {
        scatter = new Scatter(ins, D, scatter_deadline, 3, verbose - 4, arm_margin);
      } else {
        scatter = new WaitScatter(ins, D, scatter_deadline, 3, verbose - 4, arm_margin);
      }
      scatter->construct(5);
      refiners_scatter_mab.set_cached(arm_idx, ins->starts, scatter);
    }

  } else {
    int time_limit = deadline->time_limit_ms;
    std::vector<std::thread> workers;
    for (int i = 0; i < (int)main_scatter_mab.arms.size(); ++i) {
      auto dl = new Deadline(deadline->time_limit_ms);
      workers.emplace_back([this, i, dl]() {
        auto &arm = main_scatter_mab.arms[i];
        IScatter *s;
        if (arm.config.scatter_type == ST_Scatter)
          s = new Scatter(ins, D, dl, 3, verbose - 4, arm.config.margin);
        else
          s = new WaitScatter(ins, D, dl, 3, verbose - 4, arm.config.margin);
        s->construct(5);
        std::cout << "elapsed: " << dl->elapsed_ms() << "ms\tfinished " << arm.config.str() << std::endl;
        main_scatter_mab.set_cached(i, ins->starts, s);
      });
    }
    for (auto &w : workers) w.join();

    arm_idx = main_scatter_mab.choose_arm();
    scatter = main_scatter_mab.get_cached(arm_idx, ins->starts);
  }

  info(1, verbose, deadline, "finish computing Scatter");
}

void Planner::set_pibt()
{
  for (auto k = 0; k < PIBT_NUM; ++k) {
    pibts.emplace_back(new PIBT(ins, D, k + seed, FLG_SWAP, scatter));
  }
}

void Planner::set_refiner()
{
  if (depth > 0) return;
  if (!FLG_REFINER) return;
  if (!FLG_MULTI_THREAD) return;
  auto plan = backtrack(H_goal);
  info(2, verbose, deadline, "invoke refiners");
  for (auto k = 0; k < REFINER_NUM; ++k) {
    ++seed_refiner;
    refiner_pool.emplace_back(
        std::async(std::launch::async, &Planner::get_refined_plan, this, plan));
  }
}

RefinedPlan Planner::get_refined_plan(const Solution &plan)
{
  auto MT_internal = std::mt19937(seed_refiner);
  if (depth < 1 && plan.size() > 3 &&
      get_random_float(MT_internal) < RECURSIVE_RATE) {
    // recursive LaCAM
    int chosen_arm_idx = refiners_scatter_mab.choose_arm();
    float path_ratio = refiners_scatter_mab.arms[chosen_arm_idx].get_path_ratio();
    int insert_idx = static_cast<int>(path_ratio * static_cast<float>(plan.size() - 1));
    insert_idx = std::max(1, std::min(insert_idx, static_cast<int>(plan.size()) - 2));
    auto ins_tmp = Instance(ins->G, plan[insert_idx], ins->goals, N);
    auto deadline_tmp = Deadline(std::min(
        RECURSIVE_TIME_LIMIT,
        deadline == nullptr ? INT_MAX
                            : deadline->time_limit_ms - elapsed_ms(deadline)));
    auto planner_tmp =
        Planner(&ins_tmp, 0, &deadline_tmp, seed_refiner, depth + 1, D);
    planner_tmp.arm_idx = chosen_arm_idx;
    refiners_scatter_mab.record_pull(chosen_arm_idx);
    info(4, verbose, deadline, "refiner-", planner_tmp.seed,
         "\tactivated (recursive LaCAM)");
    auto res = planner_tmp.solve();
    info(4, verbose, deadline, "refiner-", planner_tmp.seed,
         "\tcompleted (recursive LaCAM)");
    return {res, Refiner::RecursiveLaCAM, chosen_arm_idx};
  } else if (RECURSIVE_RATE < 1.0) {
    // iterative refinement
    return {refine(ins, deadline, plan, D, seed_refiner, verbose - 4), Refiner::SIPP, -1};
  } else {
    return {Solution(), Refiner::SIPP, -1};
  }
}

void Planner::update_checkpoints()
{
  const auto time = elapsed_ms(deadline);
  while (time >= checkpoints.size() * CHECKPOINTS_DURATION) {
    checkpoints.push_back(H_goal != nullptr ? H_goal->f : CHECKPOINTS_NIL);
  }
}

void Planner::logging()
{
  if (depth > 0) return;
  MSG += "checkpoints=";
  for (auto &k : checkpoints) MSG += std::to_string(k) + ",";
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
  info(1, verbose, deadline, "search iteration:", search_iter,
       "\texplored:", EXPLORED.size());
}
