#include "../include/scatter_mab_planner.hpp"
#include "../include/wait_scatter.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>

std::string ScatterMABPlanner::MSG;

constexpr auto TIME_ZERO = std::chrono::seconds(0);

ScatterMABPlanner::ScatterMABPlanner(const Instance *_ins, int _verbose, const Deadline *_deadline,
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
      is_exploring(true),
      arms(),
      exploration_thread_pool(EXPLORATION_THREADS),
      scatter(nullptr),
      seed_refiner(0),
      refiner_pool(),
      OPEN(),
      EXPLORED(),
      H_init(nullptr),
      H_goal(nullptr),
      search_iter(0),
      time_initial_solution(-1),
      cost_initial_solution(-1)
{
}

ScatterMABPlanner::~ScatterMABPlanner()
{
  if (heuristic != nullptr) delete heuristic;
  if (scatter != nullptr) delete scatter;
  for (auto *p : pibts) delete p;
  for (auto &arm : arms) delete arm.scatter;
  if (delete_dist_table_after_used) delete D;
}

Solution ScatterMABPlanner::solve()
{
  info(1, verbose, deadline, "start search");
  LB = get_sum_of_costs_lower_bound(*ins, *D);
  info(1, verbose, deadline, "LB=", LB);

  if (is_exploring) {
    explore_scatters();
    return Solution{};
  }

  // insert initial node
  H_init = create_highlevel_node(ins->starts, nullptr);
  OPEN.push_front(H_init);

  set_scatter();
  set_pibt();

  // search loop
  while (!OPEN.empty() && !is_expired(deadline)) {
    search_iter += 1;

    // check pooled procedures
    refiner_pool.remove_if([&](auto &proc) {
      if ((proc).wait_for(TIME_ZERO) != std::future_status::ready) return false;
      apply_new_solution(proc.get());
      ++seed_refiner;
      refiner_pool.emplace_back(std::async(std::launch::async,
                                           &ScatterMABPlanner::get_refined_plan, this,
                                           backtrack(H_goal)));
      return true;
    });

    // do not pop here!
    auto H = OPEN.front();

    // random insert after initial solution found
    if (H_goal != nullptr && get_random_float(MT) < Params::RANDOM_INSERT_PROB2) {
      H = Params::FLG_RANDOM_INSERT_INIT_NODE
              ? H_init
              : OPEN[get_random_int(MT, 0, OPEN.size() - 1)];
    }

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop_front();
      continue;
    }

    // check goal condition
    if (H_goal == nullptr && is_same_config(H->C, ins->goals)) {
      time_initial_solution = elapsed_ms(deadline);
      cost_initial_solution = H->g;
      H_goal = H;
      info(1, verbose, deadline, "found initial solution, cost: ", H_goal->g);
      set_refiner();         // refining start
      continue;
    }

    // low level search
    LNode* L = H->get_next_lowlevel_node(MT);
    H->ll_count++;


    if (L == nullptr) {
      // if (H->ll_count > 100) OPEN.pop_front();
      continue;
    }

    // create successors at the high-level search
    auto Q_to = Config(N, nullptr);
    auto res = set_new_config(H, L, Q_to);
    delete L;
    if (!res) {
      continue;
    }

    auto g_val = H->g + get_edge_cost(H->C, Q_to);
    auto h_val = heuristic->get(Q_to);
    auto f_val = g_val + h_val;

    // check explored list
    auto iter = EXPLORED.find(Q_to);
    if (iter != EXPLORED.end()) {
      rewrite(H, iter->second);

      if (get_random_float(MT) >= Params::RANDOM_INSERT_PROB1) {
        OPEN.push_front(iter->second);  // usual
      } else {
        OPEN.push_front(H_init);  // sometimes
      }
    } else {
      // prune check
      if (H_goal != nullptr) {
        if (f_val >= H_goal->g) {
          continue;
        }
      }
      auto H_new = create_highlevel_node(Q_to, H);
      OPEN.push_front(H_new);
    }
  }

  // clear pooled operaitons
  bool is_optimal = OPEN.empty();
  for (auto &proc : refiner_pool) apply_new_solution(proc.get());
  if (is_optimal) OPEN.clear();

  // end processing
  logging();
  auto solution = backtrack(H_goal);        // obtain solution
  for (auto p : EXPLORED) delete p.second;  // memory management
  return solution;
}

static int get_next_arm(const std::vector<Arm> &arms, int total_runs, int min_cost, int p90_cost)
{
  const int n = static_cast<int>(arms.size());

  // warm-up: round-robin until every arm has at least 5 costs
  bool warming_up = std::any_of(arms.begin(), arms.end(),
                                [](const Arm &a) { return a.costs.size() < 3; });
  if (warming_up) {
    // pick arm with fewest runs so far
    int best = 0;
    for (int i = 1; i < n; ++i)
      if (arms[i].num_of_runs < arms[best].num_of_runs) best = i;
    return best;
  }

  int best = -1;
  double best_score = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < n; ++k) {
    double s = arms[k].get_ucb1_score(total_runs, min_cost, p90_cost);
    if (s > best_score) { best_score = s; best = k; }
  }
  return best;
}

static std::pair<int, int> get_min_and_p90_costs(const std::vector<Arm> &arms)
{
  std::vector<int> all_costs;
  for (auto &arm : arms)
    for (int c : arm.costs) all_costs.push_back(c);
  if (all_costs.empty()) return {0, 0};
  std::sort(all_costs.begin(), all_costs.end());
  int min_cost = all_costs.front();
  int p90_idx = static_cast<int>(std::ceil(0.9 * all_costs.size())) - 1;
  p90_idx = std::max(0, std::min(p90_idx, static_cast<int>(all_costs.size()) - 1));
  return {min_cost, all_costs[p90_idx]};
}

void ScatterMABPlanner::explore_scatters()
{
  const std::vector<int> margins = {5, 10, 20, 40};
  const int SCATTER_NUM = static_cast<int>(margins.size());

  // construct scatters in parallel
  auto scatter_deadline_ms = deadline->time_limit_ms / 3;
  std::vector<std::future<IScatter *>> scatter_futures;
  for (int k = 0; k < SCATTER_NUM; ++k) {
    scatter_futures.push_back(exploration_thread_pool.submit([&, k]() -> IScatter * {
      auto sd = Deadline(scatter_deadline_ms);
      auto *s = new WaitScatter(ins, D, &sd, 3, verbose - 4, margins[k]);
      s->construct(5);
      info(1, verbose, deadline, "Scatter", k, "created");
      return s;
    }));
  }

  info(1, verbose, deadline, "Starting exploration");

  arms.reserve(SCATTER_NUM);
  for (int k = 0; k < SCATTER_NUM; ++k) {
    Arm arm;
    arm.id = k;
    arm.scatter = scatter_futures[k].get();
    arms.push_back(std::move(arm));
  }

  std::mutex result_mutex;
  int total_runs = 0;
  int global_best_cost = INT_MAX;

  // generate per-worker seeds deterministically
  std::vector<uint32_t> worker_seeds(EXPLORATION_THREADS);
  for (auto &s : worker_seeds) s = MT();

  auto worker = [&](uint32_t worker_seed) {
    std::mt19937 local_mt(worker_seed);
    while (!is_expired(deadline)) {
      // select arm under lock
      int arm_id;
      {
        std::lock_guard<std::mutex> lock(result_mutex);
        auto [min_cost, p90_cost] = get_min_and_p90_costs(arms);
        arm_id = get_next_arm(arms, total_runs, min_cost, p90_cost);
      }

      EpochResult result = run_epoch(arm_id, arms[arm_id].scatter, local_mt);

      {
        std::lock_guard<std::mutex> lock(result_mutex);
        ++total_runs;
        ++arms[arm_id].num_of_runs;
        if (result.success) {
          arms[arm_id].costs.push_back(result.cost);
          if (result.cost < global_best_cost) {
            global_best_cost = result.cost;
            info(1, verbose, deadline, "best cost update: ", global_best_cost);
          }
        }
      }
    }
  };

  std::vector<std::future<void>> futures;
  futures.reserve(EXPLORATION_THREADS);
  for (int i = 0; i < EXPLORATION_THREADS; ++i)
    futures.emplace_back(std::async(std::launch::async, worker, worker_seeds[i]));
  for (auto &f : futures) f.wait();

  info(1, verbose, deadline, "explore_scatters done, best cost: ", global_best_cost,
       ", total runs: ", total_runs);

  for (auto &arm : arms) {
    if (arm.costs.empty()) {
      std::cout << "[Arm " << arm.id << "] no successful runs" << std::endl;
      continue;
    }
    auto sorted = arm.costs;
    std::sort(sorted.begin(), sorted.end());
    auto percentile = [&](double p) {
      int idx = static_cast<int>(std::ceil(p * sorted.size())) - 1;
      return sorted[std::max(0, std::min(idx, static_cast<int>(sorted.size()) - 1))];
    };
    double avg = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
    std::cout << "[Arm " << arm.id << "] runs=" << arm.num_of_runs
              << " successes=" << sorted.size()
              << " avg=" << avg
              << " min=" << sorted.front()
              << " p10=" << percentile(0.1)
              << " p20=" << percentile(0.2)
              << " p50=" << percentile(0.5)
              << std::endl;
  }

  auto [gmin, gp90] = get_min_and_p90_costs(arms);
  (void)gmin; (void)gp90;
  std::vector<int> all_costs_sorted;
  for (auto &arm : arms)
    for (int c : arm.costs) all_costs_sorted.push_back(c);
  if (!all_costs_sorted.empty()) {
    std::sort(all_costs_sorted.begin(), all_costs_sorted.end());
    auto gpercentile = [&](double p) {
      int idx = static_cast<int>(std::ceil(p * all_costs_sorted.size())) - 1;
      return all_costs_sorted[std::max(0, std::min(idx, static_cast<int>(all_costs_sorted.size()) - 1))];
    };
    int gp10 = gpercentile(0.1);
    int gp20 = gpercentile(0.2);
    std::cout << "Global p10=" << gp10 << " p20=" << gp20 << std::endl;
    for (auto &arm : arms) {
      int in_p10 = static_cast<int>(std::count_if(arm.costs.begin(), arm.costs.end(), [gp10](int c) { return c <= gp10; }));
      int in_p20 = static_cast<int>(std::count_if(arm.costs.begin(), arm.costs.end(), [gp20](int c) { return c <= gp20; }));
      std::cout << "[Arm " << arm.id << "] in_global_p10=" << in_p10 << " in_global_p20=" << in_p20 << std::endl;
    }
  }
}

EpochResult ScatterMABPlanner::run_epoch(int arm_id, IScatter *scatter, std::mt19937 &local_mt)
{
  // thread-local search state
  std::deque<HNode *> OPEN;
  std::unordered_map<Config, HNode *, ConfigHasher> EXPLORED;

  auto init_h = heuristic->get(ins->starts);
  auto *H_init = new HNode(ins->starts, D, nullptr, 0, init_h);
  EXPLORED[ins->starts] = H_init;
  OPEN.push_back(H_init);

  // PIBT is not threadsafe; construct one per epoch with a fresh seed from local_mt
  PIBT local_pibt(ins, D, static_cast<int>(local_mt()), Params::FLG_SWAP, scatter);

  int failure_budget = 50;
  int livelock_budget = 50;

  while (!OPEN.empty()) {
    auto *H = OPEN.front();

    if (is_same_config(H->C, ins->goals)) {
      int cost = H->g;
      for (auto &p : EXPLORED) delete p.second;
      std::cout << "[Arm " << arm_id << "] reached goal. Cost: " << cost << std::endl;
      return {true, cost};
    }

    LNode *L = H->get_next_lowlevel_node(local_mt);
    if (L == nullptr) {
      OPEN.pop_front();
      continue;
    }

    auto Q_to = Config(N, nullptr);
    for (auto d = 0; d < L->depth; ++d) Q_to[L->who[d]] = L->where[d];
    bool ok = local_pibt.set_new_config(H->depth, H->C, Q_to, H->order);
    delete L;

    if (!ok) {
      if (--failure_budget <= 0) {
        for (auto &p : EXPLORED) delete p.second;
        std::cout << "[Arm " << arm_id << "] epoch failed for PIBT deadlock" << std::endl;
        return {false, 0};
      }
      continue;
    }

    auto g_val = H->g + get_edge_cost(H->C, Q_to);
    auto h_val = heuristic->get(Q_to);
    auto f_val = g_val + h_val;

    auto iter = EXPLORED.find(Q_to);
    if (iter != EXPLORED.end()) {
      if (--livelock_budget <= 0) {
        for (auto &p : EXPLORED) delete p.second;
        std::cout << "[Arm " << arm_id << "] epoch failed for PIBT livelock" << std::endl;
        return {false, 0};
      }
      HNode *next_H = iter->second;
      if (f_val < next_H->f) {
        next_H->g = g_val;
        next_H->h = h_val;
        next_H->f = f_val;
        next_H->parent = H;
        next_H->depth = next_H->parent == nullptr ? 0 : next_H->parent->depth + 1;
      }
      OPEN.push_front(next_H);
    } else {
      HNode *next_H = new HNode(Q_to, D, H, g_val, h_val);
      EXPLORED[Q_to] = next_H;
      OPEN.push_front(next_H);
    }
  }

  for (auto &p : EXPLORED) delete p.second;
  std::cout << "[Arm " << arm_id << "] epoch failed (empty OPEN)" << std::endl;
  return {false, 0};
}

HNode *ScatterMABPlanner::create_highlevel_node(const Config &Q, HNode *parent)
{
  auto g_val =
      (parent == nullptr) ? 0 : parent->g + get_edge_cost(parent->C, Q);
  auto h_val = heuristic->get(Q);
  auto H_new = new HNode(Q, D, parent, g_val, h_val);
  EXPLORED[Q] = H_new;
  return H_new;
}

void ScatterMABPlanner::apply_new_solution(const Solution &plan)
{
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
      rewrite(H_from, H_to);
    } else {
      // new
      auto g_val = H_from->g + get_edge_cost(H_from->C, Q);
      H_to = new HNode(Q, D, H_from, g_val, heuristic->get(Q));
      EXPLORED[Q] = H_to;
      OPEN.push_front(H_to);
    }
    H_from = H_to;
  }
}

Solution ScatterMABPlanner::backtrack(HNode *H)
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

bool ScatterMABPlanner::set_new_config(HNode *H, LNode *L, Config &Q_to)
{
  // worker-id, time -> configuration
  auto Q_cands = std::vector<Config>(Params::PIBT_NUM, Config(N, nullptr));
  auto f_vals = std::vector<int>(Params::PIBT_NUM, INT_MAX);

  // parallel
  auto worker = [&](int k) {
    // set constraints
    for (auto d = 0; d < L->depth; ++d) Q_cands[k][L->who[d]] = L->where[d];
    // PIBT
    auto res = pibts[k]->set_new_config(H->depth, H->C, Q_cands[k], H->order);
    if (res)
      f_vals[k] = get_edge_cost(H->C, Q_cands[k]) + heuristic->get(Q_cands[k]);
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

void ScatterMABPlanner::rewrite(HNode *H_from, HNode *H_to)
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
        if (n_to == H_goal)
          info(2, verbose, deadline, "cost update: ", H_goal->g, " -> ", g_val);
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        n_to->depth = n_to->parent == nullptr ? 0 : n_to->parent->depth + 1;
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push_front(n_to);
      }
    }
  }
}

int ScatterMABPlanner::get_edge_cost(const Config &C1, const Config &C2)
{
  auto cost = 0;
  for (uint i = 0; i < N; ++i) {
    if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
      cost += 1;
    }
  }
  return cost;
}

void ScatterMABPlanner::set_scatter()
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
  info(1, verbose, deadline, "finish computing SUO");
}

void ScatterMABPlanner::set_pibt()
{
  for (auto k = 0; k < Params::PIBT_NUM; ++k) {
    pibts.emplace_back(new PIBT(ins, D, k + seed, Params::FLG_SWAP, scatter));
  }
}

void ScatterMABPlanner::set_refiner()
{
  if (!Params::FLG_REFINER) return;
  if (!Params::FLG_MULTI_THREAD) return;
  auto plan = backtrack(H_goal);
  info(2, verbose, deadline, "invoke refiners");
  for (auto k = 0; k < Params::REFINER_NUM; ++k) {
    ++seed_refiner;
    refiner_pool.emplace_back(
        std::async(std::launch::async, &ScatterMABPlanner::get_refined_plan, this, plan));
  }
}

Solution ScatterMABPlanner::get_refined_plan(const Solution &plan)
{
  auto MT_internal = std::mt19937(seed_refiner);
  if (depth < 1 && plan.size() > 3 &&
      get_random_float(MT_internal) < Params::RECURSIVE_RATE) {
    // recursive LaCAM
    auto ins_tmp =
        Instance(ins->G, plan[get_random_int(MT_internal, 1, plan.size() - 2)],
                 ins->goals, N);
    auto deadline_tmp = Deadline(std::min(
        Params::RECURSIVE_TIME_LIMIT,
        deadline == nullptr ? INT_MAX
                            : deadline->time_limit_ms - elapsed_ms(deadline)));
    auto planner_tmp =
        ScatterMABPlanner(&ins_tmp, 0, &deadline_tmp, seed_refiner, depth + 1, D);
    info(4, verbose, deadline, "refiner-", planner_tmp.seed,
         "\tactivated (recursive LaCAM)");
    auto res = planner_tmp.solve();
    info(4, verbose, deadline, "refiner-", planner_tmp.seed,
         "\tcompleted (recursive LaCAM)");
    return res;
  } else if (Params::RECURSIVE_RATE < 0.99) {
    // iterative refinement
    return refine(ins, deadline, plan, D, seed_refiner, verbose - 4);
  } else {
    return Solution();
  }
}

void ScatterMABPlanner::logging()
{
  if (depth > 0) return;
  MSG +=
      "\ncomp_time_initial_solution=" + std::to_string(time_initial_solution);
  MSG += "\ncost_initial_solution=" + std::to_string(cost_initial_solution);
  MSG += "\nsearch_iteration=" + std::to_string(search_iter);
  MSG += "\nnum_high_level_node=" + std::to_string(HNode::COUNT);
  MSG += "\nnum_low_level_node=" + std::to_string(LNode::COUNT);

  int pibt_func_count = 0;
  for (auto& p: pibts) pibt_func_count += p->func_pibt_counter;

  if (H_goal != nullptr && OPEN.empty()) {
    info(1, verbose, deadline, "solved optimally, cost:", H_goal->g);
  } else if (H_goal != nullptr) {
    info(1, verbose, deadline, "solved sub-optimally, cost:", H_goal->g);
  } else if (OPEN.empty()) {
    info(1, verbose, deadline, "no solution");
  } else {
    info(1, verbose, deadline, "timeout");
  }
  info(1, verbose, deadline, "search iteration:", search_iter,
       "\texplored:", EXPLORED.size());
  info(1, verbose, deadline, "search npibt_func_count:", pibt_func_count/1000, "k");

}
