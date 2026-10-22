#include "../include/horizon_pair_db.hpp"
#include "../include/horizon_pair_db_generator.hpp"

#include <unordered_set>


// --- Shared state definitions ---
thread_local std::vector<int> t_closed_g;
thread_local std::vector<uint32_t> t_closed_gen;
thread_local uint32_t t_current_gen = 0;


static DistTable* create_dist_table(Graph* G) {
  std::cout << "Creating the big DistTable" << std::endl;
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  std::cout << "Finished creating the big DistTable" << std::endl;
  return D;
}


static HorizonPairDB save_and_load_db(Graph* G, int horizon, int margin) {
  int orig_horizon = HorizonPairDBGenerator::HORIZON;
  int orig_margin = HorizonPairDBGenerator::MARGIN;
  int orig_mdd_size = HorizonPairDBGenerator::MDD_SIZE;
  HorizonPairDBGenerator::HORIZON = horizon;
  HorizonPairDBGenerator::MDD_SIZE = horizon + 1;
  HorizonPairDBGenerator::MARGIN = margin;
  HorizonPairDBGenerator DB(G, "test");
  DB.generate_and_save();
  HorizonPairDBGenerator::HORIZON = orig_horizon;
  HorizonPairDBGenerator::MDD_SIZE = orig_mdd_size;
  HorizonPairDBGenerator::MARGIN = orig_margin;
  
  HorizonPairDB loaded_db(G, "test");
  loaded_db.load_from_file();
  return loaded_db;
}


int joint_astar(DistTable* D, Vertex* i_start, Vertex* i_next, Vertex* i_goal, Vertex* j_start, Vertex* j_goal) {
  struct State {
    int g, f;
    Vertex* i;
    Vertex* j;
  };

  auto cmp = [](const State& a, const State& b) {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g;
  };
  std::priority_queue<State, std::vector<State>, decltype(cmp)> open(cmp);

  int V = D->K;

  if (t_closed_g.size() < (size_t)V * V) {
    t_closed_g.assign(V * V, INT_MAX);
    t_closed_gen.assign(V * V, 0);
  }

  if (++t_current_gen == 0) {
    std::fill(t_closed_gen.begin(), t_closed_gen.end(), 0);
    t_current_gen = 1;
  }

  auto encode = [&](int i, int j) { return i * V + j; };

  int h0 = D->get(i_goal->id, i_start->id) + D->get(j_goal->id, j_start->id);
  open.push({0, h0, i_start, j_start});

  while (!open.empty()) {
    auto [g, f, ci, cj] = open.top();
    open.pop();

    bool i_is_forced_to_move = i_next != nullptr && i_next != i_start && g == 0;
    if (ci == i_goal && cj == j_goal) {
      if (!i_is_forced_to_move) return g;
    }

    int ek = encode(ci->id, cj->id);

    if (t_closed_gen[ek] == t_current_gen && t_closed_g[ek] <= g) continue;

    if (!i_is_forced_to_move) {
      t_closed_gen[ek] = t_current_gen;
      t_closed_g[ek] = g;
    }

    bool i_at_goal = (ci == i_goal);
    bool j_at_goal = (cj == j_goal);

    int i_deg = ci->neighbor.size();
    int j_deg = cj->neighbor.size();

    Vertex* moves_i[5];
    Vertex* moves_j[5];

    if (i_next != nullptr && g == 0) {
      i_deg = 0;
      moves_i[0] = i_next;
    } else {
      for (int k = 0; k < i_deg; ++k) moves_i[k] = ci->neighbor[k];
      moves_i[i_deg] = ci;
    }

    for (int k = 0; k < j_deg; ++k) moves_j[k] = cj->neighbor[k];
    moves_j[j_deg] = cj;

    for (int i_idx = 0; i_idx <= i_deg; ++i_idx) {
      Vertex* ni = moves_i[i_idx];

      int h_i = D->get(i_goal->id, ni->id);
      int step_i = (i_at_goal && ni == i_goal) ? 0 : 1;

      for (int j_idx = 0; j_idx <= j_deg; ++j_idx) {
        Vertex* nj = moves_j[j_idx];

        if (ni == ci && nj == cj && !i_at_goal && !j_at_goal) continue;
        if (ni == nj) continue;
        if (ni == cj && nj == ci) continue;

        int step_j = (j_at_goal && nj == j_goal) ? 0 : 1;
        int ng = g + step_i + step_j;

        int nek = encode(ni->id, nj->id);

        if (t_closed_gen[nek] == t_current_gen && t_closed_g[nek] <= ng) continue;

        int nf = ng + h_i + D->get(j_goal->id, nj->id);
        open.push({ng, nf, ni, nj});
      }
    }
  }

  return INT_MAX;
}


static uint8_t get_db_penalty(HorizonPairDB* DB, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal) {
  uint32_t mdd_i = DB->mdd_id_by_v_g[i_start->id * DB->G->V.size() + i_goal->id];
  uint32_t mdd_j = DB->mdd_id_by_v_g[j_start->id * DB->G->V.size() + j_goal->id];
  uint8_t penalty = 0;
  for (auto& entry : DB->penalties[mdd_i]) {
    if (entry.mdd_id == mdd_j) {
      penalty = entry.penalty;
      break;
    }
  }
  return penalty;
}


static bool test_sync_time_conflicts(Graph* G, DistTable* D, HorizonPairDB* DB, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal) {
  int astar_cost = joint_astar(D, i_start, nullptr, i_goal, j_start, j_goal);
  int naive_cost = D->get(i_start->id, i_goal->id) + D->get(j_start->id, j_goal->id);
  int astar_penalty = std::min(5, astar_cost - naive_cost);
  int db_penalty = get_db_penalty(DB, i_start, i_goal, j_start, j_goal);
  if (astar_penalty != db_penalty) {
    std::cout << "[TEST FAILED]. DB penalty: " << db_penalty
              << ", A* penalty: " << astar_penalty
              << ", i: (" << i_start->x << ", " << i_start->y << ")->("
              << i_goal->x << ", " << i_goal->y << ")"
              << ", j: (" << j_start->x << ", " << j_start->y << ")->("
              << j_goal->x << ", " << j_goal->y << ")"
              << std::endl;
      return false;
  }
  return true;
}


void HorizonPairDB::integration_test1() {
  int HORIZON = 10;
  int MARGIN  = 10;
  std::cout << "Running integration_test1" << std::endl;
  std::vector<std::string> grid = {
    "...@.",  //  0   1   2   ..   3
    ".....",  //  4   5   6    7   8
    "@...@",  //  ..  9  10   11  ..
    "..@..",  //  12 13  ..   14  15
  };
  Graph* G = new Graph(grid);
  HorizonPairDB DB = save_and_load_db(G, HORIZON, MARGIN);
  // HorizonPairDB DB(G, "bla");
  DistTable* D = create_dist_table(G);


  // MDD MDD_i, MDD_j;
  // MDD_i.populate(D, G->U[3 * G->width + 4], G->U[3 * G->width + 3], 10, MARGIN);
  // MDD_j.populate(D, G->U[3 * G->width + 3], G->U[3 * G->width + 4], 10, MARGIN);
  // std::cout << "MDD_i: " << MDD_i.str() << std::endl;
  // std::cout << "MDD_j: " << MDD_j.str() << std::endl;

  // HorizonPairDBGenerator generator(G, "bla");
  // HorizonPairDBGenerator::MARGIN = MARGIN;
  // uint8_t penalty = generator.calculate_penalty(MDD_i, MDD_j);
  // std::cout << "Penalty: " << (int)penalty << std::endl;
  // return;


  std::cout << "Testing sync-time penalties" << std::endl;

  int successes = 0;
  int failures = 0;
  for (Vertex* i_start : G->V) {
    for (Vertex* i_goal : G->V) {
      for (Vertex* j_start : G->V) {
        if (j_start == i_start) continue;
        for (Vertex* j_goal : G->V) {
          if (j_goal == i_goal) continue;
          bool success = test_sync_time_conflicts(G, D, &DB, i_start, i_goal, j_start, j_goal);
          if (success) successes++; else failures++;
        }
      }
    }
  }
  std::cout << "Successes: " << successes << ", Failures: " << failures << std::endl;
  delete G;
}
