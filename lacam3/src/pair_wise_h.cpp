#include "../include/pair_wise_h.hpp"
#include"../include/dist_table.hpp"

#include "../include/pair_wise_db.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <queue>
#include <unordered_map>

constexpr int NUM_OF_THREADS = 4;

PairWiseHeuristic::PairWiseHeuristic(Graph* _G) : G(_G) {}

DistTable* create_dist_table(Graph* G) {
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}

int astar(DistTable* D, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal) {
  // Returns minimum Sum-of-Costs for two agents to reach their goals conflict-free.
  // Cost stops accumulating for an agent once it reaches its goal, but it remains
  // an obstacle. If it must vacate its goal, extra steps are counted until it returns.

  struct State {
    int g, f;
    Vertex* i;
    Vertex* j;
  };

  auto cmp = [](const State& a, const State& b) {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g; // tie-break: prefer higher g
  };
  std::priority_queue<State, std::vector<State>, decltype(cmp)> open(cmp);

  // closed keyed on (i_id, j_id)
  std::unordered_map<long long, int> closed;
  auto encode = [&](int i, int j) { return (long long)i * D->K + j; };

  auto h = [&](Vertex* i, Vertex* j) {
    return D->get(i_goal->id, i->id) + D->get(j_goal->id, j->id);
  };

  int h0 = h(i_start, j_start);
  open.push({0, h0, i_start, j_start});

  while (!open.empty()) {
    auto [g, f, ci, cj] = open.top();
    open.pop();

    if (ci == i_goal && cj == j_goal) return g;

    int ek = encode(ci->id, cj->id);
    if (closed.count(ek) && closed[ek] <= g) continue;
    closed[ek] = g;

    // Build candidate moves: neighbors + wait for each agent
    auto moves_i = ci->neighbor;
    moves_i.push_back(ci); // wait
    auto moves_j = cj->neighbor;
    moves_j.push_back(cj); // wait

    for (Vertex* ni : moves_i) {
      for (Vertex* nj : moves_j) {
        // Skip "both wait" only when neither is at goal (pure waste)
        bool i_at_goal = (ci == i_goal);
        bool j_at_goal = (cj == j_goal);
        if (ni == ci && nj == cj && !i_at_goal && !j_at_goal) continue;

        // Vertex conflict
        if (ni == nj) continue;

        // Swap conflict
        if (ni == cj && nj == ci) continue;

        // Cost: each agent contributes 1 unless already at goal and staying
        int step_i = (i_at_goal && ni == i_goal) ? 0 : 1;
        int step_j = (j_at_goal && nj == j_goal) ? 0 : 1;
        int ng = g + step_i + step_j;

        int nek = encode(ni->id, nj->id);
        if (closed.count(nek) && closed[nek] <= ng) continue;

        open.push({ng, ng + h(ni, nj), ni, nj});
      }
    }
  }

  return INT_MAX; // no solution (shouldn't happen on connected graphs)
}

bool PairWiseHeuristic::can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal) {
  // D->get(target, source) gives distance from source to target

  // 1. Pairwise start/goal distances
  int dist_s_ij  = D->get(j_start, i_start); // dist from i_start to j_start
  int dist_g_ij  = D->get(j_goal,  i_goal);  // dist from i_goal  to j_goal

  // 2. Individual shortest path lengths
  int dist_sg_ii = D->get(i_goal,  i_start); // dist from i_start to i_goal
  int dist_sg_jj = D->get(j_goal,  j_start); // dist from j_start to j_goal

  // 3. Evaluate the Psi constraint
  int psi_ij = dist_s_ij + dist_g_ij - dist_sg_ii - dist_sg_jj;
  if (psi_ij > 0) {
    return false;
  }

  // 4. Cross-distances for the Lambda constraint
  int dist_sg_ji = D->get(i_goal,  j_start); // dist from j_start to i_goal
  int dist_sg_ij = D->get(j_goal,  i_start); // dist from i_start to j_goal

  // 5. Evaluate the Lambda constraint
  int lambda_ij = dist_sg_ii - dist_sg_ji;
  int lambda_ji = dist_sg_jj - dist_sg_ij;

  if (lambda_ij + lambda_ji < 0) {
    return false;
  }

  return true;
}

void PairWiseHeuristic::construct() { 
  // 1. Create the APSP (All-Pairs Shortest Path) distance table
  Config all_vertices = G->V;
  std::cout << "Creating dist table... " << std::flush;
  DistTable* D = create_dist_table(G);
  std::cout << "Done" << std::endl;

  long long count_evaluated = 0;
  long long count_interfering = 0;
  const int num_vertices = G->V.size();

  // 2. Iterate over all valid 4-tuples (i_start, i_goal, j_start, j_goal)
  long long total_outer = (long long)num_vertices * (num_vertices - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  for (int i_g = 0; i_g < num_vertices; ++i_g) {
    for (int j_g = i_g + 1; j_g < num_vertices; ++j_g, ++outer_idx) {
      int pct = (int)(outer_idx * 100000 / total_outer); // units of 0.001%
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000; // 50-char bar over 100000 units
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }

      std::string tmp_path = "./db/tmp_" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = "./db/" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories("./db");

      std::vector<PairEntry> pair_entries;
      auto last_flush = std::chrono::steady_clock::now();

      for (int i_s = 0; i_s < num_vertices; ++i_s) {
        if (i_s == i_g) continue;

        for (int j_s = 0; j_s < num_vertices; ++j_s) {
          if (j_s == j_g) continue;
          if (i_s == j_s) continue;

          count_evaluated++;

          // 3. Evaluate the geometric constraints
          if (can_interfere(D, i_s, i_g, j_s, j_g)) {
            count_interfering++;

            Vertex* vi_s = G->V[i_s];
            Vertex* vi_g = G->V[i_g];
            Vertex* vj_s = G->V[j_s];
            Vertex* vj_g = G->V[j_g];

            int independent_cost = D->get(i_g, i_s) + D->get(j_g, j_s);
            int joint_cost = astar(D, vi_s, vi_g, vj_s, vj_g);
            int d_h = joint_cost - independent_cost;

            pair_entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)d_h});
          }

          // Periodic flush every 5 seconds
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(now - last_flush).count() >= 5) {
            append_entries(tmp_path, pair_entries);
            pair_entries.clear();
            last_flush = now;
          }
        }
      }

      // Final flush and promote tmp file to permanent
      append_entries(tmp_path, pair_entries);
      std::filesystem::rename(tmp_path, final_path);
    }
  }

  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
  std::cout << "Total 4-tuples evaluated: " << count_evaluated << std::endl;
  std::cout << "Pairs flagged for A*:     " << count_interfering << std::endl;

  // Cleanup to prevent memory leaks during offline pre-processing
  delete D; 
}

void PairWiseHeuristic::test() {

  std::vector<std::string> grid = {
    "....",
    "....",
    "....",
  };
  Graph* G = new Graph(grid);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  Config starts = { coord(0, 0) , coord(2, 0) };
  Config goals  = { coord(0, 3) , coord(2, 3) };

  Instance* ins = new Instance(G, starts, goals, starts.size());
  auto D = new DistTable(*ins);
  bool can_interfere = PairWiseHeuristic::can_interfere(
    D,
    starts[0]->id, goals[0]->id,
    starts[1]->id, goals[1]->id
  );
  std::cout << "[Test 1] can interfere: " << (can_interfere ? "yes" : "no") << std::endl;
}
