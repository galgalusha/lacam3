#include "../include/mdd.hpp"
#include "../include/drawing.hpp"

#include "absl/container/inlined_vector.h"
#include <sstream>
#include <string>
#include <unordered_map>


void MDD::populate(DistTable* D, Vertex* v_i, Vertex* v_g, int size) {
  frontiers.resize(size);

  frontiers[0].push_back(v_i);
  int D_t_minus_1 = D->get(v_i->id, v_g->id);

  int t;
  for (t = 1; t < size; t++) { 
    bool was_goal_inserted_for_this_depth = false;
    for (Vertex* v_t_minus_1 : frontiers[t - 1]) {
      for (Vertex* v_t : v_t_minus_1->neighbor) {
        // Verify optimal progression
        if (D->get(v_t->id, v_g->id) == D_t_minus_1 - 1) {
          // 3. Duplicate Prevention: Since K is tiny, layer sizes are small. 
          // A linear std::find is faster than allocating a std::unordered_set.
          if (std::find(frontiers[t].begin(), frontiers[t].end(), v_t) == frontiers[t].end()) {
            frontiers[t].push_back(v_t);
          }
        }
      }
    }
    D_t_minus_1--;
    if (D_t_minus_1 < 0) { break; }
  }
  // goal reached before horizon: pad remaining depths with the goal vertex
  for (int t_pad = t; t_pad < size; t_pad++) frontiers[t_pad].push_back(v_g);
}


std::string MDD::str() const {
  std::ostringstream oss;
  oss << "{ ";
  for (size_t t = 0; t < frontiers.size(); t++) {
    if (t > 0) oss << ", ";
    oss << t << ": [";
    for (size_t i = 0; i < frontiers[t].size(); i++) {
      if (i > 0) oss << ", ";
      oss << frontiers[t][i]->id;
    }
    oss << "]";
  }
  oss << " }";
  return oss.str();
}

void MDD::render(Instance* ins) {
  int agent_id = frontiers[0][0]->id; // not really an agent id, but its OK for coloring
  using namespace drawing_detail;
  const Graph* G = ins->G;
  const int W = G->width;
  const int H = G->height;

  // map each vertex index (U-index) to the smallest depth t at which it appears
  std::unordered_map<int, int> depth_of;
  for (int t = 0; t < (int)frontiers.size(); t++) {
    for (Vertex* v : frontiers[t]) {
      auto [it, inserted] = depth_of.emplace(v->index, t);
      (void)it;
      (void)inserted;
    }
  }

  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      int idx = W * y + x;
      auto it = depth_of.find(idx);
      if (it == depth_of.end()) {
        std::cout << (G->U[idx] ? '.' : '#');
      } else {
        std::cout << COLORS[agent_id % NUM_COLORS] << (it->second % 10) << RESET;
      }
    }
    std::cout << '\n';
  }
}


MDD MDD::get_mdd_with_wait() {
  MDD new_mdd;
  new_mdd.frontiers.resize(frontiers.size());
  new_mdd.frontiers[0] = frontiers[0];

  for (int t = 1; t < frontiers.size(); t++) {
    new_mdd.frontiers[t] = frontiers[t - 1];
  }
  return new_mdd;
}


MDD MDD::get_mdd_with_wait_at_time_1() {
  MDD new_mdd;
  new_mdd.frontiers.resize(frontiers.size());
  new_mdd.frontiers[0] = frontiers[0];
  new_mdd.frontiers[1] = frontiers[1];

  for (int t = 2; t < frontiers.size(); t++) {
    new_mdd.frontiers[t] = frontiers[t - 1];
  }
  return new_mdd;
}

bool MDD::check_joint_mdd_conflict(MDD& other_mdd, Graph* G, int horizon, int dt_me, int dt_other) {
    if (horizon + dt_me > frontiers.size()) {
      throw std::runtime_error("this->frontiers is too small for joint comparison. frontiers.size: " +
                               std::to_string(frontiers.size()));
    }
    if (horizon + dt_other > other_mdd.frontiers.size()) {
      throw std::runtime_error("other->frontiers is too small for joint comparison. frontiers.size: " +
                               std::to_string(other_mdd.frontiers.size()));
    }

    std::vector<Vertex*> u1_self_vector({ nullptr });
    std::vector<Vertex*> u2_self_vector({ nullptr });

    thread_local std::vector<std::pair<Vertex*, Vertex*>> current_layer;
    thread_local std::vector<std::pair<Vertex*, Vertex*>> next_layer;
    
    current_layer.clear();
    for (Vertex* v1 : frontiers[dt_me]) {
      for (Vertex* v2 : other_mdd.frontiers[dt_other]) {
        if (v1 != v2)
          current_layer.push_back({ v1, v2 });
      }
    }
    if (current_layer.size() == 0) return true;

    // Allocate once per thread, reuse endlessly
    thread_local std::vector<bool> in_f1;
    thread_local std::vector<bool> in_f2;
    // Assuming G->V.size() is known/accessible, otherwise resize on first use
    if (in_f1.empty()) {
        in_f1.resize(G->V.size(), false);
        in_f2.resize(G->V.size(), false);
    }

    bool is_1_waiting = false;
    bool is_2_waiting = false;

    for (int time = 1; time < horizon; ++time) {
        int t1 = time + dt_me;
        int t2 = time + dt_other;
        next_layer.clear();

        is_1_waiting = frontiers[t1].size() == 1 && frontiers[t1-1].size() == 1 && frontiers[t1][0] == frontiers[t1-1][0];
        is_2_waiting = other_mdd.frontiers[t2].size() == 1 && other_mdd.frontiers[t2-1].size() == 1 && other_mdd.frontiers[t2][0] == other_mdd.frontiers[t2-1][0];

        for (Vertex* v : frontiers[t1]) in_f1[v->id] = true;
        for (Vertex* v : other_mdd.frontiers[t2]) in_f2[v->id] = true;

        for (const auto& pair : current_layer) {
            Vertex* u1 = pair.first;
            Vertex* u2 = pair.second;

            std::vector<Vertex*>* u1_candidates = &u1->neighbor;
            std::vector<Vertex*>* u2_candidates = &u2->neighbor;

            if (is_1_waiting) { u1_self_vector[0] = u1; u1_candidates = &u1_self_vector; }
            if (is_2_waiting) { u2_self_vector[0] = u2; u2_candidates = &u2_self_vector; }

            for (Vertex* v1 : *u1_candidates) {
                if (!in_f1[v1->id]) continue;

                for (Vertex* v2 : *u2_candidates) {
                    if (!in_f2[v2->id]) continue;

                    if (v1 == v2) continue; // Vertex conflict
                    if (u1 == v2 && u2 == v1) continue; // Swap conflict

                    next_layer.push_back({v1, v2});
                }
            }
        }

        // Cleanup
        for (Vertex* v : frontiers[t1]) in_f1[v->id] = false;
        for (Vertex* v : other_mdd.frontiers[t2]) in_f2[v->id] = false;

        if (next_layer.empty()) return true; 

        // std::pair has a built-in operator< that compares .first then .second.
        // It is heavily optimized by the compiler.
        std::sort(next_layer.begin(), next_layer.end());
        next_layer.erase(std::unique(next_layer.begin(), next_layer.end()), next_layer.end());

        current_layer.swap(next_layer);
    }

    return false; 
}


static DistTable* create_dist_table(Graph* G) {
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}


void MDD::test_joint_mdd() {
  int HORIZON = 6;
  int MDD_SIZE = 8;
  std::cout << "Running test_joint_mdd" << std::endl;
  std::vector<std::string> grid = {
    "....",
    "....",
    "....",
    "....",
  };
  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(1, 0);
  auto A_goal  = coord(2, 3);

  auto B_start = coord(0, 1);
  auto B_goal  = coord(3, 2);

  MDD mdd_A;
  MDD mdd_B;

  mdd_A.populate(D, A_start, A_goal, MDD_SIZE);
  mdd_B.populate(D, B_start, B_goal, MDD_SIZE);

  bool actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_B, G, HORIZON);
  bool expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect conflict between A and B" << std::endl;
    exit(0);
  }

  auto C_start = coord(0, 2);
  auto C_goal  = coord(3, 2);

  MDD mdd_C;
  mdd_C.populate(D, C_start, C_goal, MDD_SIZE);

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_C, G, HORIZON);
  expected_conflict = false;

  if (actual_conflict != expected_conflict) {
    std::cout << "Found non existing conflict between A and C" << std::endl;
    exit(0);
  }

  std::cout << "All MDD tests passed" << std::endl;

  delete G;
  delete D;
}


void MDD::test_joint_mdd2() {
  int HORIZON = 6;
  int MDD_SIZE = 8;
  std::cout << "Running test_joint_mdd2" << std::endl;
  std::vector<std::string> grid = {
    ".@.",
    "...",
    "...",
  };
  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(0, 0);
  auto A_goal  = coord(1, 2);

  auto B_start = coord(1, 0);
  auto B_goal  = coord(0, 2);

  auto C_start = coord(1, 0);
  auto C_goal  = coord(1, 1);

  auto D_start = A_goal; // swap conflict with A
  auto D_goal  = A_start;

  auto E_start = coord(1, 1);
  auto E_goal = E_start;

  MDD mdd_A;
  MDD mdd_B;
  MDD mdd_C;
  MDD mdd_D;
  MDD mdd_E;

  mdd_A.populate(D, A_start, A_goal, MDD_SIZE);
  mdd_B.populate(D, B_start, B_goal, MDD_SIZE);
  mdd_C.populate(D, C_start, C_goal, MDD_SIZE);
  mdd_D.populate(D, D_start, D_goal, MDD_SIZE);
  mdd_E.populate(D, E_start, E_goal, MDD_SIZE);

  bool actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_B, G, HORIZON);
  bool expected_conflict = false;

  if (actual_conflict != expected_conflict) {
    std::cout << "Detected non existing conflict between A and B" << std::endl;
    exit(0);
  }

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_C, G, HORIZON);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect a conflict between A and C" << std::endl;
    exit(0);
  }

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_D, G, HORIZON);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect a conflict between A and D" << std::endl;
    exit(0);
  }

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_E, G, HORIZON);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect a conflict between A and E" << std::endl;
    exit(0);
  }

  std::cout << "All MDD tests passed" << std::endl;

  delete G;
  delete D;
}

void MDD::test_joint_mdd_with_wait() {
  int HORIZON = 6;
  int MDD_SIZE = 8;
  std::cout << "Running test_joint_mdd_with_wait" << std::endl;
  std::vector<std::string> grid = {
    "....",
    "....",
    "....",
    "....",
  };
  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(1, 0);
  auto A_goal  = coord(2, 3);

  auto B_start = coord(0, 1);
  auto B_goal  = coord(3, 2);

  MDD mdd_A;
  MDD mdd_B;

  mdd_A.populate(D, A_start, A_goal, MDD_SIZE);
  mdd_B.populate(D, B_start, B_goal, MDD_SIZE);

  bool actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_B, G, HORIZON);
  bool expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect conflict between A and B" << std::endl;
    exit(0);
  }

  auto B_next = coord(1, 1);

  MDD mdd_B_next;
  mdd_B_next.populate(D, B_next, B_goal, MDD_SIZE);

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_B_next, G, HORIZON, 1, 0);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect conflict between A and B_next" << std::endl;
    exit(0);
  }

  std::cout << "All MDD tests passed" << std::endl;

  delete G;
  delete D;
}

void MDD::test_joint_mdd_with_wait2() {
  int HORIZON = 6;
  int MDD_SIZE = 8;
  std::cout << "Running test_joint_mdd_with_wait2" << std::endl;
  std::vector<std::string> grid = {
    ".@.",
    "...",
    "...",
  };
  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(0, 0);
  auto A_goal  = coord(1, 2);

  auto B_start = coord(1, 0);
  auto B_goal  = coord(0, 2);

  auto C_start = coord(1, 0);
  auto C_goal  = coord(1, 1);

  auto D_start = A_goal; // swap conflict with A
  auto D_goal  = A_start;

  auto E_start = coord(1, 1);
  auto E_goal = E_start;

  MDD mdd_A;
  MDD mdd_B;
  MDD mdd_C;
  MDD mdd_D;
  MDD mdd_E;

  mdd_A.populate(D, A_start, A_goal, MDD_SIZE);
  mdd_B.populate(D, B_start, B_goal, MDD_SIZE); mdd_B = mdd_B.get_mdd_with_wait();
  mdd_C.populate(D, C_start, C_goal, MDD_SIZE); mdd_C = mdd_C.get_mdd_with_wait();
  mdd_D.populate(D, D_start, D_goal, MDD_SIZE); mdd_D = mdd_D.get_mdd_with_wait();
  mdd_E.populate(D, E_start, E_goal, MDD_SIZE); mdd_E = mdd_E.get_mdd_with_wait();

  bool actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_B, G, HORIZON);
  bool expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect conflict between A and B" << std::endl;
    exit(0);
  }

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_C, G, HORIZON);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect a conflict between A and C" << std::endl;
    exit(0);
  }

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_D, G, HORIZON);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect a conflict between A and D" << std::endl;
    exit(0);
  }

  actual_conflict = mdd_A.check_joint_mdd_conflict(mdd_E, G, HORIZON);
  expected_conflict = true;

  if (actual_conflict != expected_conflict) {
    std::cout << "Failed to detect a conflict between A and E" << std::endl;
    exit(0);
  }

  std::cout << "All MDD tests passed" << std::endl;

  delete G;
  delete D;
}
