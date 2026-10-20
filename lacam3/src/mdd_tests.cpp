#include "../include/mdd.hpp"
#include "../include/drawing.hpp"

#include "absl/container/inlined_vector.h"
#include <sstream>
#include <string>
#include <unordered_map>


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

  delete G;
  delete D;
}


void MDD::test_mdd_with_margin() {
  std::cout << "Running test_mdd_with_margin" << std::endl;
  std::vector<std::string> grid = {
    ".....", // A B C D E
    ".....", // F G H I J
    ".....", // K L M N O
    ".....", // P Q R S T
    ".....", // U V W X Y
  };

  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  // Map each cell's letter (per the grid comment above) to its Vertex, and
  // back again from a vertex id to its letter, e.g. V['A'] -> coord(0, 0).
  std::unordered_map<char, Vertex*> V;
  std::unordered_map<int, char> L;
  char letter = 'A';
  for (int row = 0; row < (int)grid.size(); ++row) {
    for (int col = 0; col < (int)grid[row].size(); ++col) {
      Vertex* v = coord(row, col);
      V[letter] = v;
      L[v->id] = letter;
      ++letter;
    }
  }

  MDD mdd;
  int horizon = 3;
  int margin = 2;
  mdd.populate(D, V['S'], V['G'], horizon + 1, margin);



  MDD expected_mdd;
  expected_mdd.frontiers.resize(4);
  expected_mdd.frontiers[0] = { V['S'] };
  expected_mdd.frontiers[1] = { V['R'], V['T'], V['N'], V['X'], V['S'] };
  expected_mdd.frontiers[2] = { V['Q'], V['S'], V['M'], V['W'], V['R'], V['O'], V['I'], V['N'] };
  expected_mdd.frontiers[3] = { V['P'], V['R'], V['L'], V['V'], V['Q'], V['N'], V['H'], V['M'], V['J'], V['D'], V['I'] };

  std::string actual_str = mdd.str();
  std::string expected_str = expected_mdd.str();
  if (actual_str != expected_str) {
    std::cout << "Expected MDD: " << expected_str << std::endl;
    std::cout << "Actual MDD  : " << actual_str << std::endl;
    exit(0);
  }

  // for (int t = 0; t < mdd.frontiers.size(); t++) {
  //   for (Vertex* v : mdd.frontiers[t]) {
  //     std::cout << " " << L[v->id];
  //   }
  //   std::cout << std::endl;
  // }
  delete G;
  delete D;
}

