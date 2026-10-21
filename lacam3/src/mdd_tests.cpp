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
    ".....", // A B C D E  //   0   1   2   3   4
    ".....", // F G H I J  //   5   6   7   8   9
    ".....", // K L M N O  //  10  11  12  13  14
    ".....", // P Q R S T  //  15  16  17  18  19
    ".....", // U V W X Y  //  20  21  22  23  24
  };

  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);

  MDD mdd;
  int horizon = 3;
  int margin = 2;
  mdd.populate(D, G->V[18], G->V[6], horizon + 1, margin);

  std::string actual_str = mdd.str();
  std::string expected_str = "{ 0: [18], 1: [17, 19, 13, 23, 18], 2: [16, 18, 12, 22, 17, 14, 8, 13], 3: [15, 17, 11, 21, 16, 13, 7, 12, 9, 3, 8] }";
  if (actual_str != expected_str) {
    std::cout << "Expected MDD: " << expected_str << std::endl;
    std::cout << "Actual MDD  : " << actual_str << std::endl;
    exit(0);
  }

  mdd.frontiers.clear();
  mdd.populate(D, G->V[18], G->V[5], horizon + 1, margin);

  actual_str = mdd.str();
  expected_str = "{ 0: [18], 1: [17, 19, 13, 23, 18], 2: [16, 18, 12, 22, 17, 14, 8, 13], 3: [15, 17, 11, 21, 16, 13, 7, 12, 9, 3, 8] }";
  if (actual_str != expected_str) {
    std::cout << "Expected MDD: " << expected_str << std::endl;
    std::cout << "Actual MDD  : " << actual_str << std::endl;
    exit(0);
  }

  delete G;
  delete D;
}

