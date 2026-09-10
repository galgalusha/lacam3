/*
 * high-level node of LaCAM
 */

#pragma once

#include "dist_table.hpp"
#include "lnode.hpp"

// high-level search node
struct HNode;
struct CompareHNodePointers {  // for determinism
  bool operator()(const HNode *lhs, const HNode *rhs) const;
};

struct HNode {
  static int COUNT;

  const Config C;
  HNode *parent;
  int depth;
  std::set<HNode *, CompareHNodePointers> neighbor;

  // value
  int g;
  int h;
  int f;

  // for low-level search
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<LNode *> search_tree;

  // prefix refinement: true = agent has reached its prefix goal (phase 2)
  std::vector<bool> agent_modes;

  HNode(Config _C, DistTable *D, HNode *_parent = nullptr, int _g = 0,
        int _h = 0, std::vector<bool> _modes = {});

  void clear_tree();
  void reset_tree();

  ~HNode();

  LNode *get_next_lowlevel_node(std::mt19937 &MT);
  bool inline is_at_goal(int i, DistTable* D) { return D->get(i, C[i]) == 0; }
};
using HNodes = std::vector<HNode *>;

std::ostream &operator<<(std::ostream &os, const HNode *H);
