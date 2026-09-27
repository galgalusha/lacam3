#pragma once

#include "../include/graph.hpp"
#include "../include/dist_table.hpp"
#include "../include/instance.hpp"
#include "absl/hash/hash.h"

struct MDD {

  // an index in frontiers represents time so that frontiers[t]
  // is an MDD frontier, meaning, all the vertices that are on one of the
  // agents shortests paths at time t where frontiers[0] is the contains only
  // one vertex, the vertex the agent is at time 0.
  // The frontiers only go on in time until a specific horizon which is
  // frontiers.size(), so they do not necessarilly reach the goal of the agent.
  std::vector<std::vector<Vertex*>> frontiers;

  void populate(DistTable* D, Vertex* v_i, Vertex* v_g, int horizon);

  void render(Instance* ins);

  std::string str() const;

  bool operator==(const MDD& other) const {
    if (frontiers.size() != other.frontiers.size()) return false;
    for (size_t t = 0; t < frontiers.size(); ++t) {
      const auto& f1 = frontiers[t];
      const auto& f2 = other.frontiers[t];
      if (f1.size() != f2.size()) return false;
      for (size_t i = 0; i < f1.size(); ++i) {
        if (f1[i]->id != f2[i]->id) return false;
      }
    }
    return true;
  }  

  // "hashCode" method (using Abseil's framework)
  template <typename H>
  friend H AbslHashValue(H h, const MDD& mdd) {
    for (const auto& frontier : mdd.frontiers) {
      h = H::combine(std::move(h), frontier.size());
      for (Vertex* v : frontier) {
          h = H::combine(std::move(h), v->id);
      }
    }
    return h;
  }

  bool check_joint_mdd_conflict(MDD& other_mdd, Graph* G);

  MDD get_mdd_with_wait();

  static void test_joint_mdd();
  static void test_joint_mdd2();
  static void test_joint_mdd_with_wait();
  static void test_joint_mdd_with_wait2();

};

