/*
 * Implementation of SUO
 *
 * references:
 * Optimizingspaceutilizationformoreeffective multi-robot path planning.
 * Shuai D Han and Jingjin Yu.
 * In Proceedings of IEEE International Conference on Robotics and Automation
 * (ICRA). 2022.
 */
#pragma once

#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "utils.hpp"

struct IScatter {
  // Returns the suggested next vertex for agent at the given time and current vertex,
  // or nullptr if no suggestion exists.
  virtual Vertex* get_neighbor(int time, int agent, int vertex_id) const = 0;
  virtual void construct(int iterations) = 0;
  virtual const std::vector<Path>& get_paths() const = 0;
  virtual ~IScatter() = default;
};


struct Scatter : IScatter {
  const Instance *ins;
  Deadline *deadline;
  std::mt19937 MT;
  const int verbose;
  const int N;
  const int V_size;
  const int T;  // makespan lower bound
  DistTable *D;
  const int cost_margin;
  int sum_of_path_length;

  // outcome
  std::vector<Path> paths;
  // agent, vertex-id, next vertex
  std::vector<std::unordered_map<int, Vertex *>> scatter_data;

  Vertex* get_neighbor(int /*time*/, int agent, int vertex_id) const override
  {
    auto itr = scatter_data[agent].find(vertex_id);
    return itr != scatter_data[agent].end() ? itr->second : nullptr;
  }

  const std::vector<Path>& get_paths() const override { return paths; }

  // collision data
  CollisionTable CT;

  void construct(int iterations);

  Scatter(const Instance *_ins, DistTable *_D, Deadline *_deadline,
          const int seed = 0, int _verbose = 0, int _cost_margin = 2);
};

struct NoScatter : IScatter {
  std::vector<Path> paths;
  Vertex* get_neighbor(int /*time*/, int agent, int vertex_id) const override
  {
    return nullptr;
  }
  const std::vector<Path>& get_paths() const override { return paths; }
  void construct(int iterations) {}
};
