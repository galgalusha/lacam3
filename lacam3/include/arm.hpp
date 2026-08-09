#pragma once

#include "graph.hpp"
#include "scatter.hpp"

#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

struct ArmConfig {
  ScatterType scatter_type;
  int margin;

  std::string str() const;
  bool operator==(const ArmConfig &o) const {
    return scatter_type == o.scatter_type && margin == o.margin;
  }
};

extern std::vector<float> PATH_RATIO;

struct Arm {
  ArmConfig config;
  int improvement_count;
  int pulls;
  int idx_path_ratio;

  Arm(ArmConfig _config);

  float get_path_ratio();
  float get_score(int total_pulls) const;
};

struct ScatterMAB {
  std::vector<Arm> arms;
  std::vector<std::unordered_map<Config, IScatter *, ConfigHasher>> cache;
  std::mutex mtx;
  int total_pulls;

  ScatterMAB(std::vector<ArmConfig> configs);
  ~ScatterMAB();

  int choose_arm();
  int choose_ready_arm(const Config &starts);  // only considers arms with a cached scatter
  void record_pull(int arm_idx);
  void record_improvement(int arm_idx);
  IScatter *get_cached(int arm_idx, const Config &starts);
  void set_cached(int arm_idx, const Config &starts, IScatter *scatter);
};

extern ScatterMAB main_scatter_mab;
extern ScatterMAB refiners_scatter_mab;
