#pragma once

#include <string>


// struct EvalGlobals {
//   static std::string FOLDER;
// };


struct EvalConfig {
  std::string file_name;
  std::string asset;
  int num_of_agents = 1;
  int num_of_scenes = 1;
  int objective = 2;
  int seeds_per_scene = 1;
  bool pair_db = false;
  int pair_db_horizon = 0;
  bool no_star = false;
  bool no_scatter = false;
  int pibt_num = 10;
  int refiner_num = 4;
  bool no_refiner = false;
  int verbose = 0;
  int timeout_seconds = 60;
};


struct PlannerResult {
  int run_time_milliseconds;
  bool is_success;
  int cost;
  int num_of_iterations;
  int num_of_explorations;
};


struct EvalResultRow {
  PlannerResult planner_result;
  int seed;
  std::string scen_name;
};


struct EvalRunner {
  /**
   * Loads the asset map and first ${conf.num_of_scenes} scenario files sorted
   * by file name. Assets may be stored in assets/${conf.asset}/ with a
   * matching map file, or directly in assets/.
   * For each scen, for each seed, it shall create
   * a new Planner and run it to get a PlannerResult.
   * Then, create EvalResultRow and write it to a CSV file
   * in evaluations/${config.file_name}.results.csv
   */
  void run_config(EvalConfig& conf);

};
