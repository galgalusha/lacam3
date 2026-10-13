#include "../include/evaluation.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "../include/horizon_pair_db.hpp"
#include "../include/metrics.hpp"
#include "../include/pibt.hpp"
#include "../include/planner.hpp"
#include "../include/restarter.hpp"

namespace {

namespace fs = std::filesystem;

std::string csv_escape(const std::string& value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) return value;

  std::string escaped = "\"";
  for (const auto character : value) {
    if (character == '"') escaped += '"';
    escaped += character;
  }
  escaped += '"';
  return escaped;
}

fs::path find_map_file(const std::string& asset)
{
  const fs::path assets_dir("assets");
  const fs::path asset_path(asset);
  const auto asset_folder = assets_dir / asset_path;

  if (fs::is_directory(asset_folder)) {
    const auto map_path =
        asset_folder / (asset_folder.filename().string() + ".map");
    if (fs::is_regular_file(map_path)) return map_path;
  }

  const auto path_as_given = assets_dir / asset_path;
  if (path_as_given.extension() == ".map" &&
      fs::is_regular_file(path_as_given)) {
    return path_as_given;
  }

  const auto map_path = fs::path(path_as_given.string() + ".map");
  if (fs::is_regular_file(map_path)) return map_path;

  throw std::runtime_error("could not find map file for asset '" + asset +
                           "' under " + assets_dir.string());
}

std::vector<fs::path> find_scenarios(const fs::path& directory,
                                     const int requested_count)
{
  std::vector<fs::path> scenarios;
  for (const auto& entry : fs::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".scen") {
      scenarios.push_back(entry.path());
    }
  }
  std::sort(scenarios.begin(), scenarios.end(),
            [](const fs::path& lhs, const fs::path& rhs) {
              return lhs.filename().string() < rhs.filename().string();
            });

  if (scenarios.size() < static_cast<size_t>(requested_count)) {
    throw std::runtime_error("asset directory '" + directory.string() +
                             "' contains only " +
                             std::to_string(scenarios.size()) +
                             " scenario files; " +
                             std::to_string(requested_count) +
                             " were requested");
  }
  scenarios.resize(static_cast<size_t>(requested_count));
  return scenarios;
}

}  // namespace

void EvalRunner::run_config(EvalConfig& conf)
{
  if (conf.file_name.empty()) {
    throw std::invalid_argument("evaluation file_name must not be empty");
  }
  if (conf.asset.empty()) {
    throw std::invalid_argument("evaluation asset must not be empty");
  }
  if (conf.num_of_agents <= 0 || conf.num_of_scenes <= 0 ||
      conf.seeds_per_scene <= 0 || conf.timeout_seconds <= 0 ||
      conf.pibt_num <= 0 || conf.refiner_num < 0 || conf.objective < 0) {
    throw std::invalid_argument("evaluation config contains an invalid value");
  }
  if (conf.pair_db && conf.pair_db_horizon < 0) {
    throw std::invalid_argument(
        "pair_db_horizon must be non-negative when pair_db is enabled");
  }

  const auto map_path = find_map_file(conf.asset);
  const auto scenarios =
      find_scenarios(map_path.parent_path(), conf.num_of_scenes);
  const auto output_path =
      fs::path("evaluations") / (conf.file_name + ".results.csv");
  if (!output_path.parent_path().empty()) {
    fs::create_directories(output_path.parent_path());
  }
  std::ofstream output(output_path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("could not open evaluation output file '" +
                             output_path.string() + "'");
  }

  output << "run_time_milliseconds,is_success,cost,num_of_iterations,"
            "num_of_explorations,seed,scen_name\n";
  if (!output) {
    throw std::runtime_error("could not write evaluation output file '" +
                             output_path.string() + "'");
  }

  Planner::FLG_STAR = !conf.no_star;
  Planner::FLG_SCATTER = !conf.no_scatter;
  Planner::FLG_REFINER = !conf.no_refiner;
  Planner::PIBT_NUM = conf.pibt_num;
  Planner::REFINER_NUM = conf.refiner_num;

  for (const auto& scen_path : scenarios) {
    const Instance instance(scen_path.string(), map_path.string(),
                            conf.num_of_agents);
    if (!instance.is_valid(conf.verbose)) {
      throw std::runtime_error("scenario '" + scen_path.string() +
                               "' does not contain a valid instance for " +
                               std::to_string(conf.num_of_agents) + " agents");
    }

    std::unique_ptr<HorizonPairDB> pair_db;
    if (conf.pair_db) {
      const auto db_name =
          "H_" + std::to_string(conf.pair_db_horizon) + "_" +
          map_path.stem().string();
      pair_db = std::make_unique<HorizonPairDB>(instance.G, db_name);
      if (!pair_db->load_from_file()) {
        throw std::runtime_error("could not load pair database '" + db_name +
                                 "'");
      }
      PIBT::pair_db = pair_db.get();
    } else {
      PIBT::pair_db = nullptr;
    }

    for (int seed = 0; seed < conf.seeds_per_scene; ++seed) {
      const Deadline deadline(
          static_cast<double>(conf.timeout_seconds) * 1000.0);
      Planner::MSG.clear();
      Planner planner(&instance, conf.verbose, &deadline, seed);
      planner.restarter = new SPD();
      const auto solution = planner.solve();

      const PlannerResult result{
          static_cast<int>(deadline.elapsed_ms()),
          !solution.empty(),
          solution.empty() ? -1 : get_sum_of_costs(solution),
          planner.search_iter,
          static_cast<int>(planner.EXPLORED.size())};

      output << result.run_time_milliseconds << ','
             << (result.is_success ? "true" : "false") << ',' << result.cost
             << ',' << result.num_of_iterations << ','
             << result.num_of_explorations << ',' << seed << ','
             << csv_escape(scen_path.filename().string()) << '\n';
      output.flush();
      if (!output) {
        throw std::runtime_error("failed while writing evaluation output file '" +
                                 output_path.string() + "'");
      }
    }
  }
}
