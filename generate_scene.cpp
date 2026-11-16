#include <dist_table.hpp>
#include <planner.hpp>
#include <restarter.hpp>
#include <thread_pool.hpp>

#include <filesystem>
#include <mutex>
#include <stdexcept>

namespace {

constexpr int worker_count = 7;
constexpr int max_failures = 5;
constexpr int bar_width = 20;

int positive_integer(const char *text, const char *name)
{
  const std::string value(text);
  size_t end = 0;
  int result;
  try {
    result = std::stoi(value, &end);
  } catch (const std::invalid_argument &) {
    throw std::invalid_argument(std::string(name) + " must be a positive integer");
  } catch (const std::out_of_range &) {
    throw std::invalid_argument(std::string(name) + " is too large");
  }
  if (end != value.size() || result <= 0) {
    throw std::invalid_argument(std::string(name) + " must be a positive integer");
  }
  return result;
}

void add_random_vertices(const Graph &graph, Config &config, int count,
                         std::mt19937 &rng)
{
  std::vector<bool> used(graph.size(), false);
  for (const auto vertex : config) used[vertex->id] = true;
  Vertices available;
  for (const auto vertex : graph.V) {
    if (!used[vertex->id]) available.push_back(vertex);
  }
  std::shuffle(available.begin(), available.end(), rng);
  config.insert(config.end(), available.begin(), available.begin() + count);
}

void print_progress(int completed, int total)
{
  const int percent = static_cast<int>(100LL * completed / total);
  const int filled = static_cast<int>(1LL * bar_width * completed / total);
  std::cout << '\r' << '[' << std::string(filled, '#')
            << std::string(bar_width - filled, '.') << "] " << percent << '%'
            << std::flush;
}

bool run_seeds(const Instance &instance, DistTable &distances, int seed_count)
{
  std::mutex progress_mutex;
  int completed = 0;
  bool all_successful = true;
  std::vector<std::future<void>> workers;
  ThreadPool pool(worker_count);
  print_progress(0, seed_count);

  for (int worker = 0; worker < std::min(worker_count, seed_count); ++worker) {
    workers.push_back(pool.submit([&, worker] {
      for (int seed = worker; seed < seed_count;) {
        const Deadline deadline(1000);
        Planner::MSG.clear();
        SPD restarter;
        Planner planner(&instance, 0, &deadline, seed, 0, &distances);
        planner.restarter = &restarter;
        const bool success = !planner.solve().empty();
        {
          std::lock_guard<std::mutex> lock(progress_mutex);
          all_successful = all_successful && success;
          print_progress(++completed, seed_count);
        }
        if (seed_count - seed <= worker_count) break;
        seed += worker_count;
      }
    }));
  }
  for (auto &worker : workers) worker.get();
  std::cout << '\n';
  return all_successful;
}

void save_scene(const Instance &instance, DistTable &distances,
                const std::string &map_name)
{
  std::ofstream out("out.scen", std::ios::trunc);
  if (!out) throw std::runtime_error("cannot open out.scen for writing");
  out << "version 1\n";
  for (size_t i = 0; i < instance.N; ++i) {
    const auto start = instance.starts[i];
    const auto goal = instance.goals[i];
    out << "0\t" << map_name << '\t' << instance.G->width << '\t'
        << instance.G->height << '\t' << start->x << '\t' << start->y << '\t'
        << goal->x << '\t' << goal->y << '\t' << distances.get(i, start)
        << '\n';
  }
  out.close();
  if (!out) throw std::runtime_error("cannot write out.scen");
}

}  // namespace

int main(int argc, char *argv[])
{
  if (argc != 5) {
    std::cerr << "usage: " << argv[0]
              << " <map_file> <num_of_seeds> <step_size> <max_agents>\n";
    return 1;
  }
  try {
    const std::filesystem::path map_path(argv[1]);
    const int seed_count = positive_integer(argv[2], "num_of_seeds");
    const int step_size = positive_integer(argv[3], "step_size");
    const int max_agents = positive_integer(argv[4], "max_agents");
    const std::string map_name = map_path.filename().string();
    if (map_path.extension() != ".map" ||
        map_name.find_first_of("\t\r\n") != std::string::npos) {
      throw std::invalid_argument("map file must have a valid .map filename");
    }
    if (!std::filesystem::is_regular_file(map_path)) {
      throw std::runtime_error("cannot open map file " + map_path.string());
    }
    Graph graph(map_path.string());
    if (graph.size() == 0) throw std::runtime_error("map has no passable vertices");
    if (max_agents > graph.size()) {
      throw std::invalid_argument("max_agents exceeds the map's passable vertices");
    }

    Planner::FLG_STAR = false;
    Planner::PIBT_NUM = 1;
    Planner::FLG_SCATTER = false;
    Planner::REFINER_NUM = 0;
    Planner::FLG_REFINER = false;
    Planner::FLG_MULTI_THREAD = false;

    std::mt19937 rng(std::random_device{}());
    Config starts;
    Config goals;
    int failures = 0;
    while (starts.size() < static_cast<size_t>(max_agents) &&
           failures < max_failures) {
      const auto accepted_count = starts.size();
      const int added = std::min(step_size, max_agents - static_cast<int>(accepted_count));
      add_random_vertices(graph, starts, added, rng);
      add_random_vertices(graph, goals, added, rng);
      const Instance instance(&graph, starts, goals, starts.size());
      std::cout << "Current number of agents: " << instance.N << ". Running "
                << seed_count << " seeds...\n";
      DistTable distances(instance);
      if (run_seeds(instance, distances, seed_count)) {
        save_scene(instance, distances, map_name);
        failures = 0;
      } else {
        starts.resize(accepted_count);
        goals.resize(accepted_count);
        ++failures;
        std::cout << "Candidate failed (" << failures << '/' << max_failures
                  << ").\n";
      }
    }
    if (starts.empty()) {
      std::cerr << "No scene passed all seeds; out.scen was not updated.\n";
      return 1;
    }
    std::cout << "Saved " << starts.size() << " agents to out.scen.\n";
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
