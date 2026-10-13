#include <cereal/archives/json.hpp>
#include <evaluation.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

static void load_conf(cereal::JSONInputArchive &ar, EvalConfig &c)
{
  ar(cereal::make_nvp("asset", c.asset),
     cereal::make_nvp("num_of_agents", c.num_of_agents),
     cereal::make_nvp("num_of_scenes", c.num_of_scenes),
     cereal::make_nvp("objective", c.objective),
     cereal::make_nvp("seeds_per_scene", c.seeds_per_scene),
     cereal::make_nvp("pair_db", c.pair_db),
     cereal::make_nvp("pair_db_horizon", c.pair_db_horizon),
     cereal::make_nvp("no_star", c.no_star),
     cereal::make_nvp("no_scatter", c.no_scatter),
     cereal::make_nvp("pibt_num", c.pibt_num),
     cereal::make_nvp("refiner_num", c.refiner_num),
     cereal::make_nvp("no_refiner", c.no_refiner),
     cereal::make_nvp("verbose", c.verbose),
     cereal::make_nvp("timeout_seconds", c.timeout_seconds));
}

int main(int argc, char *argv[])
{
  std::string path;
  bool recover = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-recover" || arg == "--recover") {
      recover = true;
    } else if (path.empty()) {
      path = arg;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::cerr << "usage: " << argv[0] << " [-recover] <evaluation.json>"
              << std::endl;
    return 1;
  }
  try {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    EvalConfig conf;
    {
      cereal::JSONInputArchive archive(in);
      load_conf(archive, conf);
    }
    conf.recover = recover;
    conf.file_name = std::filesystem::path(path).stem().string();
    EvalRunner().run_config(conf);
    std::cout << "evaluation finished: evaluations/" << conf.file_name
              << ".results.csv" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
