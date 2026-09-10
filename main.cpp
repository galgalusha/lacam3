#include <argparse/argparse.hpp>
#include <filesystem>
#include <iostream>
#include <lacam.hpp>
#include <pair_wise_bin.hpp>
#include <pair_wise_db.hpp>
#include <drawing.hpp>
#include <asha_planner.hpp>
#include <pattern_db.hpp>
#include <mdd.hpp>
#include <horizon_pair_db.hpp>


int main(int argc, char *argv[])
{
  // arguments parser
  argparse::ArgumentParser program("lacam3", "0.1.0");
  program.add_argument("-m", "--map").help("map file").required();
  program.add_argument("-i", "--scen")
      .help("scenario file")
      .default_value(std::string(""));
  program.add_argument("-pair-db", "--pair-db")
      .help("use pair db (folder derived from map name)")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("-N", "--num").help("number of agents").required();
  program.add_argument("-s", "--seed")
      .help("seed")
      .default_value(std::string("0"));
  program.add_argument("-radius", "--radius")
      .help("PairDB radius")
      .default_value(std::string("3"));
  program.add_argument("-v", "--verbose")
      .help("verbose")
      .default_value(std::string("0"));
  program.add_argument("-t", "--time_limit_sec")
      .help("time limit sec")
      .default_value(std::string("3"));
  program.add_argument("-o", "--output")
      .help("output file")
      .default_value(std::string("./build/result.txt"));
  program.add_argument("-l", "--log_short")
      .default_value(false)
      .implicit_value(true);

  // solver parameters
  program.add_argument("--no-all")
      .help("turn off all options, i.e., vanilla LaCAM")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--fixed-tie")
      .help("Randomize tie breakers at the beginning of PIBT get_new_config")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--no-star")
      .help("turn off the anytime part, i.e., usual LaCAM")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--pair-db-gen")
      .help("Generate PairDB Database")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--pattern-db-gen")
      .help("Generate PatternDB")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--pair-db-test")
      .help("Test PairDB Database")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--random-insert-prob1")
      .help("probability of inserting the start node")
      .default_value(std::string("0.001"));
  program.add_argument("--random-insert-prob2")
      .help("probability of inserting a node after finding the goal")
      .default_value(std::string("0.01"));
  program.add_argument("--random-insert-init-node")
      .help("insert start node instead of random ones")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--no-swap")
      .help("turn off swap operation in PIBT")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--no-multi-thread")
      .help("turn off multi-threading")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--pibt-num")
      .help("used in Monte-Carlo configuration generation")
      .default_value(std::string("10"));
  program.add_argument("--no-scatter")
      .help("turn off SUO")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--scatter-margin")
      .help("allowing non-shortest paths in SUO")
      .default_value(std::string("10"));
  program.add_argument("--no-refiner")
      .help("turn off iterative refinement")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--asha")
      .help("use ASHA Planner")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--refiner-num")
      .help("specify the number of refiners")
      .default_value(std::string("4"));
  program.add_argument("--recursive-rate")
      .help("specify the rate of the recursive call of LaCAM")
      .default_value(std::string("0.2"));
  program.add_argument("--recursive-time-limit")
      .help("time limit (sec) of the recursive call")
      .default_value(std::string("1"));
  program.add_argument("--checkpoints-duration")
      .help("for recording")
      .default_value(std::string("5"));
  try {
    program.parse_known_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  // setup instance
  const auto verbose = std::stoi(program.get<std::string>("verbose"));
  const auto time_limit_sec =
      std::stoi(program.get<std::string>("time_limit_sec"));
  const auto scen_name = program.get<std::string>("scen");
  const auto seed = std::stoi(program.get<std::string>("seed"));
  const auto radius = std::stoi(program.get<std::string>("radius"));
  const auto map_name = program.get<std::string>("map");
  const auto test_pair_db = program.get<bool>("pair-db-test");
  const auto gen_pair_db = program.get<bool>("pair-db-gen");
  const auto gen_pattern_db = program.get<bool>("pattern-db-gen");
  const auto asha_planner = program.get<bool>("asha");
  const auto fixed_tie = program.get<bool>("fixed-tie");
  const auto use_pair_db = program.get<bool>("pair-db");
  if (use_pair_db && (map_name.size() < 4 || map_name.substr(map_name.size() - 4) != ".map")) {
    std::cerr << "error: map file must have a .map extension to use --pair-db" << std::endl;
    return 1;
  }
  const auto pair_db_name = std::filesystem::path(map_name).stem().string();
  const auto output_name = program.get<std::string>("output");
  const auto log_short = program.get<bool>("log_short");
  const auto N = std::stoi(program.get<std::string>("num"));
  auto ins = scen_name.size() > 0 ? Instance(scen_name, map_name, N)
                                        : Instance(map_name, N, seed);
  if (!ins.is_valid(1)) return 1;

  // solver parameters
  const auto flg_no_all = program.get<bool>("no-all");
  Planner::FLG_SWAP = !program.get<bool>("no-swap") && !flg_no_all;
  Planner::FLG_STAR = !program.get<bool>("no-star") && !flg_no_all;
  Planner::FLG_MULTI_THREAD =
      !program.get<bool>("no-multi-thread") && !flg_no_all;
  Planner::PIBT_NUM =
      flg_no_all ? 1 : std::stoi(program.get<std::string>("pibt-num"));
  Planner::FLG_REFINER = !program.get<bool>("no-refiner") && !flg_no_all;
  Planner::REFINER_NUM = std::stoi(program.get<std::string>("refiner-num"));
  Planner::FLG_SCATTER = !program.get<bool>("no-scatter") && !flg_no_all;
  Planner::SCATTER_MARGIN =
      std::stoi(program.get<std::string>("scatter-margin"));
  Planner::RANDOM_INSERT_PROB1 =
      flg_no_all ? 0
                 : std::stof(program.get<std::string>("random-insert-prob1"));
  Planner::RANDOM_INSERT_PROB2 =
      flg_no_all ? 0
                 : std::stof(program.get<std::string>("random-insert-prob2"));
  Planner::FLG_RANDOM_INSERT_INIT_NODE =
      program.get<bool>("random-insert-init-node") && !flg_no_all;
  Planner::RECURSIVE_RATE =
      flg_no_all ? 0 : std::stof(program.get<std::string>("recursive-rate"));
  Planner::RECURSIVE_TIME_LIMIT =
      flg_no_all
          ? 0
          : std::stof(program.get<std::string>("recursive-time-limit")) * 1000;
  Planner::CHECKPOINTS_DURATION =
      std::stof(program.get<std::string>("checkpoints-duration")) * 1000;

  PairWiseDB::RADIUS = radius;

  if (gen_pair_db) {
    std::cout << "\n[1] Constructing main bin files part" << std::endl;
    PairWiseDB pwh_no_goals(ins.G, pair_db_name);
    pwh_no_goals.construct_for_instance(ins.goals);
    std::cout << "\n[2] Adding goals bin files" << std::endl;
    PairWiseDB pwh_goals(ins.G, pair_db_name + "_goals");
    pwh_goals.construct_for_instance_only_goals(ins.goals);
    std::cout << "\n[3] Merging goals to main" << std::endl;
    merge_goal_folder(DB_PATH + pair_db_name, DB_PATH + pair_db_name + "_goals");
    PairWiseDB pair_db_mem(ins.G, pair_db_name);
    std::cout << "\n[4] Loading DB to memory" << std::endl;
    pair_db_mem.load_all(&ins);
    std::cout << "\n[5] Writing bin2 files" << std::endl;
    pair_db_mem.write_bin2_files();
    std::cout << "\nDone. You can delete the bin files and leave only the bin2 files." << std::endl;
    exit(0);
  }

  if (gen_pattern_db) {
    std::cout << "\n[1] Loading pair DB" << std::endl;
    auto pair_db = new PairWiseDB(ins.G, pair_db_name);
    pair_db->load_kernels(&ins);
    std::cout << "\n[2] Generating Pattern DB" << std::endl;
    auto pattern_db = new PatternDB();
    pattern_db->set_instance(&ins);
    pattern_db->populate_from(pair_db);
  }

  PIBT::FIXED_TIE = fixed_tie;
  if (fixed_tie) std::cout << "PIBT::FIXED_TIE=true" << std::endl;

  if (test_pair_db) {
    PairWiseDB pair_db(ins.G, pair_db_name);
    pair_db.test_interactive(&ins);
    // PairWiseDB::test();
    exit(0);
  }


  if (use_pair_db) {
    PIBT::pair_db = new PairWiseDB(ins.G, pair_db_name);
    PIBT::pair_db->load_kernels(&ins);
    // PIBT::pair_db->load_all2(&ins);
  }

//   int agent = 0;
//   MDD mdd(agent);
//   PairWiseDB pdb(ins.G, "bla");
//   mdd.populate(pdb.D, ins.starts[agent], ins.goals[agent], HorizonPairDB::HORIZON);
//   mdd.render(&ins);
//   exit(0);
  HorizonPairDB db(ins.G);
  db.generate_mdds();
  db.generate_conflicting_pairs();
  exit(0);


  // solve

  const auto deadline = Deadline(time_limit_sec * 1000);

  Solution solution;

  if (asha_planner) {
    info(1, verbose, &deadline, "pre-processing");
    auto planner = ASHA_Planner(&ins, verbose, &deadline, seed);
    solution = planner.solve();
  } else {
    solution = solve(ins, verbose - 1, &deadline, seed);
  }
  const auto comp_time_ms = deadline.elapsed_ms();

  // failure
  if (solution.empty()) info(1, verbose, &deadline, "failed to solve");

  // check feasibility
  if (!is_feasible_solution(ins, solution, verbose)) {
    info(0, verbose, &deadline, "invalid solution");
    return 1;
  }

  // post processing
  print_stats(verbose, &deadline, ins, solution, comp_time_ms);
  make_log(ins, solution, output_name, comp_time_ms, map_name, seed, log_short);

  return 0;
}
