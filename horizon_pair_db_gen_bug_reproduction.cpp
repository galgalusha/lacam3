#include <horizon_pair_db.hpp>
#include <horizon_pair_db_generator.hpp>



static DistTable* create_dist_table(Graph* G) {
  std::cout << "Creating the big DistTable" << std::endl;
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  std::cout << "Finished creating the big DistTable" << std::endl;
  return D;
}


int main() {
  int H = 3;
  int M = 3;
  std::cout << "Running test_db_gen_bug" << std::endl;
  std::vector<std::string> grid = {
  "..........@......@..",
  "@...@.@@...........@",
  "..............@.....",
  ".....@.@..........@.",
  ".@@...........@@.@..",
  "......@..@.@..@@....",
  "......@..@@........@",
  ".@...@..............",
  "......@.........@...",
  "..@.@............@.@",
  "@.@.........@.@.....",
  "............@.....@.",
  "..@.....@.@.@@.@....",
  ".@................@.",
  "...@@@.......@......",
  "........@....@.@.@..",
  "..@...@........@....",
  ".@......@@@..@.@..@.",
  "@.........@........@",
  "......@...@..@..@.@.",
  };
  Graph* G = new Graph(grid);
  DistTable* D = create_dist_table(G);
  HorizonPairDBGenerator generator(G, "test_bug", H, M);
  Vertex *start1 = G->U[0]                , *goal1 = G->U[0];
  Vertex *start2 = G->U[0 * G->width + 4] , *goal2 = G->U[19 * G->width + 19];
  MDD mdd1, mdd2;
  mdd1.populate(D, start1, goal1, H + 1, M);
  mdd2.populate(D, start2, goal2, H + 1, M);

  generator.mdd_by_id = { mdd2 }; // id of mdd2 is 0
  auto penalty = generator.calculate_penalty(mdd1, 0);
  std::cout << "penalty: " << (int)penalty.penalty << std::endl;
  std::cout << "time   : " << (int)penalty.time << std::endl;

  std::cout << mdd1.str() << std::endl;
  std::cout << mdd2.str() << std::endl;

  std::cout << "D(3) : " << D->get(3, goal2) << std::endl;
  std::cout << "D(4) : " << D->get(4, goal2) << std::endl;
  std::cout << "D(5) : " << D->get(5, goal2) << std::endl;


  delete G;
  delete D;
  return 0;
}
