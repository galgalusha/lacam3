#include <cstdint>
#include <iostream>
#include <lacam.hpp>
#include <horizon_pair_db.hpp>

static void print_progress(uint64_t done, uint64_t total)
{
  const int width = 50;
  const double frac = total ? (double)done / (double)total : 1.0;
  const int filled = (int)(frac * width);
  std::cerr << "\r[";
  for (int i = 0; i < width; ++i) std::cerr << (i < filled ? '#' : '-');
  std::cerr << "] " << (int)(frac * 100) << "% (" << done << "/" << total << ")"
            << std::flush;
}

int main(int argc, char *argv[])
{
  if (argc != 4) {
    std::cerr << "usage: " << argv[0] << " <map file> <pair-db name 1> <pair-db name 2>"
              << std::endl;
    return 1;
  }
  const std::string map_name = argv[1];

  Instance ins(map_name, 0, 0);
  Graph *G = ins.G;

  HorizonPairDB db1(G, argv[2]);
  HorizonPairDB db2(G, argv[3]);
  if (!db1.load_from_file()) {
    std::cerr << "failed to load pair-db " << argv[2] << std::endl;
    return 1;
  }
  if (!db2.load_from_file()) {
    std::cerr << "failed to load pair-db " << argv[3] << std::endl;
    return 1;
  }

  const uint64_t n = G->V.size();
  const uint64_t total = n * n * n * n;
  const uint64_t report_every = std::max<uint64_t>(1, total / 1000);
  uint64_t done = 0;

  auto xy = [](Vertex *v) {
    return "(" + std::to_string(v->x) + ", " + std::to_string(v->y) + ")";
  };

  for (auto start1 : G->V) {
    for (auto goal1 : G->V) {
      const uint32_t db1_mdd1 = db1.mdd_id_by_v_g[start1->id * db1.V_SIZE + goal1->id];
      const uint32_t db2_mdd1 = db2.mdd_id_by_v_g[start1->id * db2.V_SIZE + goal1->id];
      for (auto start2 : G->V) {
        for (auto goal2 : G->V) {
          const uint32_t db1_mdd2 = db1.mdd_id_by_v_g[start2->id * db1.V_SIZE + goal2->id];
          const uint32_t db2_mdd2 = db2.mdd_id_by_v_g[start2->id * db2.V_SIZE + goal2->id];
          const auto p1 = db1.get_penalty(db1_mdd1, db1_mdd2);
          const auto p2 = db2.get_penalty(db2_mdd1, db2_mdd2);
          if (p1.penalty != p2.penalty) {
            
            bool is_ok1 = p1.penalty < p2.penalty && p2.penalty - p1.penalty == db2.MARGIN - db1.MARGIN;
            bool is_ok2 = p2.penalty < p1.penalty && p1.penalty - p2.penalty == db1.MARGIN - db2.MARGIN;

            if ((!is_ok1) && (!is_ok2)) {
              std::cerr << std::endl;
              std::cout << "Agent1:  " << xy(start1) << " -> " << xy(goal1) << "\n"
                        << "Agent2:  " << xy(start2) << " -> " << xy(goal2) << "\n"
                        << "DB1 penalty: " << (int)p1.penalty << "\n"
                        << "DB2 penalty: " << (int)p2.penalty << std::endl;
              return 0;
            }
          }
          if (++done % report_every == 0) print_progress(done, total);
        }
      }
    }
  }
  print_progress(total, total);
  std::cerr << std::endl << "All penalties are equal." << std::endl;
  return 0;
}
