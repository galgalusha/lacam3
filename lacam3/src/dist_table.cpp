#include "../include/dist_table.hpp"

#include <atomic>
#include <iomanip>
#include <iostream>
#include <mutex>

static void print_progress_bar(size_t done, size_t total) {
  const int bar_width = 40;
  float frac = total > 0 ? (float)done / (float)total : 1.0f;
  int filled = (int)(frac * bar_width);
  std::cout << "\r[";
  for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
  std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
}

DistTable::DistTable(const Instance &ins, bool toward_goal)
    : K(ins.G->V.size()), table(ins.N, std::vector<int>(K, K))
{
  setup(&ins, toward_goal);
}

DistTable::DistTable(const Instance *ins, bool toward_goal)
    : K(ins->G->V.size()), table(ins->N, std::vector<int>(K, K))
{
  setup(ins, toward_goal);
}

void DistTable::setup(const Instance *ins, bool toward_goal)
{
  std::mutex io_mtx;  // protects the progress bar since bfs() runs on multiple threads
  std::atomic<int> done = 0;
  print_progress_bar(0, ins->N);

  auto bfs = [&](const int i) {
    auto g_i = toward_goal ? ins->goals[i] : ins->starts[i];
    auto Q = std::queue<Vertex *>({g_i});
    table[i][g_i->id] = 0;
    while (!Q.empty()) {
      auto n = Q.front();
      Q.pop();
      const int d_n = table[i][n->id];
      for (auto &m : n->neighbor) {
        const int d_m = table[i][m->id];
        if (d_n + 1 >= d_m) continue;
        table[i][m->id] = d_n + 1;
        Q.push(m);
      }
    }
    const int done_now = ++done;
    std::lock_guard lock(io_mtx);
    print_progress_bar(done_now, ins->N);
  };

  const int num_threads =
      std::max(1u, std::min((unsigned)ins->N,
                            std::thread::hardware_concurrency()));
  auto pool = std::vector<std::future<void>>();
  for (int t = 0; t < num_threads; ++t) {
    pool.emplace_back(std::async(std::launch::async, [&, t]() {
      for (int i = t; i < (int)ins->N; i += num_threads) bfs(i);
    }));
  }

  // std::future from std::async(launch::async, ...) blocks in its destructor
  // until the task finishes, so this explicit get() loop just makes that
  // existing wait visible rather than relying on pool's implicit destruction.
  for (auto &fut : pool) fut.get();
  std::cout << std::endl;
}

DistTable::DistTable(int K) : K(K) {}

int DistTable::get(const int i, const int v_id) { return table[i][v_id]; }

int DistTable::get(const int i, const Vertex *v) { return get(i, v->id); }

DoubleModeDistTable::DoubleModeDistTable(int K, DistTable *d_prefix, DistTable *d_real)
    : DistTable(K), D_prefix(d_prefix), D_real(d_real), agent_modes(nullptr) {}

void DoubleModeDistTable::set_active_modes(const std::vector<bool> *modes)
{
  agent_modes = modes;
}

int DoubleModeDistTable::get(const int i, const int v_id)
{
  if (agent_modes && (*agent_modes)[i]) return D_real->get(i, v_id);
  return D_prefix->get(i, v_id);
}

int DoubleModeDistTable::get(const int i, const Vertex *v)
{
  return get(i, v->id);
}
