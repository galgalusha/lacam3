#include "../include/pair_wise_db.hpp"

#include <climits>
#include <queue>
#include <vector>

// --- Shared state definitions ---
thread_local std::vector<int> t_closed_g;
thread_local std::vector<uint32_t> t_closed_gen;
thread_local uint32_t t_current_gen = 0;

std::vector<uint32_t> shared_zero_dh_gen;
uint32_t shared_zero_current_gen = 0;

std::vector<std::pair<Vertex*, Vertex*>> shared_bfs_q;

int joint_astar(DistTable* D, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal) {
  struct State {
    int g, f;
    Vertex* i;
    Vertex* j;
  };

  auto cmp = [](const State& a, const State& b) {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g;
  };
  std::priority_queue<State, std::vector<State>, decltype(cmp)> open(cmp);

  int V = D->K;

  if (t_closed_g.size() < (size_t)V * V) {
    t_closed_g.assign(V * V, INT_MAX);
    t_closed_gen.assign(V * V, 0);
  }

  if (++t_current_gen == 0) {
    std::fill(t_closed_gen.begin(), t_closed_gen.end(), 0);
    t_current_gen = 1;
  }

  auto encode = [&](int i, int j) { return i * V + j; };

  int h0 = D->get(i_goal->id, i_start->id) + D->get(j_goal->id, j_start->id);
  open.push({0, h0, i_start, j_start});

  while (!open.empty()) {
    auto [g, f, ci, cj] = open.top();
    open.pop();

    if (ci == i_goal && cj == j_goal) return g;

    int ek = encode(ci->id, cj->id);

    if (t_closed_gen[ek] == t_current_gen && t_closed_g[ek] <= g) continue;
    t_closed_gen[ek] = t_current_gen;
    t_closed_g[ek] = g;

    bool i_at_goal = (ci == i_goal);
    bool j_at_goal = (cj == j_goal);

    int i_deg = ci->neighbor.size();
    int j_deg = cj->neighbor.size();

    Vertex* moves_i[5];
    Vertex* moves_j[5];

    for (int k = 0; k < i_deg; ++k) moves_i[k] = ci->neighbor[k];
    moves_i[i_deg] = ci;

    for (int k = 0; k < j_deg; ++k) moves_j[k] = cj->neighbor[k];
    moves_j[j_deg] = cj;

    for (int i_idx = 0; i_idx <= i_deg; ++i_idx) {
      Vertex* ni = moves_i[i_idx];

      int h_i = D->get(i_goal->id, ni->id);
      int step_i = (i_at_goal && ni == i_goal) ? 0 : 1;

      for (int j_idx = 0; j_idx <= j_deg; ++j_idx) {
        Vertex* nj = moves_j[j_idx];

        if (ni == ci && nj == cj && !i_at_goal && !j_at_goal) continue;
        if (ni == nj) continue;
        if (ni == cj && nj == ci) continue;

        int step_j = (j_at_goal && nj == j_goal) ? 0 : 1;
        int ng = g + step_i + step_j;

        int nek = encode(ni->id, nj->id);

        if (t_closed_gen[nek] == t_current_gen && t_closed_g[nek] <= ng) continue;

        int nf = ng + h_i + D->get(j_goal->id, nj->id);
        open.push({ng, nf, ni, nj});
      }
    }
  }

  return INT_MAX;
}

bool PairWiseDB::can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal) {
  int dist_s_ij  = D->get(j_start, i_start);
  int dist_g_ij  = D->get(j_goal,  i_goal);
  int dist_sg_ii = D->get(i_goal,  i_start);
  int dist_sg_jj = D->get(j_goal,  j_start);

  int psi_ij = dist_s_ij + dist_g_ij - dist_sg_ii - dist_sg_jj;
  if (psi_ij > 0) return false;

  int dist_sg_ji = D->get(i_goal,  j_start);
  int dist_sg_ij = D->get(j_goal,  i_start);

  int lambda_ij = dist_sg_ii - dist_sg_ji;
  int lambda_ji = dist_sg_jj - dist_sg_ij;

  if (lambda_ij + lambda_ji < 0) return false;

  return true;
}

bool PairWiseDB::has_alternative_path(DistTable* D, Vertex* blocked, Vertex* v_s, Vertex* v_g) {
  int target_dist = D->get(v_g->id, v_s->id);
  if (target_dist == D->K) return false;
  if (v_s == blocked) return false;

  std::vector<bool> visited(D->K, false);
  std::vector<Vertex*> frontier;
  frontier.push_back(v_s);
  visited[v_s->id] = true;

  while (!frontier.empty()) {
    std::vector<Vertex*> next;
    for (Vertex* cur : frontier) {
      if (cur == v_g) return true;
      int remaining = D->get(v_g->id, cur->id);
      for (Vertex* nb : cur->neighbor) {
        if (nb == blocked) continue;
        if (visited[nb->id]) continue;
        if (D->get(v_g->id, nb->id) != remaining - 1) continue;
        visited[nb->id] = true;
        next.push_back(nb);
      }
    }
    frontier = std::move(next);
  }
  return false;
}

void populate_zero_dh_by_bfs(DistTable* D, Vertex* i_goal, Vertex* j_goal) {
  int V = D->K;

  if (++shared_zero_current_gen == 0) {
    std::fill(shared_zero_dh_gen.begin(), shared_zero_dh_gen.end(), 0);
    shared_zero_current_gen = 1;
  }

  shared_bfs_q.clear();
  shared_bfs_q.push_back({i_goal, j_goal});

  shared_zero_dh_gen[i_goal->id * V + j_goal->id] = shared_zero_current_gen;

  size_t head = 0;
  while (head < shared_bfs_q.size()) {
    auto [u, v] = shared_bfs_q[head++];

    int d_i = D->get(i_goal->id, u->id);
    int d_j = D->get(j_goal->id, v->id);

    Vertex* moves_i[5];
    int deg_i = 0;
    for (Vertex* n : u->neighbor) {
      if (D->get(i_goal->id, n->id) == d_i + 1) moves_i[deg_i++] = n;
    }
    if (u == i_goal) moves_i[deg_i++] = u;

    Vertex* moves_j[5];
    int deg_j = 0;
    for (Vertex* n : v->neighbor) {
      if (D->get(j_goal->id, n->id) == d_j + 1) moves_j[deg_j++] = n;
    }
    if (v == j_goal) moves_j[deg_j++] = v;

    for (int i_idx = 0; i_idx < deg_i; ++i_idx) {
      Vertex* u_prime = moves_i[i_idx];

      for (int j_idx = 0; j_idx < deg_j; ++j_idx) {
        Vertex* v_prime = moves_j[j_idx];

        if (u_prime == u && v_prime == v) continue;
        if (u_prime == v_prime) continue;
        if (u_prime == v && v_prime == u) continue;

        int ek = u_prime->id * V + v_prime->id;

        if (shared_zero_dh_gen[ek] != shared_zero_current_gen) {
          shared_zero_dh_gen[ek] = shared_zero_current_gen;
          shared_bfs_q.push_back({u_prime, v_prime});
        }
      }
    }
  }
}
