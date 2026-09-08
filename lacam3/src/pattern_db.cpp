#include "../include/pattern_db.hpp"
#include "../include/pair_wise_db.hpp"
#include "../include/drawing.hpp"

#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>

const int PatternDB::RADIUS = 2;

namespace
{
// records which two agents (and their start/goal vertices) produced a given
// pattern, so conflicting inserts can be visualized for debugging.
struct DebugRecord {
  Vertex *i_start, *i_goal, *j_start, *j_goal;
  uint8_t value;
};

// MoveQuality = D(goal_id, v) - D(goal_id, neighbor), or BLOCKED_MOVE if
// neighbor does not exist. Clamped to keep BLOCKED_MOVE a unique sentinel.
MoveQuality move_quality(DistTable *D, int goal_id, Vertex *v, Vertex *neighbor)
{
  const int diff = D->get(goal_id, v) - D->get(goal_id, neighbor);
  return static_cast<MoveQuality>(diff);
}

AgentProfile build_profile(DistTable *D, int goal_id, Vertex *v)
{
  AgentProfile profile;
  profile.left = BLOCKED_MOVE;
  profile.right = BLOCKED_MOVE;
  profile.up = BLOCKED_MOVE;
  profile.down = BLOCKED_MOVE;
  const int x = v->x;
  const int y = v->y;
  for (auto n : v->neighbor) {
    const int dx = n->x - x;
    const int dy = n->y - y;
    if (dx == -1 && dy == 0) profile.left = move_quality(D, goal_id, v, n);
    else if (dx == 1 && dy == 0) profile.right = move_quality(D, goal_id, v, n);
    else if (dx == 0 && dy == -1) profile.up = move_quality(D, goal_id, v, n);
    else if (dx == 0 && dy == 1) profile.down = move_quality(D, goal_id, v, n);
  }
  return profile;
}

std::string profile_str(const AgentProfile &p)
{
  std::ostringstream oss;
  auto append = [&](const char *name, MoveQuality q) {
    oss << name << "=";
    if (q == BLOCKED_MOVE) oss << "X";
    else oss << static_cast<int>(q);
  };
  oss << "{";
  append("L", p.left);
  oss << ",";
  append("R", p.right);
  oss << ",";
  append("U", p.up);
  oss << ",";
  append("D", p.down);
  oss << "}";
  return oss.str();
}
// pairs a pattern with the two conflicting scenarios that produced it, for
// deferred (non-interleaved with progress bar) reporting.
struct Conflict {
  Pattern pattern;
  DebugRecord orig;
  DebugRecord conflicting;
};
}  // namespace

std::string Pattern::str() const
{
  std::ostringstream oss;
  oss << "Pattern(dx=" << static_cast<int>(dx) << ", dy=" << static_cast<int>(dy)
      << ", agent=" << profile_str(agent)
      << ", other_agent=" << profile_str(other_agent) << ")";
  return oss.str();
}

uint8_t PatternDB::get(Pattern &pattern)
{
  auto it = data.find(pattern);
  return it == data.end() ? NO_DATA : it->second;
}

uint8_t PatternDB::set(Pattern &pattern, uint8_t value)
{
  auto it = data.find(pattern);
  if (it != data.end()) {
    if (it->second != value) {
      std::cout << "PatternDB conflict for " << pattern.str()
                << ": existing=" << static_cast<int>(it->second)
                << ", new=" << static_cast<int>(value) << std::endl;
    }
    return it->second;
  }
  data.emplace(pattern, value);
  return value;
}

void PatternDB::set_instance(Instance *_ins)
{
  ins = _ins;
  D = create_dist_table(ins->G);
}

DistTable *PatternDB::create_dist_table(Graph *G)
{
  Config goals = G->V;
  Instance *ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}

Pattern PatternDB::pattern(int agent, int other_agent, Vertex *v, Vertex *other_v)
{
  Pattern p;
  p.dx = static_cast<int8_t>(other_v->x - v->x);
  p.dy = static_cast<int8_t>(other_v->y - v->y);
  p.agent = build_profile(D, ins->goals[agent]->id, v);
  p.other_agent = build_profile(D, ins->goals[other_agent]->id, other_v);
  return p;
}

void PatternDB::populate_from(PairWiseDB *pair_db)
{
  Graph *G = ins->G;
  const Config &goals = ins->goals;
  const int K = (int)goals.size();
  const int num_vertices = (int)G->V.size();

  const long long total_outer = (long long)K * (K - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  std::mutex data_mutex;
  absl::flat_hash_map<Pattern, DebugRecord, Pattern::PatternHash> tmp_data;
  std::atomic<bool> conflict_found{false};
  std::optional<Conflict> conflict;
  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;

  for (int agent1 = 0; agent1 < K && !conflict_found.load(); ++agent1) {
    for (int agent2 = agent1 + 1; agent2 < K && !conflict_found.load(); ++agent2, ++outer_idx) {
      const int pct = (int)(outer_idx * 100000 / total_outer);
      if (pct != last_pct) {
        last_pct = pct;
        const int filled = pct / 2000;
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0')
                  << (pct % 1000) << "%" << std::flush;
      }

      Vertex *i_g = goals[agent1];
      Vertex *j_g = goals[agent2];

      futures.push_back(pool.submit([this, G, i_g, j_g, agent1, agent2, num_vertices, pair_db,
                                      &data_mutex, &tmp_data, &conflict_found, &conflict]() {
        for (int i_s = 0; i_s < num_vertices && !conflict_found.load(); ++i_s) {
          Vertex *vi_s = G->V[i_s];

          // 
          // TEMP!
          // We don't support goals inside the pattern of i_s
          //
          if (D->get(i_s, i_g) <= RADIUS || D->get(i_s, j_g) <= RADIUS) continue;

          for (int j_s = 0; j_s < num_vertices && !conflict_found.load(); ++j_s) {
            if (i_s == j_s) continue;
            if (D->get(i_s, j_s) > RADIUS) continue;

            Vertex *vj_s = G->V[j_s];

            const uint8_t dh = pair_db->get(agent1, agent2, vi_s, vj_s);
            // if (dh == 0) continue;

            Pattern p = pattern(agent1, agent2, vi_s, vj_s);
            DebugRecord record{vi_s, i_g, vj_s, j_g, dh};

            std::lock_guard<std::mutex> lock(data_mutex);
            auto it = tmp_data.find(p);
            if (it != tmp_data.end() && it->second.value != dh) {
              conflict = Conflict{p, it->second, record};
              conflict_found.store(true);
              break;
            }
            tmp_data.emplace(p, record);
            set(p, dh);
          }
        }
        return ThreadResult{{}, 0};
      }));
    }
  }

  for (auto &fut : futures) fut.get();

  if (conflict) {
    const DebugRecord &orig = conflict->orig;
    const DebugRecord &confl = conflict->conflicting;
    std::cout << std::endl
              << "PatternDB conflict for " << conflict->pattern.str() << ": original dh="
              << (int)orig.value << ", conflicting dh=" << (int)confl.value << std::endl;
    std::cout << "Original scenario:" << std::endl;
    draw_map(G, {}, orig.i_goal->index, orig.i_start->index,
             {{orig.j_goal->index, orig.j_start->index}});
    std::cout << "Conflicting scenario:" << std::endl;
    draw_map(G, {}, confl.i_goal->index, confl.i_start->index,
             {{confl.j_goal->index, confl.j_start->index}});
    exit(0);
  }

  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
  std::cout << "PatternDB populated with " << data.size() << " entries" << std::endl;
}

bool PatternDB::save_to_file(const std::string &filename) const
{
  std::ofstream ofs(filename, std::ios::binary | std::ios::trunc);
  if (!ofs) return false;

  const uint64_t count = data.size();
  ofs.write(reinterpret_cast<const char *>(&count), sizeof(count));
  for (const auto &[pattern, value] : data) {
    ofs.write(reinterpret_cast<const char *>(&pattern), sizeof(Pattern));
    ofs.write(reinterpret_cast<const char *>(&value), sizeof(value));
  }
  return static_cast<bool>(ofs);
}

bool PatternDB::load_from_file(const std::string &filename)
{
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) return false;

  uint64_t count = 0;
  ifs.read(reinterpret_cast<char *>(&count), sizeof(count));
  if (!ifs) return false;

  decltype(data) loaded;
  loaded.reserve(count);
  for (uint64_t i = 0; i < count; ++i) {
    Pattern pattern;
    uint8_t value;
    ifs.read(reinterpret_cast<char *>(&pattern), sizeof(Pattern));
    ifs.read(reinterpret_cast<char *>(&value), sizeof(value));
    if (!ifs) return false;
    loaded.emplace(pattern, value);
  }

  data = std::move(loaded);
  return true;
}
