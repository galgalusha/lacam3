#pragma once

#include "graph.hpp"
#include "instance.hpp"

#include <iostream>
#include <unordered_map>
#include <vector>

// ANSI palette shared by drawing helpers
namespace drawing_detail {
  static constexpr const char* COLORS[] = {
    "\033[31m", "\033[32m", "\033[33m", "\033[34m",
    "\033[35m", "\033[36m", "\033[91m", "\033[92m",
    "\033[93m", "\033[94m", "\033[95m", "\033[96m",
  };
  static constexpr int NUM_COLORS = 12;
  static constexpr const char* RESET  = "\033[0m";
  static constexpr const char* CURSOR = "\033[7m";  // inverse video
  static constexpr const char* YELLOW = "\033[33m";
  static constexpr const char* CYAN   = "\033[36m";
  static constexpr const char* BOLD_W = "\033[1;37m";
}

// Optional extras for draw_instance
struct DrawExtra {
  int  cursor_index = -1;   // U-array index to show as cursor (-1 = none)
  bool goals_only   = false; // suppress start markers
};

inline void draw_instance(const Instance* ins,
                          const DrawExtra& extra = {},
                          std::ostream& out = std::cout)
{
  using namespace drawing_detail;
  const int W = ins->G->width;
  const int H = ins->G->height;

  std::unordered_map<int, std::pair<int, char>> marks;
  for (size_t i = 0; i < ins->N; ++i) {
    if (!extra.goals_only && ins->starts[i])
      marks[ins->starts[i]->index] = {(int)i, 's'};
    if (ins->goals[i])
      marks[ins->goals[i]->index] = {(int)i, 'g'};
  }

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int idx = W * y + x;
      if (idx == extra.cursor_index) {
        out << CURSOR << '*' << RESET;
      } else {
        auto it = marks.find(idx);
        if (it != marks.end()) {
          out << COLORS[it->second.first % NUM_COLORS] << it->second.second << RESET;
        } else {
          out << (ins->G->U[idx] ? '.' : '#');
        }
      }
    }
    out << '\n';
  }
}

// Low-level map renderer used by the interactive explorer (no Instance needed).
// bg_goals:    U-indices shown as dim 'g' in cursor mode; pass empty in DH mode
// sel_goal_u:  selected agent's goal U-index (bold white 'G'), -1 = none
// sel_start_u: selected agent's start U-index (bold white 'S'), -1 = none
// agents:      matched agents as (goal_u, start_u); each auto-colored from palette
// cursor_u:    cursor cell U-index (-1 = none)
inline void draw_map(const Graph* G,
                     const std::vector<int>& bg_goals,
                     int sel_goal_u,
                     int sel_start_u,
                     const std::vector<std::pair<int,int>>& agents,
                     int cursor_u = -1,
                     std::ostream& out = std::cout)
{
  using namespace drawing_detail;
  const int W = G->width;
  const int H = G->height;

  struct Mark { const char* color; char ch; };
  std::unordered_map<int, Mark> marks;

  for (int u : bg_goals)
    marks[u] = {COLORS[4], 'g'};

  for (int i = 0; i < (int)agents.size(); ++i) {
    const char* col = COLORS[i % NUM_COLORS];
    auto [gu, su] = agents[i];
    if (gu >= 0) marks[gu]             = {col, 'g'};
    if (su >= 0) marks.insert_or_assign(su, Mark{col, 's'});
  }

  // Selected agent drawn on top of everything else
  if (sel_start_u >= 0) marks.insert_or_assign(sel_start_u, Mark{BOLD_W, 'S'});
  if (sel_goal_u  >= 0) marks.insert_or_assign(sel_goal_u,  Mark{BOLD_W, 'G'});

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int idx = W * y + x;
      // Selected agent always on top, even over the cursor
      if (idx == sel_goal_u) {
        out << BOLD_W << 'G' << RESET;
      } else if (idx == sel_start_u) {
        out << BOLD_W << 'S' << RESET;
      } else if (idx == cursor_u) {
        out << CURSOR << '*' << RESET;
      } else {
        auto it = marks.find(idx);
        if (it != marks.end())
          out << it->second.color << it->second.ch << RESET;
        else
          out << (G->U[idx] ? '.' : '#');
      }
    }
    out << '\n';
  }
}
