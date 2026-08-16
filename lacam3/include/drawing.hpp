#pragma once

#include "instance.hpp"

#include <iostream>
#include <unordered_map>

inline void draw_instance(const Instance* ins, std::ostream& out = std::cout)
{
  // ANSI foreground colors (16-color, works on most terminals)
  static constexpr const char* COLORS[] = {
    "\033[31m", "\033[32m", "\033[33m", "\033[34m",
    "\033[35m", "\033[36m", "\033[91m", "\033[92m",
    "\033[93m", "\033[94m", "\033[95m", "\033[96m",
  };
  static constexpr int NUM_COLORS = 12;
  static constexpr const char* RESET = "\033[0m";

  const int W = ins->G->width;
  const int H = ins->G->height;

  // map index -> (agent_index, 's' or 'g')
  std::unordered_map<int, std::pair<int, char>> marks;
  for (size_t i = 0; i < ins->N; ++i) {
    if (ins->starts[i]) marks[ins->starts[i]->index] = {(int)i, 's'};
    if (ins->goals[i])  marks[ins->goals[i]->index]  = {(int)i, 'g'};
  }

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      int idx = W * y + x;
      auto it = marks.find(idx);
      if (it != marks.end()) {
        int agent = it->second.first;
        char ch   = it->second.second;
        out << COLORS[agent % NUM_COLORS] << ch << RESET;
      } else {
        out << (ins->G->U[idx] ? '.' : '#');
      }
    }
    out << '\n';
  }
}
