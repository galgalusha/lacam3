#pragma once

#include <cassert>

#include "graph.hpp"

enum Move {
  LEFT   = 0,
  RIGHT  = 1,
  UP     = 2,
  DOWN   = 3
};

const int NUM_OF_MOVES = 4;

Move get_move(Vertex* v1, Vertex* v2) {
  int dx = v2->x - v1->x;
  int dy = v2->y - v1->y;
  if (dx == -1 && dy == 0) return LEFT;
  if (dx == 1 && dy == 0) return RIGHT;
  if (dx == 0 && dy == -1) return UP;
  if (dx == 0 && dy == 1) return DOWN;
  throw std::runtime_error(
      "FATAL: v1 and v2 must be adjacent spatial moves. dx=" +
      std::to_string(dx) + ", dy=" + std::to_string(dy));
}
