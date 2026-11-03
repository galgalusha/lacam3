#include <dist_table.hpp>
#include <graph.hpp>
#include <instance.hpp>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

static const std::vector<std::string> GRID = {
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

enum Key { K_NONE, K_UP, K_DOWN, K_LEFT, K_RIGHT, K_SPACE, K_QUIT };

static Key read_key()
{
  char c;
  if (read(STDIN_FILENO, &c, 1) != 1) return K_QUIT;
  if (c == ' ') return K_SPACE;
  if (c == 'q' || c == 'Q' || c == 3) return K_QUIT;
  if (c == 27) {
    char seq[2];
    if (read(STDIN_FILENO, &seq[0], 1) != 1) return K_QUIT;
    if (read(STDIN_FILENO, &seq[1], 1) != 1) return K_NONE;
    if (seq[0] == '[') {
      switch (seq[1]) {
        case 'A': return K_UP;
        case 'B': return K_DOWN;
        case 'C': return K_RIGHT;
        case 'D': return K_LEFT;
      }
    }
  }
  return K_NONE;
}

int main()
{
  Graph *G = new Graph(GRID);
  Config goals = G->V;
  Instance *ins = new Instance(G, goals, goals, goals.size());
  DistTable D(ins);  // agent i == goal G->V[i]

  const int W = G->width, H = G->height;
  int cx = 0, cy = 0;
  Vertex *marked = nullptr;

  termios orig;
  tcgetattr(STDIN_FILENO, &orig);
  termios raw = orig;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

  while (true) {
    Vertex *hover = G->U[cy * W + cx];
    std::string out = "\033[2J\033[H";
    out += "Arrows: move | Space: mark passable cell | q: quit\r\n\r\n";
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        Vertex *v = G->U[y * W + x];
        char ch = v == nullptr ? '@' : '.';
        if (v != nullptr && v == marked) ch = 'M';
        const bool cur = (x == cx && y == cy);
        if (cur) out += "\033[7m";
        out += ch;
        if (cur) out += "\033[0m";
        out += ' ';
      }
      out += "\r\n";
    }
    out += "\r\nHover : (" + std::to_string(cx) + ", " + std::to_string(cy) + ")" +
           (hover ? "" : " [obstacle]") + "\r\n";
    if (marked == nullptr) {
      out += "Marked: none\r\n";
    } else {
      out += "Marked: (" + std::to_string(marked->x) + ", " +
             std::to_string(marked->y) + ")\r\n";
      if (hover == nullptr) {
        out += "D: -\r\n";
      } else {
        const int d = D.get(marked->id, hover);
        out += "D(hover -> marked): " +
               (d >= (int)G->V.size() ? std::string("unreachable") : std::to_string(d)) +
               "\r\n";
      }
    }
    std::cout << out << std::flush;

    switch (read_key()) {
      case K_UP: if (cy > 0) --cy; break;
      case K_DOWN: if (cy < H - 1) ++cy; break;
      case K_LEFT: if (cx > 0) --cx; break;
      case K_RIGHT: if (cx < W - 1) ++cx; break;
      case K_SPACE: if (hover != nullptr) marked = hover; break;
      case K_QUIT:
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig);
        std::cout << std::endl;
        return 0;
      default: break;
    }
  }
}
