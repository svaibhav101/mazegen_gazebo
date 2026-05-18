#ifndef MAZE_PARSE_H_
#define MAZE_PARSE_H_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

/// \file maze_parse.h
/// \brief Parser for micromouseonline-format maze text files.

namespace mazegen_plugin
{
/// \brief A maze parsed from the micromouseonline text format.
struct Maze
{
  std::size_t cols = 0;  ///< Number of cell columns (along +x).
  std::size_t rows = 0;  ///< Number of cell rows (along +y).

  std::vector<std::vector<bool>> hWall;

  std::vector<std::vector<bool>> vWall;

  int startCol = 0;  ///< Column of the cell marked 'S' (0 if absent).
  int startRow = 0;  ///< Row of the cell marked 'S' (0 if absent).

  std::vector<std::pair<int, int>> goalCells;
};

Maze ParseMazeFile(const std::string &_path);
}  // namespace mazegen_plugin

#endif
