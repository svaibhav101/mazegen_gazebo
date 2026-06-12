#ifndef MAZE_PARSE_H_
#define MAZE_PARSE_H_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

/// \file maze_parse.h
/// \brief Parser for micromouseonline-format maze text files.

namespace mazegen
{
  /// \brief A maze parsed from the micromouseonline text format.
  ///
  /// Cells are addressed as '(col, row)', where 'row == 0' is the southern
  /// (bottom) edge and 'col == 0' the western (left) edge - matching the
  /// orientation of the source text file. Walls are stored on shared edges
  /// rather than per cell, so each interior wall appears exactly once.
  struct Maze
  {
    std::size_t cols = 0; ///< Number of cell columns (along +x).
    std::size_t rows = 0; ///< Number of cell rows (along +y).

    /// \brief Horizontal (east-west) wall edges, indexed 'hWall[col][r]'.
    ///
    /// 'hWall[col][r]' is 'true' when a wall runs along the latitude 'r',
    /// separating row 'r - 1' from row 'r'. The index 'r' ranges over
    /// '[0, rows]': 'r == 0' is the south outer edge and 'r == rows' the
    /// north outer edge.
    std::vector<std::vector<bool>> hWall;

    /// \brief Vertical (north-south) wall edges, indexed 'vWall[c][row]'.
    ///
    /// 'vWall[c][row]' is 'true' when a wall runs along the longitude 'c',
    /// separating column 'c - 1' from column 'c'. The index 'c' ranges over
    /// '[0, cols]': 'c == 0' is the west outer edge and 'c == cols' the east
    /// outer edge.
    std::vector<std::vector<bool>> vWall;

    int startCol = 0; ///< Column of the cell marked 'S' (0 if absent).
    int startRow = 0; ///< Row of the cell marked 'S' (0 if absent).

    /// \brief Cells marked 'G', as '(col, row)' pairs.
    ///
    /// A classic 16x16 micromouse goal is a 2x2 block, so this commonly holds
    /// up to four cells.
    std::vector<std::pair<int, int>> goalCells;
  };

  /// \brief Parse a micromouseonline-format maze file into a Maze.
  /// \param[in] _path Filesystem path to the maze text file.
  /// \return The parsed maze, with walls, start, and goal cells populated.
  /// \throws std::runtime_error if the file cannot be opened or is malformed.
  Maze ParseMazeFile(const std::string &_path);
} // namespace mazegen

#endif
