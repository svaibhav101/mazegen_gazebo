/// \file maze_parse.cpp
/// \brief Implementation of ParseMazeFile for micromouseonline-format mazes.

#include "maze_parse.h"

#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace mazegen
{
  namespace
  {
    /// Character columns spanned by one cell in the ASCII layout.
    ///
    /// micromouseonline format alternates post rows and wall rows:
    /// \code
    ///   o---o---o---o   <- post row  (even text line)
    ///   |   |       |   <- wall row  (odd text line)
    /// \endcode
    /// Each cell occupies four character columns: the left post sits at
    /// column (4 * c) and the wall body spans (4*c + 1 .. 4*c + 3).
    constexpr std::size_t kCellWidth = 4;
  } // namespace

  Maze ParseMazeFile(const std::string &_path)
  {
    std::ifstream in(_path);
    if (!in)
      throw std::runtime_error("maze_parse: cannot open '" + _path + "'");

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line))
    {
      // Tolerate Windows line endings.
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      lines.push_back(std::move(line));
    }

    // Skip leading and trailing blank lines with index arithmetic (O(1) per
    // line) instead of erasing from the front of the vector (O(n^2)).
    auto isBlank = [](const std::string &s)
    {
      return s.find_first_not_of(" \t") == std::string::npos;
    };
    std::size_t first = 0;
    while (first < lines.size() && isBlank(lines[first]))
      ++first;
    std::size_t last = lines.size();
    while (last > first && isBlank(lines[last - 1]))
      --last;
    const std::size_t nLines = last - first; // trimmed line count

    if (nLines < 3 || (nLines % 2) == 0)
      throw std::runtime_error("maze_parse: expected an odd number of "
                               "non-blank lines (>=3), got " +
                               std::to_string(nLines));

    // Count posts on the header row to determine column count.
    std::size_t postCount = 0;
    for (char c : lines[first])
      if (c == 'o')
        ++postCount;
    if (postCount < 2)
      throw std::runtime_error("maze_parse: cannot count posts in header row");

    Maze m;
    m.cols = postCount - 1;
    m.rows = (nLines - 1) / 2;

    m.hWall.assign(m.cols, std::vector<bool>(m.rows + 1, false));
    m.vWall.assign(m.cols + 1, std::vector<bool>(m.rows, false));

    // Return the character at position i, treating short lines as padded with spaces.
    auto charAt = [](const std::string &s, std::size_t i) -> char
    {
      return i < s.size() ? s[i] : ' ';
    };

    // Text rows run north-to-south (tr==0 is the northern edge).
    // Even rows carry horizontal walls; odd rows carry vertical walls + markers.
    bool hasStart = false;
    for (std::size_t tr = 0; tr < nLines; ++tr)
    {
      const std::string &L = lines[first + tr];
      if ((tr % 2) == 0)
      {
        // Post row: tr==0 → latitude r==rows (north outer edge).
        const std::size_t r = m.rows - (tr / 2);
        for (std::size_t c = 0; c < m.cols; ++c)
          m.hWall[c][r] = (charAt(L, kCellWidth * c + 2) == '-');
      }
      else
      {
        // Wall row: tr==1 → cell row == rows-1 (top-most cell row).
        const std::size_t row = m.rows - 1 - (tr / 2);
        for (std::size_t c = 0; c <= m.cols; ++c)
          m.vWall[c][row] = (charAt(L, kCellWidth * c) == '|');

        // Scan cell interiors for 'S' and 'G' markers.
        for (std::size_t c = 0; c < m.cols; ++c)
        {
          for (std::size_t k = 1; k < kCellWidth; ++k)
          {
            const char cell = charAt(L, kCellWidth * c + k);
            if (cell == 'S')
            {
              m.startCol = static_cast<int>(c);
              m.startRow = static_cast<int>(row);
              hasStart = true;
            }
            else if (cell == 'G')
            {
              const auto cc = static_cast<int>(c);
              const auto rr = static_cast<int>(row);
              // Avoid duplicating the same cell when 'G' appears multiple times
              // in the same cell interior (e.g. "GGG" spanning the body).
              if (m.goalCells.empty() ||
                  m.goalCells.back() != std::make_pair(cc, rr))
                m.goalCells.emplace_back(cc, rr);
            }
          }
        }
      }
    }

    if (!hasStart)
    {
      // Warn but do not throw: mazes without an 'S' marker are valid for
      // pure-traversal use cases; startCol/startRow default to (0, 0).
      fprintf(stderr,
              "maze_parse: warning: no 'S' marker found in '%s'; "
              "startCol/startRow default to (0, 0)\n",
              _path.c_str());
    }

    if (m.goalCells.empty())
    {
      fprintf(stderr,
              "maze_parse: warning: no 'G' marker found in '%s'\n",
              _path.c_str());
    }

    return m;
  }

} // namespace mazegen
