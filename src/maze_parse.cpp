#include "maze_parse.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace mazegen_plugin
{
  namespace
  {
    /// \brief Character columns spanned by one maze cell in the text layout.
    ///
    /// micromouseonline ASCII alternates post rows and wall rows:
    /// \code
    ///   o---o---o---o   <- post row
    ///   |   |       |   <- wall row
    /// \endcode
    /// Each cell occupies four character columns: the post sits at column
    /// '4 * c', and the wall body spans '4 * c + 1 .. 4 * c + 3'.
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
      // Tolerate Windows line endings by dropping a trailing carriage return.
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      lines.push_back(std::move(line));
    }

    // Strip leading and trailing blank lines.
    while (!lines.empty() && lines.front().find_first_not_of(" \t") ==
                                 std::string::npos)
      lines.erase(lines.begin());
    while (!lines.empty() && lines.back().find_first_not_of(" \t") ==
                                 std::string::npos)
      lines.pop_back();

    if (lines.size() < 3 || (lines.size() % 2) == 0)
      throw std::runtime_error("maze_parse: expected an odd number of "
                               "non-blank lines (>=3), got " +
                               std::to_string(lines.size()));

    // The number of posts on the header row is one more than the column count.
    std::size_t postCount = 0;
    for (char c : lines.front())
      if (c == 'o')
        ++postCount;
    if (postCount < 2)
      throw std::runtime_error("maze_parse: cannot count posts in header row");

    Maze m;
    m.cols = postCount - 1;
    m.rows = (lines.size() - 1) / 2;

    // Edge grids: hWall has (rows + 1) latitudes, vWall has (cols + 1)
    // longitudes.
    m.hWall.assign(m.cols, std::vector<bool>(m.rows + 1, false));
    m.vWall.assign(m.cols + 1, std::vector<bool>(m.rows, false));

    // Read a character, treating positions past the end of a (short) line as
    // blank rather than out-of-range.
    auto charAt = [](const std::string &s, std::size_t i) -> char
    {
      return i < s.size() ? s[i] : ' ';
    };

    // Text rows run north-to-south, so tr == 0 is the northern edge. Even
    // rows carry horizontal walls; odd rows carry vertical walls and markers.
    for (std::size_t tr = 0; tr < lines.size(); ++tr)
    {
      const std::string &L = lines[tr];
      if ((tr % 2) == 0)
      {
        // Post row, mapping to latitude r: tr == 0 is the north outer edge
        // (r == rows) and the last row is the south outer edge (r == 0).
        std::size_t r = m.rows - (tr / 2);
        for (std::size_t c = 0; c < m.cols; ++c)
        {
          // The wall body occupies columns 4c+1..4c+3; sample its midpoint.
          char mid = charAt(L, kCellWidth * c + 2);
          m.hWall[c][r] = (mid == '-');
        }
      }
      else
      {
        // Wall row, mapping to cell row 'row': tr == 1 is the top-most cell
        // row (row == rows - 1).
        std::size_t row = m.rows - 1 - (tr / 2);
        for (std::size_t c = 0; c <= m.cols; ++c)
        {
          char ch = charAt(L, kCellWidth * c);
          m.vWall[c][row] = (ch == '|');
        }
        // Scan the cell interiors of this row for 'S' and 'G' markers.
        for (std::size_t c = 0; c < m.cols; ++c)
        {
          for (std::size_t k = 1; k < kCellWidth; ++k)
          {
            char cell = charAt(L, kCellWidth * c + k);
            if (cell == 'S')
            {
              m.startCol = static_cast<int>(c);
              m.startRow = static_cast<int>(row);
            }
            else if (cell == 'G')
            {
              const auto cc = static_cast<int>(c);
              const auto rr = static_cast<int>(row);
              if (m.goalCells.empty() ||
                  m.goalCells.back() != std::make_pair(cc, rr))
                m.goalCells.emplace_back(cc, rr);
            }
          }
        }
      }
    }

    return m;
  }
} // namespace mazegen_plugin
