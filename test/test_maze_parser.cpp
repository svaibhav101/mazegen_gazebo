// Standalone unit tests for ParseMazeFile.
// Writes small mazes to temp files, parses them, and checks the resulting Maze
// struct. Returns non-zero on the first failure.

#include "maze_parse.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using mazegen::Maze;
using mazegen::ParseMazeFile;

namespace
{
  int g_failures = 0;

  void Check(bool cond, const std::string &what)
  {
    if (!cond)
    {
      std::cerr << "FAIL: " << what << "\n";
      ++g_failures;
    }
  }

  // Write text to a unique temp file and return its path.
  std::string WriteTemp(const std::string &text)
  {
    static int n = 0;
    auto path = std::filesystem::temp_directory_path() /
                ("maze_parser_test_" + std::to_string(n++) + ".txt");
    std::ofstream f(path);
    f << text;
    return path.string();
  }

  // ---------------------------------------------------------------------------
  // 3x3 maze with a start (SW corner) and a goal (center).
  //   o---o---o---o
  //   |           |
  //   o   o---o   o
  //   |   | G |   |
  //   o   o---o   o
  //   | S |       |
  //   o---o---o---o
  const char *kMaze3x3 =
      "o---o---o---o\n"
      "|           |\n"
      "o   o---o   o\n"
      "|   | G |   |\n"
      "o   o---o   o\n"
      "| S |       |\n"
      "o---o---o---o\n";

  void TestDimensionsAndMarkers()
  {
    const auto path = WriteTemp(kMaze3x3);
    Maze m = ParseMazeFile(path);

    Check(m.cols == 3, "cols == 3");
    Check(m.rows == 3, "rows == 3");

    Check(m.hWall.size() == 3, "hWall has cols columns");
    Check(m.hWall[0].size() == 4, "hWall column has rows+1 entries");
    Check(m.vWall.size() == 4, "vWall has cols+1 columns");
    Check(m.vWall[0].size() == 3, "vWall column has rows entries");

    Check(m.startCol == 0 && m.startRow == 0, "start cell is (0,0)");

    Check(m.goalCells.size() == 1, "exactly one goal cell");
    if (!m.goalCells.empty())
      Check(m.goalCells[0] == std::make_pair(1, 1), "goal cell is (1,1)");

    // Outer boundary walls must all be present.
    for (std::size_t c = 0; c < m.cols; ++c)
    {
      Check(m.hWall[c][0], "south outer wall present");
      Check(m.hWall[c][m.rows], "north outer wall present");
    }
    for (std::size_t r = 0; r < m.rows; ++r)
    {
      Check(m.vWall[0][r], "west outer wall present");
      Check(m.vWall[m.cols][r], "east outer wall present");
    }

    std::filesystem::remove(path);
  }

  // ---------------------------------------------------------------------------
  void TestBlankLineTrimmingAndCrlf()
  {
    // Leading/trailing blank lines and Windows CRLF endings should be tolerated
    // and produce the same maze as the clean version.
    std::string crlf;
    crlf += "\n\n";
    for (const char *p = kMaze3x3; *p; ++p)
    {
      if (*p == '\n')
        crlf += '\r';
      crlf += *p;
    }
    crlf += "\n  \n";

    const auto path = WriteTemp(crlf);
    Maze m = ParseMazeFile(path);
    Check(m.cols == 3 && m.rows == 3, "CRLF/blank-padded maze parses to 3x3");
    Check(m.startCol == 0 && m.startRow == 0, "start survives CRLF/trim");
    std::filesystem::remove(path);
  }

  // ---------------------------------------------------------------------------
  // 2x2 maze whose entire interior is a 2x2 goal block.
  //   o---o---o
  //   | G   G |
  //   o   o   o
  //   | S   G |
  //   o---o---o
  const char *kMaze2x2Goal =
      "o---o---o\n"
      "| G   G |\n"
      "o   o   o\n"
      "| S   G |\n"
      "o---o---o\n";

  void TestGoalBlock()
  {
    const auto path = WriteTemp(kMaze2x2Goal);
    Maze m = ParseMazeFile(path);

    Check(m.cols == 2 && m.rows == 2, "goal-block maze is 2x2");
    Check(m.startCol == 0 && m.startRow == 0, "start is SW cell (0,0)");

    Check(m.goalCells.size() == 3, "exactly three goal cells, no duplicates");
    auto has = [&](int c, int r)
    {
      for (const auto &g : m.goalCells)
        if (g == std::make_pair(c, r))
          return true;
      return false;
    };
    Check(has(1, 0) && has(0, 1) && has(1, 1), "all three goal cells found");

    std::filesystem::remove(path);
  }

  // ---------------------------------------------------------------------------
  void TestRejectsMalformed()
  {
    // Even line count is rejected.
    {
      bool threw = false;
      try
      {
        const auto path = WriteTemp("o---o\n|   |\n");
        ParseMazeFile(path);
        std::filesystem::remove(path);
      }
      catch (const std::exception &)
      {
        threw = true;
      }
      Check(threw, "even line count is rejected");
    }

    // Missing file is rejected.
    {
      bool threw = false;
      try
      {
        ParseMazeFile("/nonexistent/path/to/maze.txt");
      }
      catch (const std::exception &)
      {
        threw = true;
      }
      Check(threw, "missing file is rejected");
    }

    // Single line (< 3 non-blank lines) is rejected.
    {
      bool threw = false;
      try
      {
        const auto path = WriteTemp("o---o---o\n");
        ParseMazeFile(path);
        std::filesystem::remove(path);
      }
      catch (const std::exception &)
      {
        threw = true;
      }
      Check(threw, "single line (< 3) is rejected");
    }

    // Fewer than 2 posts in header is rejected.
    {
      bool threw = false;
      try
      {
        // Header has only one 'o' -> postCount < 2.
        const auto path = WriteTemp("o \n| \no \n");
        ParseMazeFile(path);
        std::filesystem::remove(path);
      }
      catch (const std::exception &)
      {
        threw = true;
      }
      Check(threw, "header with < 2 posts is rejected");
    }
  }

  // ---------------------------------------------------------------------------
  // Maze with no 'S' marker: should parse successfully, startCol/startRow
  // default to (0,0), and a warning should be emitted (not testable here,
  // but we verify no exception is thrown and defaults are correct).
  //   o---o---o
  //   |   | G |
  //   o   o   o
  //   |       |
  //   o---o---o
  void TestNoStartMarker()
  {
    const char *src =
        "o---o---o\n"
        "|   | G |\n"
        "o   o   o\n"
        "|       |\n"
        "o---o---o\n";

    const auto path = WriteTemp(src);
    bool threw = false;
    Maze m;
    try
    {
      m = ParseMazeFile(path);
    }
    catch (const std::exception &)
    {
      threw = true;
    }

    Check(!threw, "no-S maze does not throw");
    Check(m.cols == 2 && m.rows == 2, "no-S maze has correct dimensions");
    // Default start is (0,0).
    Check(m.startCol == 0 && m.startRow == 0,
          "no-S maze defaults startCol/startRow to (0,0)");
    // Goal should still be found.
    Check(m.goalCells.size() == 1, "no-S maze still finds goal");

    std::filesystem::remove(path);
  }

  // ---------------------------------------------------------------------------
  // Maze with no 'G' marker: should parse successfully with an empty goalCells.
  //   o---o---o
  //   |       |
  //   o   o   o
  //   | S     |
  //   o---o---o
  void TestNoGoalMarker()
  {
    const char *src =
        "o---o---o\n"
        "|       |\n"
        "o   o   o\n"
        "| S     |\n"
        "o---o---o\n";

    const auto path = WriteTemp(src);
    bool threw = false;
    Maze m;
    try
    {
      m = ParseMazeFile(path);
    }
    catch (const std::exception &)
    {
      threw = true;
    }

    Check(!threw, "no-G maze does not throw");
    Check(m.goalCells.empty(), "no-G maze has empty goalCells");
    Check(m.startCol == 0 && m.startRow == 0, "start still found without G");

    std::filesystem::remove(path);
  }

  // ---------------------------------------------------------------------------
  // Non-square maze (4 columns x 2 rows).
  //   o---o---o---o---o
  //   | S |       | G |
  //   o   o---o   o   o
  //   |           |   |
  //   o---o---o---o---o
  void TestNonSquareMaze()
  {
    const char *src =
        "o---o---o---o---o\n"
        "| S |       | G |\n"
        "o   o---o   o   o\n"
        "|           |   |\n"
        "o---o---o---o---o\n";

    const auto path = WriteTemp(src);
    bool threw = false;
    Maze m;
    try
    {
      m = ParseMazeFile(path);
    }
    catch (const std::exception &)
    {
      threw = true;
    }

    Check(!threw, "non-square maze (4x2) does not throw");
    Check(m.cols == 4, "non-square maze has 4 cols");
    Check(m.rows == 2, "non-square maze has 2 rows");
    Check(m.hWall.size() == 4, "hWall outer dim == cols");
    Check(m.hWall[0].size() == 3, "hWall inner dim == rows+1");
    Check(m.vWall.size() == 5, "vWall outer dim == cols+1");
    Check(m.vWall[0].size() == 2, "vWall inner dim == rows");

    // Start: col 0, row 1 (top-left cell).
    Check(m.startCol == 0 && m.startRow == 1, "start is (0,1)");
    // Goal: col 3, row 1 (top-right cell).
    Check(m.goalCells.size() == 1, "one goal cell");
    if (!m.goalCells.empty())
      Check(m.goalCells[0] == std::make_pair(3, 1), "goal is (3,1)");

    std::filesystem::remove(path);
  }

  // ---------------------------------------------------------------------------
  // Minimal 1x1 maze (smallest valid odd-line-count structure): 3 lines.
  //   o---o
  //   | S |
  //   o---o
  void TestMinimalMaze()
  {
    const char *src =
        "o---o\n"
        "| S |\n"
        "o---o\n";

    const auto path = WriteTemp(src);
    bool threw = false;
    Maze m;
    try
    {
      m = ParseMazeFile(path);
    }
    catch (const std::exception &)
    {
      threw = true;
    }

    Check(!threw, "1x1 maze does not throw");
    Check(m.cols == 1 && m.rows == 1, "1x1 maze has 1 col, 1 row");
    Check(m.startCol == 0 && m.startRow == 0, "1x1 start is (0,0)");
    // All four outer walls must be set.
    Check(m.hWall[0][0] && m.hWall[0][1], "1x1 south and north outer walls");
    Check(m.vWall[0][0] && m.vWall[1][0], "1x1 west and east outer walls");

    std::filesystem::remove(path);
  }

} // namespace

int main()
{
  TestDimensionsAndMarkers();
  TestBlankLineTrimmingAndCrlf();
  TestGoalBlock();
  TestRejectsMalformed();
  TestNoStartMarker();
  TestNoGoalMarker();
  TestNonSquareMaze();
  TestMinimalMaze();

  if (g_failures == 0)
  {
    std::cout << "All maze_parse tests passed.\n";
    return 0;
  }
  std::cerr << g_failures << " maze_parse test(s) failed.\n";
  return 1;
}
