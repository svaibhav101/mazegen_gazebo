// Standalone unit test for ParseMazeFile. 
// Writes small mazes to a temp file, parses them, and
// checks the resulting Maze struct. Returns non-zero on the first failure.

#include "maze_parse.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using mazegen_plugin::Maze;
using mazegen_plugin::ParseMazeFile;

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

  // 3x3 maze with an  start (SW corner) and goal (center).
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

    // hWall is indexed [col][0..rows], vWall [0..cols][row].
    Check(m.hWall.size() == 3, "hWall has cols columns");
    Check(m.hWall[0].size() == 4, "hWall column has rows+1 entries");
    Check(m.vWall.size() == 4, "vWall has cols+1 columns");
    Check(m.vWall[0].size() == 3, "vWall column has rows entries");

    // Start 'S' is in the bottom-left cell -> (col 0, row 0).
    Check(m.startCol == 0 && m.startRow == 0, "start cell is (0,0)");

    // Goal 'G' is in the center cell -> (col 1, row 1).
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

  void TestBlankLineTrimmingAndCrlf()
  {
    // Leading/trailing blank lines and Windows CRLF endings should be
    // tolerated and produce the same maze as the clean version.
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

  // 2x2 maze whose entire interior is a 2x2 goal block (classic micromouse
  // goal), with the start in the SW cell. Exercises multi-cell goal detection.
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

    // Three distinct goal cells: (1,0), (0,1), (1,1). Each must appear once.
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

  void TestRejectsMalformed()
  {
    bool threw = false;
    try
    {
      const auto path = WriteTemp("o---o\n|   |\n");  // even line count
      ParseMazeFile(path);
      std::filesystem::remove(path);
    }
    catch (const std::exception &)
    {
      threw = true;
    }
    Check(threw, "even line count is rejected");

    threw = false;
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
}  // namespace

int main()
{
  TestDimensionsAndMarkers();
  TestBlankLineTrimmingAndCrlf();
  TestGoalBlock();
  TestRejectsMalformed();

  if (g_failures == 0)
  {
    std::cout << "All maze_parse tests passed.\n";
    return 0;
  }
  std::cerr << g_failures << " maze_parse test(s) failed.\n";
  return 1;
}
