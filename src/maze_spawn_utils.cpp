/// \file maze_spawn_utils.cpp
/// \brief Implementation of CellCenter, SpawnYaw, and LogSpawnInfo.

#include "maze_spawn_utils.h"

#include <cmath>
#include <iostream>

#include <ignition/common/Console.hh>
#include <ignition/math/Vector3.hh>

namespace mazegen
{
  ignition::math::Vector3d CellCenter(int _col, int _row, const Params &_p)
  {
    // Rotate the local cell centre by the maze yaw, then translate.
    const double lx = (_col + 0.5) * _p.cellSize;
    const double ly = (_row + 0.5) * _p.cellSize;
    const double cosY = std::cos(_p.rotation.Z());
    const double sinY = std::sin(_p.rotation.Z());
    return {_p.origin.X() + cosY * lx - sinY * ly,
            _p.origin.Y() + sinY * lx + cosY * ly,
            _p.origin.Z()};
  }

  double SpawnYaw(const Maze &_m, const Params &_p, std::string &_dir)
  {
    const int c = _m.startCol;
    const int r = _m.startRow;

    const bool wallN = _m.hWall[c][r + 1];
    const bool wallS = _m.hWall[c][r];
    const bool wallW = _m.vWall[c][r];
    const bool wallE = _m.vWall[c + 1][r];

    // Face the first open side in priority order: East → North → West → South.
    double localYaw;
    bool enclosed = false;

    if (!wallE)
      localYaw = 0.0;
    else if (!wallN)
      localYaw = M_PI_2;
    else if (!wallW)
      localYaw = M_PI;
    else if (!wallS)
      localYaw = -M_PI_2;
    else
    {
      ignwarn << "MazegenPlugin: start cell is walled on all four sides; "
                 "defaulting mouse yaw to maze orientation."
              << std::endl;
      localYaw = 0.0;
      enclosed = true;
    }

    const double worldYaw = localYaw + _p.rotation.Z();

    if (enclosed)
    {
      _dir = "none (fully enclosed)";
    }
    else
    {
      // Normalise worldYaw to (-π, π] and map to a cardinal label.
      double a = std::fmod(worldYaw, 2.0 * M_PI);
      if (a > M_PI)
        a -= 2.0 * M_PI;
      if (a <= -M_PI)
        a += 2.0 * M_PI;

      const double absA = std::abs(a);
      if (absA <= M_PI_4)
        _dir = "east";
      else if (absA >= 3.0 * M_PI_4)
        _dir = "west";
      else if (a > 0.0)
        _dir = "north";
      else
        _dir = "south";
    }

    return worldYaw;
  }

  void LogSpawnInfo(const Maze &_m, const Params &_p)
  {
    const std::string prefix = "[MazegenPlugin/" + _p.modelName + "] ";

    std::cout << prefix << "loaded " << _m.cols << "x" << _m.rows
              << " maze" << std::endl;

    std::string dir;
    const double yaw = SpawnYaw(_m, _p, dir);
    const auto start = CellCenter(_m.startCol, _m.startRow, _p);

    std::cout << prefix << "spawn pose:"
              << " x=" << start.X()
              << " y=" << start.Y()
              << " z=" << start.Z()
              << " yaw=" << yaw << " rad"
              << " (facing " << dir << ")" << std::endl;

    std::cout << prefix << "start cell:"
              << " col=" << _m.startCol
              << " row=" << _m.startRow << std::endl;

    for (std::size_t i = 0; i < _m.goalCells.size(); ++i)
    {
      const auto g = CellCenter(_m.goalCells[i].first,
                                _m.goalCells[i].second, _p);
      std::cout << prefix << "goal " << i
                << ": col=" << _m.goalCells[i].first
                << " row=" << _m.goalCells[i].second
                << " x=" << g.X()
                << " y=" << g.Y() << std::endl;
    }
  }

} // namespace mazegen
