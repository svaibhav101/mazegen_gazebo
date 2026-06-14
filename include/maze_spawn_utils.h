#ifndef MAZE_SPAWN_UTILS_H_
#define MAZE_SPAWN_UTILS_H_

#include <string>

#include <ignition/math/Vector3.hh>

#include "maze_params.h"
#include "maze_parse.h"

/// \file maze_spawn_utils.h
/// \brief Utilities for computing robot spawn pose from a parsed maze.

namespace mazegen
{
  /// \brief Compute the world-frame centre of a maze cell.
  ///
  /// The local cell centre (in maze-frame coordinates) is rotated by the maze
  /// yaw angle from Params::rotation.Z() and then translated by Params::origin.
  ///
  /// \param[in] _col Column index of the cell (0 = west edge).
  /// \param[in] _row Row index of the cell (0 = south edge).
  /// \param[in] _p   Maze geometry and pose parameters.
  /// \return World-frame XYZ position of the cell centre (Z = origin Z).
  ignition::math::Vector3d CellCenter(int _col, int _row, const Params &_p);

  /// \brief Derive the world-frame spawn yaw from the start cell's open side.
  ///
  /// Checks the four walls of the start cell in priority order East→North→West→South
  /// and returns the yaw that faces into the first open direction.  The result
  /// is composed with the maze's own yaw (Params::rotation.Z()).
  ///
  /// \param[in]  _m   Parsed maze (walls and start cell).
  /// \param[in]  _p   Maze pose parameters.
  /// \param[out] _dir Human-readable cardinal label ("east", "north", "west",
  ///                  "south", or "none (fully enclosed)").
  /// \return World-frame spawn yaw angle in radians.
  double SpawnYaw(const Maze &_m, const Params &_p, std::string &_dir);

  /// \brief Print maze dimensions, robot spawn pose, and goal locations to stdout.
  ///
  /// Output is prefixed with [MazegenPlugin/<model_name>] for easy log filtering.
  ///
  /// \param[in] _m Parsed maze.
  /// \param[in] _p Maze geometry and pose parameters.
  void LogSpawnInfo(const Maze &_m, const Params &_p);

} // namespace mazegen

#endif /* MAZE_SPAWN_UTILS_H_ */
