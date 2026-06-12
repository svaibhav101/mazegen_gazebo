#ifndef MAZE_SPAWN_UTILS_H_
#define MAZE_SPAWN_UTILS_H_

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "maze_params.h"
#include "maze_parse.h"

namespace mazegen
{
  /// \brief Return the world-frame centre of cell (_col, _row), accounting for
  /// the maze origin and yaw rotation from Params.
  ignition::math::Vector3d CellCenter(int _col, int _row, const Params &_p);

  /// \brief Derive the world-frame spawn yaw from the first open side of the
  /// start cell, composed with the maze's own yaw. Priority: E->N→W→S.
  /// \p _dir is set to a human-readable cardinal label (or "none (fully enclosed)").
  double SpawnYaw(const Maze &_m, const Params &_p, std::string &_dir);

  /// \brief Print maze dimensions, spawn pose, and goal locations to stdout.
  void LogSpawnInfo(const Maze &_m, const Params &_p);

} // namespace mazegen

#endif /* MAZE_SPAWN_UTILS_H_ */
