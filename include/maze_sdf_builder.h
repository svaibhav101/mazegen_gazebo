#ifndef MAZE_SDF_BUILDER_H_
#define MAZE_SDF_BUILDER_H_

#include <string>

#include "maze_params.h"
#include "maze_parse.h"

namespace mazegen
{
  /// \brief Build the complete SDF document string for a maze model.
  ///
  /// Produces one static model with two links:
  ///   - 'walls': collision + visual boxes (collinear runs merged greedily).
  ///   - 'markers': visual-only floor tiles for start (blue) and goal (orange).
  std::string BuildMazeSdf(const Maze &_maze, const Params &_p);

} // namespace mazegen

#endif /* MAZE_SDF_BUILDER_H_ */
