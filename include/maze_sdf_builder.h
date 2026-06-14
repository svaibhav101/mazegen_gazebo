#ifndef MAZE_SDF_BUILDER_H_
#define MAZE_SDF_BUILDER_H_

#include <string>

#include "maze_params.h"
#include "maze_parse.h"

/// \file maze_sdf_builder.h
/// \brief SDF string generator for a maze model.

namespace mazegen
{
  /// \brief Build the complete SDF document string for a maze model.
  ///
  /// Produces one static model with three links:
  ///
  /// - **walls**: collision + visual boxes for every wall segment.
  ///   Collinear wall segments that share the same latitude (horizontal) or
  ///   longitude (vertical) are merged greedily into a single box to reduce
  ///   the entity count.  Isolated corner posts that are not absorbed by any
  ///   merged bar are emitted separately.
  ///
  /// - **markers**: visual-only floor tiles for the start cell (blue) and
  ///   each goal cell (orange).  No collision geometry.
  ///
  /// - **tiles**: static coloured floor tiles from Params::tileColors, which
  ///   are baked in from \<tile_color\> elements in the plugin SDF.  For
  ///   runtime path visualization publish to the /marker_array topic instead.
  ///
  /// \param[in] _maze Parsed maze structure (walls, start, goals).
  /// \param[in] _p    Geometry and appearance parameters.
  /// \return SDF XML string ready to be written to a file or passed to /create.
  std::string BuildMazeSdf(const Maze &_maze, const Params &_p);

} // namespace mazegen

#endif /* MAZE_SDF_BUILDER_H_ */
