#ifndef MAZE_PARAMS_H_
#define MAZE_PARAMS_H_

#include <map>
#include <string>
#include <utility>

#include <ignition/math/Vector3.hh>
#include <sdf/Element.hh>

/// \file maze_params.h
/// \brief Maze geometry/appearance parameters and SDF parsing helpers.

namespace mazegen
{
  /// \brief Tunable geometry and appearance for a spawned maze.
  ///
  /// Defaults follow the classic micromouse spec: 180 mm cells, 12 mm walls,
  /// 50 mm tall, white body with a red cap.
  struct Params
  {
    std::string mazeFile;           ///< Path to the maze text file.
    std::string modelName = "maze"; ///< Gazebo model name to register.
    double cellSize = 0.18;         ///< Cell side length in metres.
    double wallThickness = 0.012;   ///< Wall thickness in metres.
    double wallHeight = 0.05;       ///< Wall height in metres.
    double postSize = 0.012;        ///< Corner post side length in metres.

    ignition::math::Vector3d origin{0.0, 0.0, 0.0};     ///< World-frame XYZ offset.
    ignition::math::Vector3d rotation{0.0, 0.0, 0.0};   ///< Roll/pitch/yaw in radians.
    ignition::math::Vector3d wallColor{1.0, 1.0, 1.0};  ///< Wall body RGB colour.
    ignition::math::Vector3d capColor{0.9, 0.05, 0.05}; ///< Wall cap RGB colour.

    /// \brief Per-cell floor tile colours baked in at load time.
    ///
    /// Populated from \<tile_color x="col" y="row"\>R G B\</tile_color\> in SDF.
    /// For runtime path visualization use the /marker_array topic instead.
    std::map<std::pair<int, int>, ignition::math::Vector3d> tileColors;
  };

  /// \brief Resolve a maze file path, falling back to IGN_GAZEBO_RESOURCE_PATH.
  ///
  /// \param[in] _path Raw path from SDF (may be absolute or resource-relative).
  /// \return Resolved absolute path, or \p _path unchanged if not found.
  std::string ResolveMazePath(const std::string &_path);

  /// \brief Read a double from an SDF element, returning a default if absent.
  ///
  /// \param[in] _sdf SDF element to query.
  /// \param[in] _key Child element name.
  /// \param[in] _def Value to return when the element is missing.
  /// \return Parsed double, or \p _def.
  double GetDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                   const std::string &_key, double _def);

  /// \brief Read a Vector3d from an SDF element, returning a default if absent.
  ///
  /// \param[in] _sdf SDF element to query.
  /// \param[in] _key Child element name.
  /// \param[in] _def Value to return when the element is missing.
  /// \return Parsed Vector3d, or \p _def.
  ignition::math::Vector3d GetVec3(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key,
      const ignition::math::Vector3d &_def);

  /// \brief Parse an \<origin\> element into separate XYZ and RPY vectors.
  ///
  /// The element value must contain six whitespace-separated doubles
  /// (x y z roll pitch yaw). Missing or extra values are silently ignored.
  ///
  /// \param[in]  _sdf  SDF element that may contain an \<origin\> child.
  /// \param[out] _xyz  Receives the translation (x, y, z).
  /// \param[out] _rpy  Receives the rotation (roll, pitch, yaw) in radians.
  void GetPose(const std::shared_ptr<const sdf::Element> &_sdf,
               ignition::math::Vector3d &_xyz,
               ignition::math::Vector3d &_rpy);

  /// \brief Read geometry and colour parameters shared across all \<maze\> blocks.
  ///
  /// Reads cell_size, wall_thickness, wall_height, post_size, wall_color,
  /// cap_color, and any \<tile_color\> children from the plugin SDF element.
  ///
  /// \param[in] _sdf Top-level plugin SDF element.
  /// \return Params populated with shared values; per-maze fields are at defaults.
  Params ReadSharedParams(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Read one \<maze\> child block, inheriting from shared params.
  ///
  /// Per-block elements (file, model_name, origin, and geometry overrides)
  /// take precedence over the shared values. Sets mazeFile to empty and logs
  /// an error if the required \<file\> element is missing.
  ///
  /// \param[in] _maze   The \<maze\> SDF element.
  /// \param[in] _shared Shared params to inherit from.
  /// \param[in] _index  Zero-based block index used for default model names.
  /// \return Params for this maze instance.
  Params ReadMazeBlock(const std::shared_ptr<const sdf::Element> &_maze,
                       const Params &_shared, std::size_t _index);

} // namespace mazegen

#endif /* MAZE_PARAMS_H_ */
