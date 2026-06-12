#ifndef MAZE_PARAMS_H_
#define MAZE_PARAMS_H_

#include <string>

#include <ignition/math/Vector3.hh>
#include <sdf/Element.hh>

namespace mazegen
{
  /// \brief Tunable geometry and appearance for a spawned maze.
  ///
  /// Defaults follow the classic micromouse spec: 180 mm cells, 12 mm
  /// walls, 50 mm tall, white body with a red cap.
  struct Params
  {
    std::string mazeFile;
    std::string modelName = "maze";
    double cellSize = 0.18;
    double wallThickness = 0.012;
    double wallHeight = 0.05;
    double postSize = 0.012;
    ignition::math::Vector3d origin{0.0, 0.0, 0.0};
    ignition::math::Vector3d rotation{0.0, 0.0, 0.0};
    ignition::math::Vector3d wallColor{1.0, 1.0, 1.0};
    ignition::math::Vector3d capColor{0.9, 0.05, 0.05};
  };

  std::string ResolveMazePath(const std::string &_path);

  double GetDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                   const std::string &_key, double _def);

  ignition::math::Vector3d GetVec3(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key,
      const ignition::math::Vector3d &_def);

  void GetPose(const std::shared_ptr<const sdf::Element> &_sdf,
               ignition::math::Vector3d &_xyz,
               ignition::math::Vector3d &_rpy);

  Params ReadSharedParams(const std::shared_ptr<const sdf::Element> &_sdf);

  Params ReadMazeBlock(const std::shared_ptr<const sdf::Element> &_maze,
                       const Params &_shared, std::size_t _index);

} // namespace mazegen

#endif /* MAZE_PARAMS_H_ */
