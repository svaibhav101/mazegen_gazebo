#ifndef MAZEGEN_PLUGIN_H_
#define MAZEGEN_PLUGIN_H_

#include <string>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

/// \file mazegen_plugin.h
/// \brief Public header for the `MazegenPlugin` Ignition Gazebo plugin.

/// \namespace mazegen
/// \brief All public symbols exposed by the mazegen plugin.
namespace mazegen
{
  /// \brief World system plugin for Ignition Gazebo Fortress that builds one
  /// or more micromouse-style mazes from text files (micromouseonline format).
  ///
  /// Single-maze usage:
  /// \code
  /// <plugin filename="libmazegen_plugin.so"
  ///         name="mazegen::MazegenPlugin">
  ///   <maze_file>mazes/japan2007ef.txt</maze_file>
  ///   <model_name>japan2007ef</model_name>
  ///   <cell_size>0.18</cell_size>
  ///   <wall_thickness>0.012</wall_thickness>
  ///   <wall_height>0.05</wall_height>
  ///   <wall_color>1 1 1</wall_color>
  ///   <cap_color>0.9 0.05 0.05</cap_color>
  ///   <origin>0.0 0.0 0.0 0.0 0.0 0.0</origin>
  /// </plugin>
  /// \endcode
  ///
  /// Multi-maze usage (one <maze> block per maze):
  /// \code
  /// <plugin filename="libmazegen_plugin.so"
  ///         name="mazegen::MazegenPlugin">
  ///   <!-- Shared geometry/colour params (all optional) -->
  ///   <cell_size>0.18</cell_size>
  ///   <wall_color>1 1 1</wall_color>
  ///   <cap_color>0.9 0.05 0.05</cap_color>
  ///
  ///   <maze>
  ///     <file>mazes/alljapan-001-1980.txt</file>
  ///     <model_name>japan1980</model_name>   <!-- optional, default: maze_0 -->
  ///     <origin>0 0 0 0 0 0</origin>
  ///   </maze>
  ///   <maze>
  ///     <file>mazes/allamerica2013.txt</file>
  ///     <model_name>america2013</model_name>
  ///     <origin>5 0 0 0 0 0</origin>
  ///   </maze>
  /// </plugin>
  /// \endcode
  ///
  /// For each loaded maze the plugin advertises two persistent transport services:
  ///
  ///   /mazegen/<model_name>/spawn_pose  -- ignition::msgs::Pose
  ///     Center of the start cell in world coordinates; yaw from the first
  ///     open side of the start cell (east -> north -> west -> south).
  ///
  ///   /mazegen/<model_name>/goal_poses  -- ignition::msgs::Pose_V
  ///     One Pose per goal cell (position only, orientation identity),
  ///     in the order they appear in the maze file.
  ///
  class MazegenPlugin
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
  public:
    MazegenPlugin() = default;
    ~MazegenPlugin() override = default;

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    /// \brief Spawns all pending mazes on the first simulation tick, when the
    /// world's /create transport service is guaranteed to be reachable.
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

  private:
    /// \brief Per-maze runtime state.
    struct MazeInstance
    {
      std::string modelName;
      std::string pendingSdfFile;
      std::string createService;
      bool requested{false};
      bool done{false};
      unsigned pollTicks{0};
      ignition::math::Pose3d spawnPose;
      ignition::msgs::Pose_V goalPoses;
    };

    /// \brief All maze instances parsed from Configure().
    std::vector<MazeInstance> mazes_;

    /// \brief Set to true only when Configure() fully succeeds for >=1 maze.
    bool initialized_{false};

    /// \brief Transport node kept alive for the duration of the plugin.
    ignition::transport::Node node_;
  };
} // namespace mazegen

#endif /* MAZEGEN_PLUGIN_H_ */
