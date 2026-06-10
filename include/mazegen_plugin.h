#ifndef MAZEGEN_PLUGIN_H_
#define MAZEGEN_PLUGIN_H_

#include <string>

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
  /// \brief World system plugin for Ignition Gazebo Fortress that builds a
  /// micromouse-style maze from a text file (micromouseonline format).
  ///
  /// Example SDF usage:
  /// \code
  /// <plugin filename="libmazegen_plugin.so"
  ///         name="mazegen::MazegenPlugin">
  ///   <maze_file>mazes/japan2007ef.txt</maze_file>
  ///   <model_name>japan2007ef</model_name>
  ///   <cell_size>0.18</cell_size>                <!-- meters -->
  ///   <wall_thickness>0.012</wall_thickness>     <!-- meters -->
  ///   <wall_height>0.05</wall_height>            <!-- meters -->
  ///   <wall_color>1 1 1</wall_color>             <!-- R G B, range [0, 1] -->
  ///   <cap_color>0.9 0.05 0.05</cap_color>       <!-- R G B, range [0, 1] -->
  ///   <origin>0.0 0.0 0.0 0.0 0.0 0.0</origin>   <!-- x y z (m)  roll pitch yaw (rad) -->
  /// </plugin>
  /// \endcode
  ///
  /// After the maze loads, the plugin advertises two persistent transport services:
  ///
  ///   /mazegen/<model_name>/spawn_pose  -- ignition::msgs::Pose
  ///     Center of the start cell in world coordinates; yaw from the first
  ///     open side of the start cell (east -> north -> west -> south).
  ///
  ///   /mazegen/<model_name>/goal_poses  -- ignition::msgs::Pose_V
  ///     One Pose per goal cell (position only, orientation identity),
  ///     in the order they appear in the maze file.
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

    /// \brief Spawns the maze on the first simulation tick, when the
    /// world's /create transport service is guaranteed to be reachable.
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

  private:
    /// \brief Path to the generated temp SDF file; empty once the file is deleted.
    std::string pendingSdfFile_;
    /// \brief Transport service name for entity creation.
    std::string createService_;
    /// \brief Name given to the spawned model (from <model_name>, default "maze").
    std::string modelName_{"maze"};
    /// \brief Set to true after the /create request has been sent.
    bool requested_{false};
    /// \brief Set to true after the temp file has been cleaned up.
    bool done_{false};
    /// \brief Set to true only when Configure() fully succeeds.
    bool initialized_{false};
    /// \brief Number of ticks spent polling for the spawned model.
    unsigned pollTicks_{0};
    /// \brief Spawn pose served by /mazegen/<model_name>/spawn_pose.
    ignition::math::Pose3d spawnPose_;
    /// \brief Goal poses served by /mazegen/<model_name>/goal_poses.
    ignition::msgs::Pose_V goalPoses_;
    /// \brief Transport node kept alive for the duration of the plugin.
    ignition::transport::Node node_;
    /// \brief Service name: /mazegen/<model_name>/spawn_pose.
    std::string spawnPoseService_;
    /// \brief Service name: /mazegen/<model_name>/goal_poses.
    std::string goalPosesService_;

    /// \brief Service handler — fills \p _rep with the stored spawn pose.
    bool OnSpawnPoseRequest(ignition::msgs::Pose &_rep);
    /// \brief Service handler — fills \p _rep with all goal cell poses.
    bool OnGoalPosesRequest(ignition::msgs::Pose_V &_rep);
  };
} // namespace mazegen

#endif /* MAZEGEN_PLUGIN_H_ */
