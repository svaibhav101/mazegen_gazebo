#ifndef MAZEGEN_PLUGIN_H_
#define MAZEGEN_PLUGIN_H_

#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

/// \file mazegen_plugin.h
/// \brief World system plugin that spawns micromouse mazes in Ignition Gazebo.

namespace mazegen
{
  /// \brief World system plugin for Ignition Gazebo Fortress.
  ///
  /// Loads one or more micromouse-style mazes from text files (micromouseonline
  /// format) and spawns them as static SDF models via the world /create service.
  ///
  /// ### SDF configuration
  ///
  /// **Single-maze (flat) form:**
  /// \code{.xml}
  /// <plugin name="mazegen::MazegenPlugin" filename="libmazegen_plugin.so">
  ///   <maze_file>path/to/maze.txt</maze_file>
  ///   <model_name>maze</model_name>           <!-- optional -->
  ///   <origin>0 0 0 0 0 0</origin>            <!-- optional x y z r p y -->
  ///   <cell_size>0.18</cell_size>             <!-- optional; metres -->
  ///   <wall_thickness>0.012</wall_thickness>
  ///   <wall_height>0.05</wall_height>
  ///   <wall_color>1 1 1</wall_color>
  ///   <cap_color>0.9 0.05 0.05</cap_color>
  /// </plugin>
  /// \endcode
  ///
  /// **Multi-maze form** (shared geometry overrides apply to all blocks):
  /// \code{.xml}
  /// <plugin name="mazegen::MazegenPlugin" filename="libmazegen_plugin.so">
  ///   <cell_size>0.18</cell_size>
  ///   <maze>
  ///     <file>path/to/maze_a.txt</file>
  ///     <model_name>maze_a</model_name>
  ///     <origin>0 0 0 0 0 0</origin>
  ///   </maze>
  ///   <maze>
  ///     <file>path/to/maze_b.txt</file>
  ///     <origin>2 0 0 0 0 1.5707</origin>
  ///   </maze>
  /// </plugin>
  /// \endcode
  ///
  /// ### Transport services
  ///
  /// For each loaded maze the plugin advertises two persistent request/reply
  /// services:
  ///
  /// | Service                                  | Message type               |
  /// |------------------------------------------|----------------------------|
  /// | /mazegen/\<model_name\>/spawn_pose       | ignition::msgs::Pose       |
  /// | /mazegen/\<model_name\>/goal_poses       | ignition::msgs::Pose_V     |
  ///
  /// ### Runtime tile visualization
  ///
  /// Floor tile colours can be updated at runtime without plugin involvement
  /// by publishing to the standard Ignition marker topic:
  ///
  ///   /marker_array  —  ignition::msgs::Marker_V
  ///
  /// Use namespace "mazegen/\<model_name\>/tiles" and marker IDs of
  /// (row * cols + col) so each cell maps to a stable, updateable marker.
  /// Set alpha = 0.0 for cells that should be invisible.
  class MazegenPlugin
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
  public:
    MazegenPlugin() = default;
    ~MazegenPlugin() override = default;

    /// \brief Parse SDF, build maze SDF strings, write temp files, register
    ///        transport services.  The actual /create calls are deferred to
    ///        PreUpdate() because the transport graph is not fully connected
    ///        until Configure() returns.
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    /// \brief Issue /create service calls and poll the ECM until each model
    ///        appears, then clean up the temporary SDF files.
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

  private:
    /// \brief Per-maze bookkeeping for the deferred spawn handshake.
    struct MazeInstance
    {
      std::string modelName;            ///< Gazebo model name expected in the ECM.
      std::string pendingSdfFile;       ///< Temp file path; cleared after spawn.
      std::string createService;        ///< World /create service topic.
      bool requested{false};            ///< True after /create has been called.
      bool done{false};                 ///< True once the model appears in the ECM.
      unsigned pollTicks{0};            ///< PreUpdate ticks spent waiting after /create.
      ignition::math::Pose3d spawnPose; ///< Robot spawn pose in world frame.
      ignition::msgs::Pose_V goalPoses; ///< Goal cell centres in world frame.
    };

    std::vector<std::unique_ptr<MazeInstance>> mazes_; ///< All maze instances.
    bool initialized_{false};                          ///< Set to true when at least one maze is ready.
    ignition::transport::Node node_;                   ///< Shared transport node for all services.
  };

} // namespace mazegen

#endif /* MAZEGEN_PLUGIN_H_ */
