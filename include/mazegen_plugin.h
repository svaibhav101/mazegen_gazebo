#ifndef MAZEGEN_PLUGIN_H_
#define MAZEGEN_PLUGIN_H_

#include <string>

#include <ignition/gazebo/System.hh>
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
  };
} // namespace mazegen

#endif /* MAZEGEN_PLUGIN_H_ */
