#ifndef GAZEBO_MAZEGEN_PLUGIN_H_
#define GAZEBO_MAZEGEN_PLUGIN_H_

#include <ignition/gazebo/System.hh>
#include <sdf/Element.hh>

/// \file gazebo_mazegen_plugin.h
/// \brief Public header for the `mazegenPlugin` Ignition Gazebo plugin.

/// \namespace mazegen_plugin
/// \brief All public symbols exposed by the gazeboMaze plugin.
namespace mazegen_plugin
{
  /// \brief World system plugin for Ignition Gazebo Fortress that builds a
  /// micromouse-style maze from a text file (micromouseonline format).
  ///
  /// Example SDF usage:
  /// \code
  /// <plugin filename="libgazebo_mazegen_plugin.so"
  ///         name="mazegen_plugin::mazegenPlugin">
  ///   <maze_file>mazes/japan2007ef.txt</maze_file>
  ///   <cell_size>0.18</cell_size>
  ///   <wall_thickness>0.012</wall_thickness>
  ///   <wall_height>0.05</wall_height>
  ///   <origin>0 0 0</origin>
  /// </plugin>
  /// \endcode
  class mazegenPlugin
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure
  {
  public:
    mazegenPlugin() = default;
    ~mazegenPlugin() override = default;

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;
  };
} // namespace mazegen_plugin

#endif /* GAZEBO_MAZEGEN_PLUGIN_H_ */
