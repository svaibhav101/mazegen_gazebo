#include "gazebo_mazegen_plugin.h"
#include "maze_parse.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>


IGNITION_ADD_PLUGIN(
  mazegen_plugin::mazegenPlugin,
  ignition::gazebo::System,
  mazegen_plugin::mazegenPlugin::ISystemConfigure
)

namespace mazegen_plugin
{
  namespace
  {

    /// \brief Tunable geometry for a spawned maze.
    ///
    /// Defaults follow the classic micromouse spec: 180 mm cells, 12 mm
    /// walls, 50 mm tall.
    struct Params
    {
      std::string mazeFile;
      double cellSize = 0.18;
      double wallThickness = 0.012;
      double wallHeight = 0.05;
      double postSize = 0.012;
      ignition::math::Vector3d origin{0.0, 0.0, 0.0};
    };

    /// \brief Resolve a maze file path to an existing file.
    ///
    /// An existing path is returned as-is; otherwise the
    /// `IGN_GAZEBO_RESOURCE_PATH` directories are searched, so the plugin
    /// works whether launched from the source tree or an installed location.
    /// \param[in] _path Absolute or relative maze path from the SDF.
    /// \return The resolved path, or \p _path unchanged if nothing matched.
    std::string ResolveMazePath(const std::string &_path)
    {
      if (std::filesystem::exists(_path))
        return _path;
      ignition::common::SystemPaths sp;
      sp.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
      std::string found = sp.FindFile(_path);
      return found.empty() ? _path : found;
    }

    /// \brief Read an optional double-valued SDF element, with a fallback.
    /// \param[in] _sdf SDF element to query.
    /// \param[in] _key Child element name.
    /// \param[in] _def Value to return when the child is absent.
    /// \return The element's value, or \p _def if it is not present.
    double GetDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                     const std::string &_key, double _def)
    {
      return _sdf->HasElement(_key) ? _sdf->Get<double>(_key) : _def;
    }

  } // mazegenPlugin::Configure
} // namespace mazegen_plugin
