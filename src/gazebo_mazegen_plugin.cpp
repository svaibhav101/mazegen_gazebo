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
    /// 'IGN_GAZEBO_RESOURCE_PATH' directories are searched, so the plugin
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


    /// \brief Append an SDF '<material>' block in the given RGB color.
    ///
    /// In Fortress ogre2 renderer: 
    /// <ambient> and <diffuse> material components may appear 
    /// gray or have little visible effect because they rely on dynamic-
    /// lighting and scene ambient settings to be visible. 
    /// If lighting is not properly configured or if the material 
    /// is not receiving sufficient light, these components will not display as expected.
    /// -----------------------------------------------------------------------------
    /// <emissive> component renders reliably because 
    /// it simulates light emitted directly from the surface 
    /// and is not dependent on external lighting. 
    
    /// \param[in,out] _out Stream receiving the SDF fragment.
    /// \param[in] _r,_g,_b Colour components in '[0, 1]'.
    /// \param[in] _emissiveScale Factor applied to the emissive colour.
    void EmitMaterial(std::ostringstream &_out, double _r, double _g,
                      double _b, double _emissiveScale = 0.5)
    {
      _out << "<material>"
           << "<diffuse>" << _r << " " << _g << " " << _b << " 1</diffuse>"
           << "<emissive>" << _r * _emissiveScale << " "
           << _g * _emissiveScale << " " << _b * _emissiveScale
           << " 1</emissive>"
           << "</material>";
    }

    /// \brief Append a box shape - an optional collision plus a visual - to
    /// the enclosing '<link>'.
    /// \param[in,out] _out Stream receiving the SDF fragment.
    /// \param[in] _name Base name; collisions and visuals are prefixed.
    /// \param[in] _pose Box pose within the link frame.
    /// \param[in] _size Box dimensions.
    /// \param[in] _r,_g,_b Visual colour components in '[0, 1]'.
    /// \param[in] _collision Whether to also emit a '<collision>'.
    void EmitShape(std::ostringstream &_out, const std::string &_name,
                   const ignition::math::Pose3d &_pose,
                   const ignition::math::Vector3d &_size,
                   double _r, double _g, double _b, bool _collision)
    {
      if (_collision)
        _out << "<collision name='c_" << _name << "'>"
             << "<pose>" << _pose << "</pose>"
             << "<geometry><box><size>" << _size
             << "</size></box></geometry></collision>";
      _out << "<visual name='v_" << _name << "'>"
           << "<pose>" << _pose << "</pose>"
           << "<geometry><box><size>" << _size
           << "</size></box></geometry>";
      EmitMaterial(_out, _r, _g, _b);
      _out << "</visual>";
    }

    /// \brief Append a two-tone wall bar matching the micromouse spec.
    ///
    /// Emits one full-height collision plus two stacked visuals: a white
    /// body and a thin red cap, giving the "white sides, red top" look.
    /// \param[in,out] _out Stream receiving the SDF fragment.
    /// \param[in] _name Base name for the emitted shapes.
    /// \param[in] _cx,_cy Bar centre in the link's xy-plane.
    /// \param[in] _sx,_sy Bar footprint dimensions.
    /// \param[in] _h Total bar height.
    void EmitWallBar(std::ostringstream &_out, const std::string &_name,
                     double _cx, double _cy, double _sx, double _sy,
                     double _h)
    {
      const double capH = _h * 0.10;
      const double bodyH = _h - capH;

      _out << "<collision name='c_" << _name << "'>"
           << "<pose>" << _cx << " " << _cy << " " << _h * 0.5
           << " 0 0 0</pose>"
           << "<geometry><box><size>" << _sx << " " << _sy << " " << _h
           << "</size></box></geometry></collision>";

      EmitShape(_out, _name + "_body",
                ignition::math::Pose3d(_cx, _cy, bodyH * 0.5, 0, 0, 0),
                ignition::math::Vector3d(_sx, _sy, bodyH),
                1.0, 1.0, 1.0, false);
      EmitShape(_out, _name + "_cap",
                ignition::math::Pose3d(_cx, _cy, bodyH + capH * 0.5, 0, 0, 0),
                ignition::math::Vector3d(_sx, _sy, capH),
                0.9, 0.05, 0.05, false);
    }

  } // mazegenPlugin::Configure
} // namespace mazegen_plugin
