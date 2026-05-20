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

    /// \brief Build the complete SDF document for a maze.
    ///
    /// The maze is one static '<model>' with two links:
    /// - 'walls': collision+visual boxes, with collinear wall runs (and their
    ///   endpoint posts) merged into single bars to keep the shape count low.
    /// - 'markers': visual-only floor tiles colouring the start/goal cells.
    ///
    /// Geometry is built in a text-natural local frame (local +X = text-east,
    /// local +Y = text-north) kept aligned with the world (no yaw), then
    /// shifted so the start cell's south-west corner lands on the user
    /// '<origin>'. The maze therefore grows into the first quadrant.
    /// \param[in] _maze Parsed maze to render.
    /// \param[in] _p Geometry parameters and placement origin.
    /// \return A self-contained SDF document string.
    std::string BuildMazeSdf(const Maze &_maze, const Params &_p)
    {
      std::ostringstream s;
      s.setf(std::ios::fixed);
      s.precision(6);

      const double cs = _p.cellSize;
      const double wt = _p.wallThickness;
      const double wh = _p.wallHeight;
      const double wallLen = cs - _p.postSize;

      // With no yaw, a local point (lx, ly) maps to world (tx + lx, ty + ly).
      // The maze's bottom-left corner (col 0, row 0 in the text file) is the
      // local point (0, 0); we drop that onto <origin> so the maze fills the
      // first quadrant with world +X = east (right) and world +Y = north (up).
      const double yaw = 0.0;
      const double tx = _p.origin.X();
      const double ty = _p.origin.Y();

      s << "<?xml version='1.0'?><sdf version='1.9'>"
        << "<model name='maze'><static>true</static>"
        << "<pose>" << tx << " " << ty << " " << _p.origin.Z()
        << " 0 0 " << yaw << "</pose>"
        << "<link name='walls'>";

      // postCovered[c][r] becomes true once post (c,r) has been swallowed by
      // a merged horizontal or vertical bar. Remaining posts are emitted as
      // isolated cubes after the bar passes.
      std::vector<std::vector<bool>> postCovered(
          _maze.cols + 1, std::vector<bool>(_maze.rows + 1, false));

      // Horizontal bars: at each latitude r, scan east-bound and merge
      // consecutive set walls (plus their endpoint posts) into one bar.
      for (std::size_t r = 0; r <= _maze.rows; ++r)
      {
        std::size_t c = 0;
        while (c < _maze.cols)
        {
          if (!_maze.hWall[c][r])
          {
            ++c;
            continue;
          }
          const std::size_t cStart = c;
          while (c < _maze.cols && _maze.hWall[c][r])
            ++c;
          const std::size_t cEnd = c; // last post column = cEnd
          const double len = (cEnd - cStart) * cs + wt;
          const double cx = (cStart + cEnd) * 0.5 * cs;
          for (std::size_t k = cStart; k <= cEnd; ++k)
            postCovered[k][r] = true;
          std::ostringstream n;
          n << "h_" << cStart << "_" << r;
          EmitWallBar(s, n.str(), cx, r * cs, len, wt, wh);
        }
      }

      // Vertical bars: same idea along the +Y direction at each longitude c.
      for (std::size_t c = 0; c <= _maze.cols; ++c)
      {
        std::size_t r = 0;
        while (r < _maze.rows)
        {
          if (!_maze.vWall[c][r])
          {
            ++r;
            continue;
          }
          const std::size_t rStart = r;
          while (r < _maze.rows && _maze.vWall[c][r])
            ++r;
          const std::size_t rEnd = r;
          const double len = (rEnd - rStart) * cs + wt;
          const double cy = (rStart + rEnd) * 0.5 * cs;
          for (std::size_t k = rStart; k <= rEnd; ++k)
            postCovered[c][k] = true;
          std::ostringstream n;
          n << "v_" << c << "_" << rStart;
          EmitWallBar(s, n.str(), c * cs, cy, wt, len, wh);
        }
      }

      // Isolated posts that no bar absorbed (sea of dots between empty cells).
      for (std::size_t c = 0; c <= _maze.cols; ++c)
      {
        for (std::size_t r = 0; r <= _maze.rows; ++r)
        {
          if (postCovered[c][r])
            continue;
          std::ostringstream n;
          n << "p_" << c << "_" << r;
          EmitWallBar(s, n.str(), c * cs, r * cs,
                      _p.postSize, _p.postSize, wh);
        }
      }

      s << "</link>";

      // Visual-only floor tiles for start (green) and goal (red) cells.
      const double tileH = 0.001;
      const ignition::math::Vector3d tileSize(wallLen, wallLen, tileH);
      auto tile = [&](const std::string &_n, int _c, int _r,
                      double _r_, double _g_, double _b_)
      {
        EmitShape(s, _n,
                  ignition::math::Pose3d((_c + 0.5) * cs, (_r + 0.5) * cs,
                                         tileH * 0.5 + 1e-4, 0, 0, 0),
                  tileSize, _r_, _g_, _b_, false);
      };
      s << "<link name='markers'>";
      tile("start", _maze.startCol, _maze.startRow, 0.1, 0.7, 0.1);
      for (std::size_t i = 0; i < _maze.goalCells.size(); ++i)
      {
        std::ostringstream n;
        n << "goal_" << i;
        tile(n.str(), _maze.goalCells[i].first, _maze.goalCells[i].second,
             0.85, 0.1, 0.1);
      }
      s << "</link>";

      s << "</model></sdf>";
      return s.str();
    }

  } // mazegenPlugin::Configure
} // namespace mazegen_plugin
