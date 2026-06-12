#include "maze_params.h"

#include <filesystem>
#include <sstream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/math/Vector3.hh>

namespace mazegen
{
  std::string ResolveMazePath(const std::string &_path)
  {
    if (std::filesystem::exists(_path))
      return _path;
    ignition::common::SystemPaths sp;
    sp.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    std::string found = sp.FindFile(_path);
    return found.empty() ? _path : found;
  }

  double GetDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                   const std::string &_key, double _def)
  {
    return _sdf->HasElement(_key) ? _sdf->Get<double>(_key) : _def;
  }

  ignition::math::Vector3d GetVec3(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_key,
      const ignition::math::Vector3d &_def)
  {
    return _sdf->HasElement(_key)
               ? _sdf->Get<ignition::math::Vector3d>(_key)
               : _def;
  }

  void GetPose(const std::shared_ptr<const sdf::Element> &_sdf,
               ignition::math::Vector3d &_xyz,
               ignition::math::Vector3d &_rpy)
  {
    if (!_sdf->HasElement("origin"))
      return;
    std::istringstream ss(_sdf->Get<std::string>("origin"));
    double v[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; ++i)
      if (!(ss >> v[i]))
        break;
    _xyz = {v[0], v[1], v[2]};
    _rpy = {v[3], v[4], v[5]};
  }

  Params ReadSharedParams(const std::shared_ptr<const sdf::Element> &_sdf)
  {
    Params p;
    p.cellSize = GetDouble(_sdf, "cell_size", p.cellSize);
    p.wallThickness = GetDouble(_sdf, "wall_thickness", p.wallThickness);
    p.wallHeight = GetDouble(_sdf, "wall_height", p.wallHeight);
    p.postSize = GetDouble(_sdf, "post_size", p.wallThickness);
    p.wallColor = GetVec3(_sdf, "wall_color", p.wallColor);
    p.capColor = GetVec3(_sdf, "cap_color", p.capColor);
    return p;
  }

  Params ReadMazeBlock(const std::shared_ptr<const sdf::Element> &_maze,
                       const Params &_shared, std::size_t _index)
  {
    Params p = _shared;

    if (!_maze->HasElement("file"))
    {
      ignerr << "MazegenPlugin: <maze> block " << _index
             << " is missing required <file> element." << std::endl;
      p.mazeFile.clear();
      return p;
    }
    p.mazeFile = _maze->Get<std::string>("file");

    if (_maze->HasElement("model_name"))
      p.modelName = _maze->Get<std::string>("model_name");
    else
    {
      std::ostringstream n;
      n << "maze_" << _index;
      p.modelName = n.str();
    }

    p.cellSize = GetDouble(_maze, "cell_size", p.cellSize);
    p.wallThickness = GetDouble(_maze, "wall_thickness", p.wallThickness);
    p.wallHeight = GetDouble(_maze, "wall_height", p.wallHeight);
    p.postSize = GetDouble(_maze, "post_size", p.postSize);
    p.wallColor = GetVec3(_maze, "wall_color", p.wallColor);
    p.capColor = GetVec3(_maze, "cap_color", p.capColor);
    GetPose(_maze, p.origin, p.rotation);
    return p;
  }

} // namespace mazegen
