#include "mazegen_plugin.h"
#include "maze_parse.h"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>


IGNITION_ADD_PLUGIN(
  mazegen::MazegenPlugin,
  ignition::gazebo::System,
  mazegen::MazegenPlugin::ISystemConfigure,
  mazegen::MazegenPlugin::ISystemPreUpdate
)

namespace mazegen
{
  namespace
  {

    /// \brief Tunable geometry and appearance for a spawned maze.
    ///
    /// Defaults follow the classic micromouse spec: 180 mm cells, 12 mm
    /// walls, 50 mm tall, white body with a red cap.
    struct Params
    {
      std::string mazeFile;
      std::string modelName    = "maze";
      double cellSize      = 0.18;
      double wallThickness = 0.012;
      double wallHeight    = 0.05;
      double postSize      = 0.012;
      ignition::math::Vector3d origin{0.0, 0.0, 0.0};
      ignition::math::Vector3d rotation{0.0, 0.0, 0.0};
      // Wall body colour (RGB, each in [0,1])
      ignition::math::Vector3d wallColor{1.0, 1.0, 1.0};
      // Top-cap colour (RGB, each in [0,1])
      ignition::math::Vector3d capColor{0.9, 0.05, 0.05};
    };

    /// \brief Resolve a maze file path, falling back to IGN_GAZEBO_RESOURCE_PATH.
    std::string ResolveMazePath(const std::string &_path)
    {
      if (std::filesystem::exists(_path))
        return _path;
      ignition::common::SystemPaths sp;
      sp.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
      std::string found = sp.FindFile(_path);
      return found.empty() ? _path : found;
    }

    /// \brief Read an optional double SDF element, returning \p _def if absent.
    double GetDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                     const std::string &_key, double _def)
    {
      return _sdf->HasElement(_key) ? _sdf->Get<double>(_key) : _def;
    }

    /// \brief Read an optional Vector3d SDF element, returning \p _def if absent.
    ignition::math::Vector3d GetVec3(
        const std::shared_ptr<const sdf::Element> &_sdf,
        const std::string &_key,
        const ignition::math::Vector3d &_def)
    {
      return _sdf->HasElement(_key)
                 ? _sdf->Get<ignition::math::Vector3d>(_key)
                 : _def;
    }

    /// \brief Parse <origin>x y z [roll pitch yaw]</origin> : rotation defaults to 0.
    void GetPose(const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::math::Vector3d &_xyz,
                 ignition::math::Vector3d &_rpy)
    {
      if (!_sdf->HasElement("origin"))
        return;
      std::istringstream ss(_sdf->Get<std::string>("origin"));
      double v[6] = {0, 0, 0, 0, 0, 0};
      for (int i = 0; i < 6; ++i)
        if (!(ss >> v[i])) break;
      _xyz = {v[0], v[1], v[2]};
      _rpy = {v[3], v[4], v[5]};
    }

    // -----------------------------------------------------------------------
    // sdformat-based SDF builders
    // -----------------------------------------------------------------------

    /// \brief Fill an sdf::Material with diffuse + emissive in the given colour.
    ///
    /// Under Fortress ogre2, <emissive> is used because <diffuse> alone requires
    /// dynamic lighting to be visible; emissive renders regardless of scene lights.
    sdf::Material MakeMaterial(const ignition::math::Vector3d &_rgb,
                               double _emissiveScale = 0.5)
    {
      sdf::Material mat;
      ignition::math::Color c(
          static_cast<float>(_rgb.X()),
          static_cast<float>(_rgb.Y()),
          static_cast<float>(_rgb.Z()), 1.0f);
      ignition::math::Color e(
          static_cast<float>(_rgb.X() * _emissiveScale),
          static_cast<float>(_rgb.Y() * _emissiveScale),
          static_cast<float>(_rgb.Z() * _emissiveScale), 1.0f);
      mat.SetDiffuse(c);
      mat.SetEmissive(e);
      return mat;
    }

    /// \brief Append a box visual (and optionally a collision) to \p _link.
    void AddBox(sdf::Link &_link, const std::string &_name,
                const ignition::math::Pose3d &_pose,
                const ignition::math::Vector3d &_size,
                const ignition::math::Vector3d &_rgb,
                bool _collision)
    {
      sdf::Geometry geom;
      sdf::Box box;
      box.SetSize(_size);
      geom.SetType(sdf::GeometryType::BOX);
      geom.SetBoxShape(box);

      if (_collision)
      {
        sdf::Collision col;
        col.SetName("c_" + _name);
        col.SetRawPose(_pose);
        col.SetGeom(geom);
        _link.AddCollision(col);
      }

      sdf::Visual vis;
      vis.SetName("v_" + _name);
      vis.SetRawPose(_pose);
      vis.SetGeom(geom);
      vis.SetMaterial(MakeMaterial(_rgb));
      _link.AddVisual(vis);
    }

    /// \brief Append a two-tone wall bar (white body + coloured cap) to \p _link.
    ///
    /// One collision covers the full height; two visuals stack body and cap.
    /// The bar is centred at (_cx, _cy) with footprint (_sx, _sy) × height _h.
    void AddWallBar(sdf::Link &_link, const std::string &_name,
                    double _cx, double _cy, double _sx, double _sy, double _h,
                    const ignition::math::Vector3d &_wallColor,
                    const ignition::math::Vector3d &_capColor)
    {
      const double capH  = _h * 0.10;
      const double bodyH = _h - capH;

      // Single collision for the full bar.
      sdf::Geometry geom;
      sdf::Box box;
      box.SetSize({_sx, _sy, _h});
      geom.SetType(sdf::GeometryType::BOX);
      geom.SetBoxShape(box);

      sdf::Collision col;
      col.SetName("c_" + _name);
      col.SetRawPose({_cx, _cy, _h * 0.5, 0, 0, 0});
      col.SetGeom(geom);
      _link.AddCollision(col);

      // Body visual (lower portion).
      AddBox(_link, _name + "_body",
             {_cx, _cy, bodyH * 0.5, 0, 0, 0},
             {_sx, _sy, bodyH}, _wallColor, false);

      // Cap visual (top strip).
      AddBox(_link, _name + "_cap",
             {_cx, _cy, bodyH + capH * 0.5, 0, 0, 0},
             {_sx, _sy, capH}, _capColor, false);
    }

    /// \brief Build the complete SDF document for the maze as a string.
    ///
    /// The maze is one static model with two links:
    /// - 'walls': collision + visual boxes. Collinear wall runs are merged into
    ///   single bars (greedy left-to-right / bottom-to-top scan). This is a
    ///   greedy, single-pass merge: a run of walls between two gaps produces one
    ///   bar per contiguous segment, which is optimal per-row/column but does not
    ///   attempt cross-row merges. A single isolated post that sits between two
    ///   gaps on both axes will remain as a standalone cube.
    /// - 'markers': visual-only floor tiles for start (green) and goal (red).
    ///
    /// Geometry is in a local frame with +X east and +Y north, shifted so the
    /// SW corner of the start cell lands on \p _p.origin.
    std::string BuildMazeSdf(const Maze &_maze, const Params &_p)
    {
      const double cs      = _p.cellSize;
      const double wt      = _p.wallThickness;
      const double wh      = _p.wallHeight;
      const double wallLen = cs - _p.postSize;

      // postCovered[c][r]: true when post (c,r) has been absorbed into a merged bar.
      std::vector<std::vector<bool>> postCovered(
          _maze.cols + 1, std::vector<bool>(_maze.rows + 1, false));

      sdf::Link wallsLink;
      wallsLink.SetName("walls");

      // --- Horizontal bars (greedy east-bound merge per latitude) ---
      for (std::size_t r = 0; r <= _maze.rows; ++r)
      {
        std::size_t c = 0;
        while (c < _maze.cols)
        {
          if (!_maze.hWall[c][r]) { ++c; continue; }
          const std::size_t cStart = c;
          while (c < _maze.cols && _maze.hWall[c][r]) ++c;
          const std::size_t cEnd = c;
          const double len = (cEnd - cStart) * cs + wt;
          const double cx  = (cStart + cEnd) * 0.5 * cs;
          for (std::size_t k = cStart; k <= cEnd; ++k)
            postCovered[k][r] = true;
          std::ostringstream n;
          n << "h_" << cStart << "_" << r;
          AddWallBar(wallsLink, n.str(), cx, r * cs, len, wt, wh,
                     _p.wallColor, _p.capColor);
        }
      }

      // --- Vertical bars (greedy north-bound merge per longitude) ---
      for (std::size_t c = 0; c <= _maze.cols; ++c)
      {
        std::size_t r = 0;
        while (r < _maze.rows)
        {
          if (!_maze.vWall[c][r]) { ++r; continue; }
          const std::size_t rStart = r;
          while (r < _maze.rows && _maze.vWall[c][r]) ++r;
          const std::size_t rEnd = r;
          const double len = (rEnd - rStart) * cs + wt;
          const double cy  = (rStart + rEnd) * 0.5 * cs;
          for (std::size_t k = rStart; k <= rEnd; ++k)
            postCovered[c][k] = true;
          std::ostringstream n;
          n << "v_" << c << "_" << rStart;
          AddWallBar(wallsLink, n.str(), c * cs, cy, wt, len, wh,
                     _p.wallColor, _p.capColor);
        }
      }

      // --- Isolated posts not absorbed by any bar ---
      for (std::size_t c = 0; c <= _maze.cols; ++c)
        for (std::size_t r = 0; r <= _maze.rows; ++r)
        {
          if (postCovered[c][r]) continue;
          std::ostringstream n;
          n << "p_" << c << "_" << r;
          AddWallBar(wallsLink, n.str(), c * cs, r * cs,
                     _p.postSize, _p.postSize, wh,
                     _p.wallColor, _p.capColor);
        }

      // --- Marker tiles (start = green, goal = red) ---
      sdf::Link markersLink;
      markersLink.SetName("markers");

      const double tileH = 0.001;
      const ignition::math::Vector3d tileSize(wallLen, wallLen, tileH);

      auto addTile = [&](const std::string &_n, int _c, int _row,
                         const ignition::math::Vector3d &_rgb)
      {
        AddBox(markersLink, _n,
               {(_c + 0.5) * cs, (_row + 0.5) * cs, tileH * 0.5 + 1e-4,
                0, 0, 0},
               tileSize, _rgb, false);
      };

      addTile("start", _maze.startCol, _maze.startRow, {0.1, 0.7, 0.1});
      for (std::size_t i = 0; i < _maze.goalCells.size(); ++i)
      {
        std::ostringstream n;
        n << "goal_" << i;
        addTile(n.str(), _maze.goalCells[i].first,
                _maze.goalCells[i].second, {0.85, 0.1, 0.1});
      }

      // Assemble model via sdformat types and serialise to string.
      sdf::Model model;
      model.SetName(_p.modelName);
      model.SetStatic(true);
      model.SetRawPose({_p.origin.X(), _p.origin.Y(), _p.origin.Z(),
                        _p.rotation.X(), _p.rotation.Y(), _p.rotation.Z()});
      model.AddLink(wallsLink);
      model.AddLink(markersLink);

      // Wrap in a minimal SDF document.
      auto modelElem = model.ToElement();
      std::ostringstream doc;
      doc << "<?xml version='1.0'?><sdf version='1.9'>"
          << modelElem->ToString("")
          << "</sdf>";
      return doc.str();
    }

    // -----------------------------------------------------------------------
    // Spawn-info helpers
    // -----------------------------------------------------------------------

    ignition::math::Vector3d CellCenter(int _col, int _row, const Params &_p)
    {
      return {_p.origin.X() + (_col + 0.5) * _p.cellSize,
              _p.origin.Y() + (_row + 0.5) * _p.cellSize,
              _p.origin.Z()};
    }

    /// \brief Derive the spawn yaw from the open side of the start cell.
    ///
    /// Priority: east -> north -> west -> south. Returns 0 and warns if all
    /// four sides are walled. Also fills \p _dir with a human-readable label.
    double SpawnYaw(const Maze &_m, std::string &_dir)
    {
      const int c = _m.startCol, r = _m.startRow;
      const bool wallN = _m.hWall[c][r + 1];
      const bool wallS = _m.hWall[c][r];
      const bool wallW = _m.vWall[c][r];
      const bool wallE = _m.vWall[c + 1][r];

      if      (!wallE) { _dir = "east (+X)";  return  0.0; }
      else if (!wallN) { _dir = "north (+Y)"; return  1.5707963267948966; }
      else if (!wallW) { _dir = "west (-X)";  return  3.1415926535897931; }
      else if (!wallS) { _dir = "south (-Y)"; return -1.5707963267948966; }

      ignwarn << "MazegenPlugin: start cell is walled on all four sides; "
                 "defaulting mouse yaw to 0." << std::endl;
      _dir = "none (fully enclosed)";
      return 0.0;
    }

    void LogSpawnInfo(const Maze &_m, const Params &_p)
    {
      const auto start = CellCenter(_m.startCol, _m.startRow, _p);
      ignmsg << "==> MazegenPlugin [" << _p.modelName << "]: start cell (col "
             << _m.startCol << ", row " << _m.startRow << ") at world x="
             << start.X() << " y=" << start.Y() << " z=" << start.Z()
             << std::endl;

      for (std::size_t i = 0; i < _m.goalCells.size(); ++i)
      {
        const auto g = CellCenter(_m.goalCells[i].first,
                                  _m.goalCells[i].second, _p);
        ignmsg << "==> MazegenPlugin [" << _p.modelName << "]: goal cell " << i
               << " (col " << _m.goalCells[i].first << ", row "
               << _m.goalCells[i].second << ") at world x=" << g.X()
               << " y=" << g.Y() << " z=" << g.Z() << std::endl;
      }

      std::string dir;
      const double yaw = SpawnYaw(_m, dir);

      ignmsg << "==> MazegenPlugin [" << _p.modelName << "]: spawn mouse at x="
             << start.X() << " y=" << start.Y() << " z=" << start.Z()
             << " yaw=" << yaw << " rad (facing " << dir
             << ", the open side of the start cell)" << std::endl;
      ignmsg << "==> MazegenPlugin [" << _p.modelName << "]: cell_size="
             << _p.cellSize << " m, maze spans x=[" << _p.origin.X() << ", "
             << _p.origin.X() + _m.cols * _p.cellSize << "] y=["
             << _p.origin.Y() << ", "
             << _p.origin.Y() + _m.rows * _p.cellSize
             << "] (first quadrant, bottom-left corner at origin)" << std::endl;
    }

    // -----------------------------------------------------------------------
    // SDF parameter parsing helpers
    // -----------------------------------------------------------------------

    /// \brief Read shared geometry/colour params from the top-level plugin SDF.
    Params ReadSharedParams(const std::shared_ptr<const sdf::Element> &_sdf)
    {
      Params p;
      p.cellSize      = GetDouble(_sdf, "cell_size",      p.cellSize);
      p.wallThickness = GetDouble(_sdf, "wall_thickness", p.wallThickness);
      p.wallHeight    = GetDouble(_sdf, "wall_height",    p.wallHeight);
      p.postSize      = GetDouble(_sdf, "post_size",      p.wallThickness);
      p.wallColor     = GetVec3 (_sdf, "wall_color",      p.wallColor);
      p.capColor      = GetVec3 (_sdf, "cap_color",       p.capColor);
      return p;
    }

    /// \brief Populate per-maze fields from a <maze> child element, inheriting
    /// geometry/colour from \p _shared. \p _index is used for the default name.
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

      // Per-maze geometry overrides (optional).
      p.cellSize      = GetDouble(_maze, "cell_size",      p.cellSize);
      p.wallThickness = GetDouble(_maze, "wall_thickness", p.wallThickness);
      p.wallHeight    = GetDouble(_maze, "wall_height",    p.wallHeight);
      p.postSize      = GetDouble(_maze, "post_size",      p.postSize);
      p.wallColor     = GetVec3 (_maze, "wall_color",      p.wallColor);
      p.capColor      = GetVec3 (_maze, "cap_color",       p.capColor);
      GetPose(_maze, p.origin, p.rotation);
      return p;
    }

  } // anonymous namespace


  // -------------------------------------------------------------------------
  // Configure: parse SDF params, build maze SDF(s), write to temp files.
  // The actual /create service calls are deferred to PreUpdate() because the
  // transport graph isn't fully connected until Configure() returns.
  //
  // Two input formats are accepted:
  //   Single: flat <maze_file> / <model_name> / <origin> / geometry params
  //   Multi:  one or more <maze> child blocks, each with <file> and <origin>
  // -------------------------------------------------------------------------
  void MazegenPlugin::Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager & /*_eventMgr*/)
  {
    if (!_ecm.Component<ignition::gazebo::components::World>(_entity))
    {
      ignerr << "MazegenPlugin must be attached to a <world>." << std::endl;
      return;
    }

    const auto *nameComp =
        _ecm.Component<ignition::gazebo::components::Name>(_entity);
    if (!nameComp)
    {
      ignerr << "MazegenPlugin: world has no Name component." << std::endl;
      return;
    }
    const std::string createService =
        "/world/" + nameComp->Data() + "/create";

    // Build the list of Params : one per maze to load.
    std::vector<Params> paramList;

    if (_sdf->HasElement("maze"))
    {
      // Multi-maze path: collect all <maze> child blocks.
      const Params shared = ReadSharedParams(_sdf);
      auto mazeElem = _sdf->GetElementImpl("maze");
      std::size_t idx = 0;
      while (mazeElem)
      {
        paramList.push_back(ReadMazeBlock(mazeElem, shared, idx++));
        mazeElem = mazeElem->GetNextElement("maze");
      }
    }
    else
    {
      // Legacy single-maze path: flat params directly on the plugin element.
      if (!_sdf->HasElement("maze_file"))
      {
        ignerr << "MazegenPlugin: provide either <maze> blocks or a flat "
                  "<maze_file> element." << std::endl;
        return;
      }
      Params p = ReadSharedParams(_sdf);
      p.mazeFile = _sdf->Get<std::string>("maze_file");
      if (_sdf->HasElement("model_name"))
        p.modelName = _sdf->Get<std::string>("model_name");
      GetPose(_sdf, p.origin, p.rotation);
      paramList.push_back(std::move(p));
    }

    static std::atomic<unsigned> tmpCounter{0};

    for (const Params &p : paramList)
    {
      if (p.mazeFile.empty())
        continue; // error already logged by ReadMazeBlock

      const std::string resolved = ResolveMazePath(p.mazeFile);
      Maze maze;
      try
      {
        maze = ParseMazeFile(resolved);
      }
      catch (const std::exception &e)
      {
        ignerr << "MazegenPlugin [" << p.modelName << "]: " << e.what()
               << std::endl;
        continue;
      }

      ignmsg << "==> MazegenPlugin [" << p.modelName << "]: loaded "
             << maze.cols << "x" << maze.rows << " maze from '"
             << resolved << "'" << std::endl;
      LogSpawnInfo(maze, p);

      const std::string sdfStr = BuildMazeSdf(maze, p);

      // Write to a temp file: the inline-string path of EntityFactory drops
      // materials under ogre2; the filename path goes through the full SDF loader.
      // Counter + PID makes the name unique across all instances and processes.
      auto tmpPath = std::filesystem::temp_directory_path() /
                     ("mazegen_" + std::to_string(::getpid()) + "_" +
                      std::to_string(tmpCounter++) + ".sdf");
      {
        std::ofstream f(tmpPath);
        if (!f)
        {
          ignerr << "MazegenPlugin [" << p.modelName << "]: cannot write "
                 << tmpPath << std::endl;
          continue;
        }
        f << sdfStr;
      }

      // Build and store the instance.
      MazeInstance inst;
      inst.modelName      = p.modelName;
      inst.pendingSdfFile = tmpPath.string();
      inst.createService  = createService;

      // Spawn pose (position + yaw from open side of start cell).
      const auto startPos = CellCenter(maze.startCol, maze.startRow, p);
      std::string dir;
      const double yaw = SpawnYaw(maze, dir);
      inst.spawnPose = ignition::math::Pose3d(
          startPos.X(), startPos.Y(), startPos.Z(), 0.0, 0.0, yaw);

      // Goal poses (position only, identity orientation).
      for (const auto &gc : maze.goalCells)
      {
        const auto pos = CellCenter(gc.first, gc.second, p);
        ignition::msgs::Pose *entry = inst.goalPoses.add_pose();
        ignition::msgs::Set(entry,
            ignition::math::Pose3d(pos.X(), pos.Y(), pos.Z(), 0.0, 0.0, 0.0));
      }

      // Advertise per-maze query services. Capture by value so each lambda
      // holds its own copy of the pose data independent of the vector.
      const ignition::math::Pose3d spawnPoseCopy  = inst.spawnPose;
      const ignition::msgs::Pose_V goalPosesCopy   = inst.goalPoses;

      const std::string spawnSvc = "/mazegen/" + p.modelName + "/spawn_pose";
      const std::string goalSvc  = "/mazegen/" + p.modelName + "/goal_poses";

      std::function<bool(ignition::msgs::Pose &)> spawnCb =
          [spawnPoseCopy](ignition::msgs::Pose &_rep) mutable -> bool
          {
            ignition::msgs::Set(&_rep, spawnPoseCopy);
            return true;
          };
      std::function<bool(ignition::msgs::Pose_V &)> goalCb =
          [goalPosesCopy](ignition::msgs::Pose_V &_rep) mutable -> bool
          {
            _rep = goalPosesCopy;
            return true;
          };
      node_.Advertise(spawnSvc, spawnCb);
      node_.Advertise(goalSvc,  goalCb);

      ignmsg << "==> MazegenPlugin [" << p.modelName
             << "]: services ready at '" << spawnSvc
             << "' and '" << goalSvc << "'" << std::endl;

      mazes_.push_back(std::move(inst));
    }

    if (!mazes_.empty())
      initialized_ = true;
  }


  // -------------------------------------------------------------------------
  // PreUpdate runs every simulation tick.
  //
  // For each pending MazeInstance:
  //   Tick 1  (requested == false): send the /create request.
  //   Tick 2+ (requested == true):  poll ECM until the model appears, then
  //                                  delete the temp file. Give up after
  //                                  kMaxPollTicks to avoid a permanent leak.
  // -------------------------------------------------------------------------
  void MazegenPlugin::PreUpdate(
      const ignition::gazebo::UpdateInfo & /*_info*/,
      ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (!initialized_)
      return;

    constexpr unsigned kMaxPollTicks = 100;

    for (MazeInstance &inst : mazes_)
    {
      if (inst.done || inst.pendingSdfFile.empty())
        continue;

      if (!inst.requested)
      {
        ignition::msgs::EntityFactory req;
        req.set_sdf_filename(inst.pendingSdfFile);
        ignition::msgs::Boolean rep;
        bool ok = false;

        const bool called =
            node_.Request(inst.createService, req, 5000, rep, ok);
        if (!called || !ok || !rep.data())
        {
          ignerr << "MazegenPlugin [" << inst.modelName
                 << "]: /create service call failed." << std::endl;
          std::error_code ec;
          std::filesystem::remove(inst.pendingSdfFile, ec);
          inst.pendingSdfFile.clear();
          inst.done = true;
          continue;
        }
        inst.requested = true;
        continue; // wait at least one more tick before checking ECM
      }

      // Poll: check whether the model entity has appeared in the ECM.
      bool found = false;
      _ecm.Each<ignition::gazebo::components::Name>(
          [&](const ignition::gazebo::Entity &,
              const ignition::gazebo::components::Name *_name) -> bool
          {
            if (_name->Data() == inst.modelName)
            {
              found = true;
              return false; // stop iteration
            }
            return true;
          });

      if (found)
      {
        std::error_code ec;
        std::filesystem::remove(inst.pendingSdfFile, ec);
        inst.pendingSdfFile.clear();
        inst.done = true;
        continue;
      }

      // Give up after kMaxPollTicks to avoid leaking the temp file if the
      // spawned model never appears in the ECM.
      if (++inst.pollTicks >= kMaxPollTicks)
      {
        ignerr << "MazegenPlugin [" << inst.modelName
               << "]: model did not appear in ECM after " << kMaxPollTicks
               << " ticks; deleting temp file and giving up." << std::endl;
        std::error_code ec;
        std::filesystem::remove(inst.pendingSdfFile, ec);
        inst.pendingSdfFile.clear();
        inst.done = true;
      }
    }
  }

} // namespace mazegen
