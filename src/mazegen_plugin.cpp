/// \file mazegen_plugin.cpp
/// \brief Implementation of MazegenPlugin — Configure and PreUpdate.

#include "mazegen_plugin.h"
#include "maze_params.h"
#include "maze_parse.h"
#include "maze_sdf_builder.h"
#include "maze_spawn_utils.h"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

IGNITION_ADD_PLUGIN(
    mazegen::MazegenPlugin,
    ignition::gazebo::System,
    mazegen::MazegenPlugin::ISystemConfigure,
    mazegen::MazegenPlugin::ISystemPreUpdate)

namespace mazegen
{
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
    const std::string createService = "/world/" + nameComp->Data() + "/create";

    // Parse the list of maze parameter blocks from SDF.
    // Two input formats are accepted:
    //   Multi:  one or more <maze> child blocks, each with <file> and <origin>.
    //   Single: flat <maze_file> / <model_name> / <origin> / geometry params.
    std::vector<Params> paramList;

    if (_sdf->HasElement("maze"))
    {
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
      if (!_sdf->HasElement("maze_file"))
      {
        ignerr << "MazegenPlugin: provide either <maze> blocks or a flat "
                  "<maze_file> element."
               << std::endl;
        return;
      }
      Params p = ReadSharedParams(_sdf);
      p.mazeFile = _sdf->Get<std::string>("maze_file");
      if (_sdf->HasElement("model_name"))
        p.modelName = _sdf->Get<std::string>("model_name");
      GetPose(_sdf, p.origin, p.rotation);
      paramList.push_back(std::move(p));
    }

    // Counter shared across Configure() calls to guarantee unique temp filenames
    // even if the plugin is instantiated more than once in the same process.
    static std::atomic<unsigned> tmpCounter{0};

    for (const Params &p : paramList)
    {
      if (p.mazeFile.empty())
        continue;

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

      std::cout << "[MazegenPlugin/" << p.modelName << "] loading from '"
                << resolved << "'" << std::endl;
      LogSpawnInfo(maze, p);

      // Write the generated SDF to a temp file; the /create service reads it
      // by filename rather than accepting an inline SDF string.
      const std::string sdfStr = BuildMazeSdf(maze, p);
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

      auto inst = std::make_unique<MazeInstance>();
      inst->modelName = p.modelName;
      inst->pendingSdfFile = tmpPath.string();
      inst->createService = createService;

      // Compute robot spawn pose from start cell position and open-side yaw.
      const auto startPos = CellCenter(maze.startCol, maze.startRow, p);
      std::string dir;
      const double yaw = SpawnYaw(maze, p, dir);
      inst->spawnPose = ignition::math::Pose3d(
          startPos.X(), startPos.Y(), startPos.Z(), 0.0, 0.0, yaw);

      for (const auto &gc : maze.goalCells)
      {
        const auto pos = CellCenter(gc.first, gc.second, p);
        ignition::msgs::Pose *entry = inst->goalPoses.add_pose();
        ignition::msgs::Set(entry,
                            ignition::math::Pose3d(
                                pos.X(), pos.Y(), pos.Z(), 0.0, 0.0, 0.0));
      }

      // Advertise persistent request/reply services for spawn and goal poses.
      // Capture by value so the lambdas remain valid after the loop iteration.
      const ignition::math::Pose3d spawnPoseCopy = inst->spawnPose;
      const ignition::msgs::Pose_V goalPosesCopy = inst->goalPoses;

      const std::string spawnSvc = "/mazegen/" + p.modelName + "/spawn_pose";
      const std::string goalSvc = "/mazegen/" + p.modelName + "/goal_poses";

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
      node_.Advertise(goalSvc, goalCb);

      std::cout << "[MazegenPlugin/" << p.modelName << "] maze size: "
                << maze.cols << "x" << maze.rows
                << "  marker ns: mazegen/" << p.modelName << "/tiles"
                << "  marker IDs: 0.." << (maze.cols * maze.rows - 1)
                << "  (id = y * " << maze.cols << " + x)" << std::endl;
      std::cout << "[MazegenPlugin/" << p.modelName << "] services ready: "
                << spawnSvc << " | " << goalSvc << std::endl;

      mazes_.push_back(std::move(inst));
    }

    if (!mazes_.empty())
      initialized_ = true;
  }

  void MazegenPlugin::PreUpdate(
      const ignition::gazebo::UpdateInfo & /*_info*/,
      ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (!initialized_)
      return;

    // Maximum ticks to wait for a model to appear in the ECM after /create.
    constexpr unsigned kMaxPollTicks = 100;

    for (auto &instUp : mazes_)
    {
      MazeInstance &inst = *instUp;

      if (inst.done || inst.pendingSdfFile.empty())
        continue;

      if (!inst.requested)
      {
        // Issue the /create service call (blocks for up to 5 s).
        ignition::msgs::EntityFactory req;
        req.set_sdf_filename(inst.pendingSdfFile);
        ignition::msgs::Boolean rep;
        bool ok = false;

        const bool called = node_.Request(inst.createService, req, 5000, rep, ok);
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
        continue; // allow at least one tick before polling the ECM
      }

      // Poll the ECM for the model name to confirm the entity was created.
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
