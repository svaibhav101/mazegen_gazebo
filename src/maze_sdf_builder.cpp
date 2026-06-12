#include "maze_sdf_builder.h"

#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/math/Vector3.hh>

namespace mazegen
{
  namespace
  {
    // Hard wall surface: very stiff spring so the robot cannot penetrate.
    // kp = spring stiffness (N/m), kd = damping (N·s/m).
    // max_vel / min_depth control the ERP calculation in ODE.
    static const char kWallSurface[] =
        "<surface>"
        "<contact>"
        "<ode>"
        "<kp>1e8</kp>"
        "<kd>1e3</kd>"
        "<max_vel>0.01</max_vel>"
        "<min_depth>0.001</min_depth>"
        "</ode>"
        "</contact>"
        "<friction>"
        "<ode>"
        "<mu>0.8</mu>"
        "<mu2>0.8</mu2>"
        "</ode>"
        "</friction>"
        "</surface>";

    std::string fmtColor(double r, double g, double b, double a = 1.0)
    {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.3f %.3f %.3f %.3f", r, g, b, a);
      return buf;
    }

    std::string fmtVec3(double x, double y, double z)
    {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.6f %.6f %.6f", x, y, z);
      return buf;
    }

    // Emit a wall bar as raw SDF XML appended to `out`.
    // One collision (full height, with hard surface) + two visuals (body + cap).
    void EmitWallBar(std::ostringstream &out,
                     const std::string &name,
                     double cx, double cy,
                     double sx, double sy, double h,
                     const ignition::math::Vector3d &wallColor,
                     const ignition::math::Vector3d &capColor)
    {
      const double capH = h * 0.10;
      const double bodyH = h - capH;

      //  Collision (full height, hard surface)
      out << "<collision name='c_" << name << "'>"
          << "<pose>" << fmtVec3(cx, cy, h * 0.5) << " 0 0 0</pose>"
          << "<geometry><box><size>"
          << fmtVec3(sx, sy, h)
          << "</size></box></geometry>"
          << kWallSurface
          << "</collision>";

      //  Visual: body (lower, wall colour)
      const std::string wc = fmtColor(wallColor.X(), wallColor.Y(), wallColor.Z());
      out << "<visual name='v_" << name << "_body'>"
          << "<pose>" << fmtVec3(cx, cy, bodyH * 0.5) << " 0 0 0</pose>"
          << "<geometry><box><size>"
          << fmtVec3(sx, sy, bodyH)
          << "</size></box></geometry>"
          << "<material>"
          << "<ambient>" << wc << "</ambient>"
          << "<diffuse>" << wc << "</diffuse>"
          << "<emissive>" << fmtColor(wallColor.X() * 0.5, wallColor.Y() * 0.5, wallColor.Z() * 0.5)
          << "</emissive>"
          << "</material>"
          << "</visual>";

      //  Visual: cap (top 10 %, cap colour)
      const std::string cc = fmtColor(capColor.X(), capColor.Y(), capColor.Z());
      out << "<visual name='v_" << name << "_cap'>"
          << "<pose>" << fmtVec3(cx, cy, bodyH + capH * 0.5) << " 0 0 0</pose>"
          << "<geometry><box><size>"
          << fmtVec3(sx, sy, capH)
          << "</size></box></geometry>"
          << "<material>"
          << "<ambient>" << cc << "</ambient>"
          << "<diffuse>" << cc << "</diffuse>"
          << "<emissive>" << fmtColor(capColor.X() * 0.5, capColor.Y() * 0.5, capColor.Z() * 0.5)
          << "</emissive>"
          << "</material>"
          << "</visual>";
    }

    // Emit a floor tile visual (no collision).
    void EmitTile(std::ostringstream &out,
                  const std::string &name,
                  double cx, double cy,
                  double sx, double sy,
                  const ignition::math::Vector3d &color)
    {
      const double tileH = 0.001;
      const std::string c = fmtColor(color.X(), color.Y(), color.Z());
      out << "<visual name='v_" << name << "'>"
          << "<pose>" << fmtVec3(cx, cy, tileH * 0.5 + 1e-4) << " 0 0 0</pose>"
          << "<geometry><box><size>"
          << fmtVec3(sx, sy, tileH)
          << "</size></box></geometry>"
          << "<material>"
          << "<ambient>" << c << "</ambient>"
          << "<diffuse>" << c << "</diffuse>"
          << "<emissive>" << c << "</emissive>"
          << "</material>"
          << "</visual>";
    }

  } // anonymous namespace

  std::string BuildMazeSdf(const Maze &_maze, const Params &_p)
  {
    const double cs = _p.cellSize;
    const double wt = _p.wallThickness;
    const double wh = _p.wallHeight;
    const double wallLen = cs - _p.postSize;

    std::vector<std::vector<bool>> postCovered(
        _maze.cols + 1, std::vector<bool>(_maze.rows + 1, false));

    std::ostringstream walls;   // wall collision+visual elements
    std::ostringstream markers; // floor tile visuals

    //  Horizontal bars (greedy east-bound merge per latitude)
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
        const std::size_t cEnd = c;
        const double len = (cEnd - cStart) * cs + wt;
        const double cx = (cStart + cEnd) * 0.5 * cs;
        for (std::size_t k = cStart; k <= cEnd; ++k)
          postCovered[k][r] = true;
        std::ostringstream n;
        n << "h_" << cStart << "_" << r;
        EmitWallBar(walls, n.str(), cx, r * cs, len, wt, wh,
                    _p.wallColor, _p.capColor);
      }
    }

    //  Vertical bars (greedy north-bound merge per longitude)
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
        EmitWallBar(walls, n.str(), c * cs, cy, wt, len, wh,
                    _p.wallColor, _p.capColor);
      }
    }

    //  Isolated posts not absorbed by any bar
    for (std::size_t c = 0; c <= _maze.cols; ++c)
      for (std::size_t r = 0; r <= _maze.rows; ++r)
      {
        if (postCovered[c][r])
          continue;
        std::ostringstream n;
        n << "p_" << c << "_" << r;
        EmitWallBar(walls, n.str(), c * cs, r * cs,
                    _p.postSize, _p.postSize, wh,
                    _p.wallColor, _p.capColor);
      }

    //  Marker tiles
    EmitTile(markers, "start",
             (_maze.startCol + 0.5) * cs, (_maze.startRow + 0.5) * cs,
             wallLen, wallLen, {0.05, 0.35, 1.0});

    for (std::size_t i = 0; i < _maze.goalCells.size(); ++i)
    {
      std::ostringstream n;
      n << "goal_" << i;
      EmitTile(markers, n.str(),
               (_maze.goalCells[i].first + 0.5) * cs,
               (_maze.goalCells[i].second + 0.5) * cs,
               wallLen, wallLen, {1.0, 0.45, 0.0});
    }

    //  Assemble full SDF document
    const auto &o = _p.origin;
    const auto &rot = _p.rotation;

    std::ostringstream doc;
    doc << "<?xml version='1.0'?>"
        << "<sdf version='1.9'>"
        << "<model name='" << _p.modelName << "'>"
        << "<static>true</static>"
        << "<pose>"
        << o.X() << " " << o.Y() << " " << o.Z() << " "
        << rot.X() << " " << rot.Y() << " " << rot.Z()
        << "</pose>"

        // walls link
        << "<link name='walls'>"
        << walls.str()
        << "</link>"

        // markers link (visual only, no collision needed)
        << "<link name='markers'>"
        << markers.str()
        << "</link>"

        << "</model>"
        << "</sdf>";

    return doc.str();
  }

} // namespace mazegen
