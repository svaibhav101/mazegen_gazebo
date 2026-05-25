# gazeboMaze API Reference {#apiref}

`mazegenPlugin` is an Ignition Gazebo Fortress (gz-sim 6) **world system
plugin** that loads a [micromouseonline](https://github.com/micromouseonline/mazefiles)
text-format maze and spawns the corresponding walls, lattice posts, and
start/goal markers into the running world.

## Where to start

- @ref mazegen_plugin::mazegenPlugin - the plugin entry point.
  Attached to a world via SDF, configured once at startup, then spawns the
  generated `<model>` through the world's `/create` service.
- @ref mazegen_plugin::Maze - in-memory representation of a parsed maze
  (cell grid plus per-edge wall flags, start cell, goal cells).
- @ref mazegen_plugin::ParseMazeFile - reads a micromouseonline `.txt`
  file into a @ref mazegen_plugin::Maze.

## High-level flow

1. Gazebo invokes `mazegenPlugin::Configure()` when the world loads.
2. The plugin parses its SDF parameters (`<maze_file>`, `<cell_size>`,
   `<wall_thickness>`, `<wall_height>`, `<post_size>`, `<origin>`).
3. The maze file is resolved against `IGN_GAZEBO_RESOURCE_PATH` and
   parsed by `ParseMazeFile()`.
4. An SDF string for a static maze model is built - collinear walls and
   their endpoint posts are merged into single bars to keep the
   collision/visual count down.
5. The SDF is written to a temp file and submitted to
   `/world/<name>/create`, which routes it through the same loader the
   world uses for `<include>` directives. This is what reliably
   preserves visual materials under Fortress' ogre2 renderer.

See @ref architecture for the rationale behind the design choices
(text-frame vs. world-frame, why we go through the service rather than
`SdfEntityCreator`, etc.).
