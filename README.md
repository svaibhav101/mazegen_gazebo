# gazebo Maze-Gen

A **Gazebo Ignition Fortress** (`gz-sim 6`) world-system plugin that reads a
[micromouseonline](https://github.com/micromouseonline/mazefiles)-format
text maze file and spawns the corresponding walls, posts, and
start/goal markers into a running world.


| | |
|---|---|
| Library (SDF `filename=`) | `libgazebo_mazegen_plugin.so` |
| Plugin class (SDF `name=`) | `mazegen_plugin::mazegenPlugin` |
| Plugin type | World system (`ISystemConfigure`) |
| Default cell size | 180 mm (classic micromouse) |
| Default wall | 12 mm thick x 50 mm tall, white sides, red top |
| Default posts | 12 x 12 x 50 mm at every grid intersection |
| Example floor | Non-gloss black ([`worlds/maze.sdf`](./worlds/maze.sdf)) |

Geometry and colours follow the official
[micromouse maze rules](https://micromouseonline.com/micromouse-book/mazes-and-maze-solving/):
the maze is a 16 x 16 array of 180 mm unit squares, walls are 50 mm
high and 12 mm thick with white sides and red tops, lattice posts are
12 x 12 x 50 mm, and the example world's floor is non-gloss black.

---

## 1. Maze file format

Plain text, as used by
[`micromouseonline/mazefiles`](https://github.com/micromouseonline/mazefiles):

```
o---o---o---o---o---o
|       |       |   |
o   o   o   o   o   o
|   |       |   |   |
o   o   o   o   o   o
|   |   |   | G |   |
o---o   o   o---o   o
|       |       |   |
o   o   o---o   o   o
| S |       |       |
o---o---o---o---o---o
```

* `o`   : post at a grid intersection. 
* `---` : horizontal wall segment between two posts.
* `|`   : vertical wall segment between two posts.
* `S`   : start cell (green floor tile).
* `G`   : goal cell(s) (red floor tile).

The bottom-most text row is maze row 0; the left-most column
is column 0.

---

## 2. Build

System prerequisites (Ubuntu 20.04 / 22.04 with Fortress installed):

```bash
sudo apt install \
  libignition-gazebo6-dev libignition-plugin-dev \
  libignition-common4-dev libignition-math6-dev \
  libignition-msgs8-dev libignition-transport11-dev \
  libsdformat12-dev cmake build-essential
```

Build:

```bash
git clone https://github.com/svaibhav101/mazegen_ign_gazebo.git
cd mazegen_ign_gazebo
mkdir build
cmake -S . -B build
cmake --build build -j
```

This produces `build/libgazebo_mazegen_plugin.so`.

### Run the unit tests

The maze-file parser is covered by a CTest target (enabled by default):

```bash
ctest --test-dir build --output-on-failure
# or run the binary directly: build/test_maze_parser
```

### Generate API docs (optional)

If Doxygen is installed, a `doc` target renders the in-source docs:

```bash
cmake --build build --target doc
# output: build/docs/html/index.html
```

---

## 3. Run the bundled example (no install)

From the repo root, source the helper that exports the two search paths:

```bash
source setup.bash          # sets IGN_GAZEBO_SYSTEM_PLUGIN_PATH + RESOURCE_PATH
ign gazebo -v 4 worlds/maze.sdf
```

`setup.bash` is just:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$PWD/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=$PWD:$IGN_GAZEBO_RESOURCE_PATH
```

`IGN_GAZEBO_SYSTEM_PLUGIN_PATH` lets Ignition find the `.so`;

`IGN_GAZEBO_RESOURCE_PATH` lets the plugin resolve the relative
`<maze_file>mazes/alljapan-001-1980.txt</maze_file>` path.

Example world bundled with loading the maze with the full
physics / scene-broadcaster stack:

* `worlds/maze.sdf` - minimal example world.

---

## 4. Install system-wide (use from any world)

To use the plugin from your own world files without setting
`IGN_GAZEBO_SYSTEM_PLUGIN_PATH` every time, install the library.

### 4a. Install with CMake

```bash
# Default prefix is /usr/local; override with -DCMAKE_INSTALL_PREFIX=...
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
sudo cmake --install build
```

This places:

| File | Destination |
|---|---|
| `libgazebo_mazegen_plugin.so` | `<prefix>/lib/` |

Only the shared library is installed; the bundled `worlds/` and
`mazes/` stay in the source tree. Reference maze files from your world
by absolute path, or add their directory to `IGN_GAZEBO_RESOURCE_PATH`.

### 4b. Make Ignition pick it up

If `<prefix>` is `/usr/local`, add the lib dir to the plugin search path
(once, e.g. in your `~/.bashrc`):

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/local/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

If you used a custom prefix:

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/myprefix/lib:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

For maze files referenced by relative path, also expose a resource
directory containing them, e.g. the repo's `mazes/`:

```bash
export IGN_GAZEBO_RESOURCE_PATH=/path/to/gazeboMaze:$IGN_GAZEBO_RESOURCE_PATH
```

(Absolute paths in `<maze_file>` work regardless of `IGN_GAZEBO_RESOURCE_PATH`.)

### 4c. Verify the plugin is found

```bash
ign gazebo --list-plugins 2>&1 | grep -i maze
# or simply run any world that uses it; missing plugins print
#   "[Err] [SystemLoader.cc] Failed to load system plugin..."
```

---

## 5. Using `mazegenPlugin` in your own world

Drop this `<plugin>` block inside any `<world>` element. **No other
changes** to your world are required - your existing physics, lights,
robots, and models continue to work normally.

```xml
<world name="my_world">
  <!-- your existing scene -->
  <physics name="physics" type="dart">
    <max_step_size>0.001</max_step_size>
  </physics>
  <plugin filename="ignition-gazebo-physics-system"
          name="ignition::gazebo::systems::Physics"/>
  <!-- Required: mazegenPlugin spawns the maze through the world's
       /world/<name>/create service, which is provided by UserCommands. -->
  <plugin filename="ignition-gazebo-user-commands-system"
          name="ignition::gazebo::systems::UserCommands"/>
  <plugin filename="ignition-gazebo-scene-broadcaster-system"
          name="ignition::gazebo::systems::SceneBroadcaster"/>

  <!-- drop-in maze generator -->
  <plugin filename="libgazebo_mazegen_plugin.so"
          name="mazegen_plugin::mazegenPlugin">
    <maze_file>/absolute/path/to/maze.txt</maze_file>
    <!-- All remaining params are optional; defaults shown. -->
    <cell_size>0.18</cell_size>
    <wall_thickness>0.012</wall_thickness>
    <wall_height>0.05</wall_height>
    <post_size>0.012</post_size>
    <origin>0 0 0</origin>
  </plugin>
</world>
```

### Parameters

| Tag | Type | Default | Description |
|---|---|---|---|
| `<maze_file>` | string | - (required) | Path to a `.txt` maze. Absolute, or relative to a directory on `IGN_GAZEBO_RESOURCE_PATH`. |
| `<cell_size>` | double | `0.18` | Grid cell pitch, meters. |
| `<wall_thickness>` | double | `0.012` | Wall thickness, meters. |
| `<wall_height>` | double | `0.05` | Wall height, meters. |
| `<post_size>` | double | `wall_thickness` | Side of the square corner posts, meters. |
| `<origin>` | vec3 | `0 0 0` | World position of the maze's SW corner (start-cell side). |


---

## 6. Repository layout

```bash
.
├── build                          # CMake build tree (generated; .so + tests)
├── CMakeLists.txt                 # CMake configuration
├── documentation
│   └── Doxyfile                   # Doxygen config 
├── include
│   ├── gazebo_mazegen_plugin.h    # public plugin class (mazegen_plugin::mazegenPlugin)
│   └── maze_parse.h               # Maze struct + ParseMazeFile() declaration
├── mazes                          # bundled micromouseonline-format competition mazes
├── README.md                      
├── src
│   ├── gazebo_mazegen_plugin.cpp  # plugin: parse maze, build SDF, spawn via /create
│   └── maze_parse.cpp             # maze-file parser implementation
├── test
│   └── test_maze_parser.cpp       # CTest unit tests for the parser
└── worlds
    └── maze.sdf                   # example world loading in the plugin
```

---

## 7. References & acknowledgements
This project drew inspiration from previous work within the Micromouse and Gazebo communities:

* **[micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)**
  - the plain-text maze file format this plugin parses, and the source of
  the bundled competition mazes in `mazes/`.
* **[PeterMitrano/gzmaze](https://github.com/PeterMitrano/gzmaze)** - a
  maze generator for Gazebo 11 (Gazebo Classic).
* **([Micromouse maze rules](https://micromouseonline.com/micromouse-book/mazes-and-maze-solving/))  & ([UK-micromouse-classic](https://ukmars.org/contests/contest-rules/micromouse-classic/))**
  - the official geometry and colour spec the defaults
  follow: 16×16 grid of 180 mm cells, 50 mm x 12 mm walls with white
  sides and red tops, 12 mm lattice posts, non-gloss black floor.
* **[Ignition Gazebo Fortress](https://gazebosim.org/docs/fortress)** -
  the simulator and plugin/transport APIs (`gz-sim 6`, `ignition-plugin`,
  `ignition-transport`, `ignition-msgs`, `sdformat12`) this plugin targets.


## License

`mazegen_ign_gazebo` is made available under the MIT License. For more details, see [LICENSE](./LICENSE)