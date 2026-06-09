# Architecture notes {#architecture}

This page records *why* the plugin is shaped the way it is. The headers
and source files describe *what* each piece does - read those first.

## Layering

```
                +-----------------------------+
   SDF world -->|     MazegenPlugin (plugin)   |--> /world/<name>/create
                |   (ISystemConfigure only)   |     (Ignition Transport)
                +--------------+--------------+
                               |
                               v
                +-----------------------------+
                |   BuildMazeSdf (internal)   |
                |   - merges collinear walls  |
                |   - emits start/goal tiles  |
                +--------------+--------------+
                               |
                               v
                +-----------------------------+
                |       ParseMazeFile         |
                |  micromouseonline .txt ->   |
                |  Maze { hWall, vWall, ... } |
                +-----------------------------+
```

Only `MazegenPlugin` is exposed as a plugin symbol. The parser and the
SDF builder are internal - the builder lives in an anonymous namespace
inside `mazegen_plugin.cpp`, the parser sits behind `ParseMazeFile()` so
it can be unit-tested without bringing in any Ignition headers.

## Coordinate conventions

The parser and SDF builder work in a **text-natural local frame**:

- local `+X` = text-east (increasing column)
- local `+Y` = text-north (increasing row)
- row 0 is the *southern* edge of the maze

The model is placed with **no rotation** (yaw 0) and translated so the
text bottom-left (the `S` cell) lands at the SW of the user-supplied
`<origin>`. Local `+X` therefore maps to world `+X` (text-east) and
local `+Y` to world `+Y` (text-north), so the maze fills the first
quadrant. This matches Gazebo's default top-down view and the micromouse
rule that the start sits in a corner with outer walls to its west and
south.

## Why spawn via `/world/<name>/create` rather than `SdfEntityCreator`?

In Fortress' ogre2 renderer, models created directly through
`SdfEntityCreator` from inside a system plugin frequently lose their
visual `<material>` components on the way to the GUI scene - surfaces
render as flat gray. Submitting an `EntityFactory` request to the
world's `/create` service routes the model through `UserCommands`, the
same path used for `<include>` directives, which preserves the full
component set.

The plugin therefore:

1. Writes the generated SDF to a temp file in the system temp directory
   (`mazegen_<pid>_<counter>.sdf` - the counter lets several plugin
   instances in one world avoid clobbering each other's file).
2. Spawns a detached thread that retries the create service (up to 20
   attempts, 250 ms apart) - the service isn't reachable until after
   `Configure()` returns. Once the request is queued it waits ~5 s before
   deleting the temp file, since `UserCommands` reads it on a later ECM
   update.

## Material trick

Under ogre2 in Fortress, `<ambient>` and `<diffuse>` alone often look
gray; `<emissive>` is what reliably carries colour through. Every
visual emitted by `EmitMaterial()` therefore sets `diffuse` and a
scaled-down `emissive` to the same colour, so walls look *lit* rather
than glowing.

## Wall merging

Adjacent collinear walls (and the posts between them) are merged into a
single box bar by a two-pass scan over `hWall` / `vWall`. This is purely
a performance optimisation - it keeps the collision and visual counts
proportional to the number of *runs* in the maze rather than the number
of cells, which matters on the larger 32×32 maze files.
