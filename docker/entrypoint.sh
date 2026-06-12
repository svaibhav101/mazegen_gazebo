#!/bin/bash
set -euo pipefail

# Select world SDF.  Pass "single" (or any first arg) to use maze.sdf with a
# single maze; omit the arg (default) to use two_mazes.sdf.
WORLD="${1:-two_mazes}"

if [ "$WORLD" = "single" ]; then
    WORLD_SDF_SRC=/workspace/worlds/maze.sdf
else
    WORLD_SDF_SRC=/workspace/worlds/two_mazes.sdf
fi

# Work on a per-container copy so multiple containers don't race on the same file.
WORLD_SDF=$(mktemp /tmp/maze_world_XXXXXX.sdf)
cp "$WORLD_SDF_SRC" "$WORLD_SDF"
trap 'rm -f "$WORLD_SDF"' EXIT

# Make the compiled plugin and resources visible to Ignition Gazebo.
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/workspace/build${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}}
export IGN_GAZEBO_RESOURCE_PATH=/workspace${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}

exec ign gazebo "$WORLD_SDF" -r -v 4
