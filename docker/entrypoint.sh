#!/bin/bash
set -euo pipefail

MAZE_FILE="${1:-mazes/allamerica2013.txt}"
WORLD_SDF_SRC=/workspace/worlds/maze.sdf

# Work on a per-container copy of the world SDF so that:
#   - restarting the container with a different maze arg always works, and
#   - multiple containers sharing the same image don't race on the same file.
WORLD_SDF=$(mktemp /tmp/maze_world_XXXXXX.sdf)
cp "$WORLD_SDF_SRC" "$WORLD_SDF"
trap 'rm -f "$WORLD_SDF"' EXIT

# Patch the <maze_file> element to point to the chosen maze.
sed -i "s|<maze_file>.*</maze_file>|<maze_file>${MAZE_FILE}</maze_file>|" \
    "$WORLD_SDF"

# Make the compiled plugin and resources visible to Ignition Gazebo.
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/workspace/build${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}}
export IGN_GAZEBO_RESOURCE_PATH=/workspace${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}

exec ign gazebo "$WORLD_SDF" -v 4
