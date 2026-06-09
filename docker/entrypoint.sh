#!/bin/bash
set -e

# If a maze file is passed as the first argument, patch the SDF world to use it.
# The file is expected to be mounted into /workspace/mazes/ by the caller.
MAZE_FILE="${1:-mazes/allamerica2013.txt}"

# Rewrite the <maze_file> element in the world SDF so it points to the chosen maze.
WORLD_SDF=/workspace/worlds/maze.sdf
sed -i "s|<maze_file>.*</maze_file>|<maze_file>${MAZE_FILE}</maze_file>|" "$WORLD_SDF"

# Make the compiled plugin and resources visible to Ignition Gazebo.
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/workspace/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}
export IGN_GAZEBO_RESOURCE_PATH=/workspace:${IGN_GAZEBO_RESOURCE_PATH:-}

exec ign gazebo "$WORLD_SDF" -v 4
