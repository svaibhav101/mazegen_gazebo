# Docker: mazegen-ign-gazebo

Everything needed to build and run the project inside a container.
For project documentation see the [main README](../README.md).

## Files

| File | Purpose |
|---|---|
| [`Dockerfile`](./Dockerfile) | Ubuntu 22.04 image with Ignition Fortress, builds the plugin at image-build time |
| [`entrypoint.sh`](./entrypoint.sh) | Copies the world SDF, patches the maze path, sets env vars, launches `ign gazebo` |
| [`docker-compose.yml`](./docker-compose.yml) | Convenience Compose wrapper with rendering and GPU options as documented comments |

## Prerequisites

- Docker Engine >= 20.10
- A running X server (Xorg or XWayland) on the host

## Quick start

> **All commands must be run from the repository root** (the directory containing
> `CMakeLists.txt`), not from inside `docker/`. The build context must be the
> repo root so Docker can copy the source files.

### 1. Allow X11 access

```bash
xhost +local:docker
```

### 2. Build the image

```bash
docker build -f docker/Dockerfile -t mazegen-ign-gazebo .
```

Or with Compose (also builds if the image does not exist):

```bash
docker compose -f docker/docker-compose.yml build
```

### 3. Run

**Default maze** (`mazes/allamerica2013.txt`):

```bash
docker compose -f docker/docker-compose.yml up
```

**Specific bundled maze:**

```bash
docker compose -f docker/docker-compose.yml run --rm mazegen mazes/alljapan-001-1980.txt
```

**Custom maze file from the host** -- mount it into `/workspace/mazes/` and pass the path as an argument:

```bash
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/your/maze.txt:/workspace/mazes/custom.txt \
  mazegen-ign-gazebo mazes/custom.txt
```

## Rendering

The default configuration uses **Mesa LLVMpipe (software rendering)**.
This works on any machine regardless of GPU and requires no extra drivers
or toolkits. It is fast enough for Gazebo's Ogre2 viewport.

Hardware GPU acceleration is available but optional. Open
`docker/docker-compose.yml` and follow the comments:

| Your hardware | What to change in `docker-compose.yml` |
|---|---|
| Any (default) | Nothing -- `LIBGL_ALWAYS_SOFTWARE=1` is already set |
| Intel or AMD GPU | Comment out `LIBGL_ALWAYS_SOFTWARE` and `MESA_GL_VERSION_OVERRIDE`; uncomment the `devices` and `group_add` block |
| NVIDIA GPU | Comment out `LIBGL_ALWAYS_SOFTWARE` and `MESA_GL_VERSION_OVERRIDE`; uncomment the `deploy` block; install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) first |

For `docker run` without Compose:

```bash
# Software rendering -- works on any machine (default)
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  mazegen-ign-gazebo

# Intel / AMD hardware GPU
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  --group-add video \
  mazegen-ign-gazebo

# NVIDIA hardware GPU (requires nvidia-container-toolkit)
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --gpus all \
  mazegen-ign-gazebo
```

## How it works

`entrypoint.sh` receives an optional maze-file path as `$1`.
It creates a temporary copy of `worlds/maze.sdf`, patches the `<maze_file>`
element to point to the chosen maze, exports `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`
and `IGN_GAZEBO_RESOURCE_PATH` so Ignition can locate the compiled plugin and
maze assets, then execs `ign gazebo`.
