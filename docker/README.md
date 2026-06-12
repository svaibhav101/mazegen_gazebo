# Docker: mazegen-gazebo

Everything needed to build and run the **mazegen_gazebo** plugin inside a container.
For the full project documentation see the [main README](../README.md).

## Files

| File | Purpose |
|---|---|
| [`Dockerfile`](./Dockerfile) | Ubuntu 22.04 image with Ignition Fortress; builds the plugin at image-build time |
| [`entrypoint.sh`](./entrypoint.sh) | Patches the maze path, sets env vars, launches `ign gazebo` |
| [`docker-compose.yml`](./docker-compose.yml) | Convenience Compose wrapper with rendering and GPU options |

---

## Prerequisites

- Docker Engine ≥ 20.10
- A running X server (Xorg or XWayland) on the host

> **All commands must be run from the repository root** (the directory containing
> `CMakeLists.txt`), not from inside `docker/`. The build context must be the
> repo root so Docker can copy the source files.

---

## Quick start

### 1. Allow X11 access

```bash
xhost +local:docker
```

### 2. Build the image

```bash
docker build -f docker/Dockerfile -t mazegen-gazebo .
```

Or with Compose (also builds if the image does not exist):

```bash
docker compose -f docker/docker-compose.yml build
```

### 3. Run

**Two mazes side by side** (default — `worlds/two_mazes.sdf`):

```bash
docker compose -f docker/docker-compose.yml up
```

**Single maze** (`worlds/maze.sdf`):

```bash
docker compose -f docker/docker-compose.yml run --rm mazegen single
```

**Using `docker run` directly:**

```bash
# Two mazes (default)
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  mazegen-gazebo

# Single maze
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  mazegen-gazebo single
```

---

## Rendering

The default configuration uses **Mesa LLVMpipe (software rendering)**.
This works on any machine regardless of GPU and requires no extra drivers.
It is fast enough for Gazebo's Ogre2 viewport.

Hardware GPU acceleration is optional. Open `docker/docker-compose.yml` and
follow the comments:

| Hardware | What to change in `docker-compose.yml` |
|---|---|
| Any (default) | Nothing - `LIBGL_ALWAYS_SOFTWARE=1` is already set |
| Intel or AMD GPU | Comment out `LIBGL_ALWAYS_SOFTWARE` and `MESA_GL_VERSION_OVERRIDE`; uncomment the `devices` and `group_add` block |
| NVIDIA GPU | Comment out `LIBGL_ALWAYS_SOFTWARE` and `MESA_GL_VERSION_OVERRIDE`; uncomment the `deploy` block; install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) first |

For `docker run` without Compose:

```bash
# Software rendering (default - works on any machine)
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  mazegen-gazebo

# Intel / AMD hardware GPU
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  --group-add video \
  mazegen-gazebo

# NVIDIA hardware GPU (requires nvidia-container-toolkit)
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --gpus all \
  mazegen-gazebo
```

---

## How it works

`entrypoint.sh` selects a world SDF based on the optional first argument:

- No argument (default): uses `worlds/two_mazes.sdf` — spawns `alljapan1980` and `allamerica2013` side by side.
- `single`: uses `worlds/maze.sdf` — spawns a single `allamerica2013` maze.

It makes a temporary per-container copy of the chosen SDF, exports
`IGN_GAZEBO_SYSTEM_PLUGIN_PATH` and `IGN_GAZEBO_RESOURCE_PATH` so Ignition
can find the compiled plugin and maze assets, then execs `ign gazebo`.

The Docker image does **not** include the ROS 2 package (`micro_mouse_robot`).
To run the navigation nodes, build from source on a host with ROS 2 Humble
installed - see [main README § Build from source](../README.md#3-build-from-source).
