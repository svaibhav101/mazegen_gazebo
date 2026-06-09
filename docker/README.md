# Docker — mazegen-ign-gazebo

Everything needed to build and run the project inside a container.
For project documentation see the [main README](../README.md).

## Files

| File | Purpose |
|---|---|
| [`Dockerfile`](./Dockerfile) | Ubuntu 22.04 image with Ignition Fortress, builds the plugin at image-build time |
| [`entrypoint.sh`](./entrypoint.sh) | Patches the world SDF with the chosen maze file, sets env vars, launches `ign gazebo` |
| [`docker-compose.yml`](./docker-compose.yml) | Convenience Compose wrapper — wires up X11, GPU options documented as comments |

## Prerequisites

- Docker Engine ≥ 20.10
- A running X server (Xorg or XWayland) on the host
- *(Optional)* nvidia-container-toolkit for NVIDIA GPU acceleration

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

Or with Compose (also builds if the image doesn't exist):

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
docker compose -f docker/docker-compose.yml run mazegen mazes/alljapan-001-1980.txt
```

**Custom maze file from the host** — mount it into `/workspace/mazes/` and pass the path as an argument:

```bash
docker run --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/your/maze.txt:/workspace/mazes/custom.txt \
  mazegen-ign-gazebo mazes/custom.txt
```

Or uncomment the volume line in `docker-compose.yml` and use `docker compose run`.

## GPU / hardware-accelerated rendering

Ignition Fortress uses Ogre2, which requires OpenGL. Uncomment the relevant
block in [`docker-compose.yml`](./docker-compose.yml), or pass the flags
directly to `docker run`:

| GPU | `docker run` flags |
|---|---|
| Intel / AMD | `--device /dev/dri --group-add video` |
| NVIDIA | `--gpus all` *(requires nvidia-container-toolkit)* |

## How it works

`entrypoint.sh` receives an optional maze-file path as `$1`.  
It patches the `<maze_file>` element in `worlds/maze.sdf` at container start,
exports `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` and `IGN_GAZEBO_RESOURCE_PATH` so
Ignition can locate the compiled plugin and the maze assets, then execs
`ign gazebo worlds/maze.sdf`.
