FROM ubuntu:22.04

# --- Build-time defaults (override with --build-arg) -------------------------
# IGNITION_DISTRO: matches the OSRF meta-package name and the apt codename.
ARG IGNITION_DISTRO=fortress
# UBUNTU_CODENAME: must match the FROM base image.
ARG UBUNTU_CODENAME=jammy

# --- Runtime environment ------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    # /dev/dri if you want hardware acceleration.
    LIBGL_ALWAYS_SOFTWARE=1 \
    MESA_GL_VERSION_OVERRIDE=3.3 \
    QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo \
       "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        ${UBUNTU_CODENAME} main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        # Ignition Fortress runtime + dev libraries
        ignition-${IGNITION_DISTRO} \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        libignition-common4-dev \
        libignition-math6-dev \
        libignition-msgs8-dev \
        libignition-transport11-dev \
        libsdformat12-dev \
        # Build tools
        cmake \
        build-essential \
        # X11 client libraries (display forwarding)
        libx11-6 \
        libxext6 \
        libxrender1 \
        # Mesa OpenGL: provides both hardware (DRI) and software (LLVMpipe)
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        libgles2-mesa \
        mesa-utils \
        # Mesa Vulkan + LLVMpipe software renderer
        mesa-vulkan-drivers \
        libvulkan1 \
    && rm -rf /var/lib/apt/lists/*

# --- 2. Build the plugin -------------------------------------------------------
WORKDIR /workspace

# Copy only the files needed for the build (see .dockerignore).
COPY CMakeLists.txt .
COPY include/ include/
COPY src/     src/
COPY test/    test/
COPY worlds/  worlds/
COPY mazes/   mazes/

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)" \
    && cmake --install build --prefix /workspace/install

# --- 3. Entrypoint -------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
