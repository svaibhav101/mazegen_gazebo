FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Ignition Fortress (ignition-gazebo6) is in the OSRF apt repo on Ubuntu 22.04
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    # Ignition Fortress runtime + dev
    ignition-fortress \
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
    # X11 forwarding support
    libx11-6 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy project source (excluding build artefacts via .dockerignore)
COPY CMakeLists.txt .
COPY include/ include/
COPY src/ src/
COPY test/ test/
COPY worlds/ worlds/
COPY mazes/ mazes/

# Build the plugin
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j$(nproc) \
    && cmake --install build --prefix /workspace/install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
