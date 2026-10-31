# Mac-only dependency image. Keep the team's docker/Dockerfile unchanged.
# Uses the same requirements.txt, package.xml files, entrypoint and aliases.
# Development image: holds only the dependencies. The source is not baked in;
# docker compose mounts the repo's src/ folder at /ros2_ws/src and entrypoint.sh builds it
# incrementally on start, so code changes never need an image rebuild.

# Collect just the package.xml files, keeping their directory layout. rosdep
# only reads these, and copying them (instead of the whole tree) into the main
# stage means the slow rosdep layer reruns only when a dependency changes.
FROM ros:humble-ros-base AS manifests
COPY src /src
RUN mkdir /manifests && cd /src && \
    find . -name package.xml -not -path '*/external_lib/*' \
      -exec cp --parents {} /manifests \;

FROM ros:humble-ros-base AS mac

# Install basic requirements
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-humble-rviz2 \
    software-properties-common \
    ros-humble-rmw-cyclonedds-cpp \
    ffmpeg

# Don't keep pip's download cache in the image (it roughly doubles the size of
# the requirements layer), and don't let nodes write root-owned __pycache__
# folders into the mounted source on the host.
ENV PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /ros2_ws

# torch/torchvision come from the CPU-only index: the default PyPI wheels bundle
# several GB of CUDA libraries.
# requirements.txt pulls in an unpinned setuptools that's too new for colcon-core
# (colcon-core requires setuptools<80). The base image's system 'packaging' is also
# too old for that setuptools (missing canonicalize_version's strip_trailing_zero
# kwarg), which breaks package builds unless both are pinned. Pinning them in the
# same install lets pip resolve everything together.
COPY requirements.txt /tmp/requirements.txt
RUN pip install --extra-index-url https://download.pytorch.org/whl/cpu \
    -r /tmp/requirements.txt 'setuptools<80' 'packaging>=24.2'

# Install ROS dependencies. Refresh the apt lists in this same step: the ones from
# the first layer are cached and go stale, and Ubuntu deletes superseded package
# versions, so rosdep's apt installs 404 without it.
COPY --from=manifests /manifests /ros2_ws/src
RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Gazebo Fortress and its ROS bridge, for the go2_sim simulator (same layer as
# docker/Dockerfile). Published for both arm64 and amd64.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz && \
    rm -rf /var/lib/apt/lists/*

COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
COPY docker/dev_aliases.sh /etc/dev_aliases.sh
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /etc/dev_aliases.sh" >> /root/.bashrc

ENV ROBOT_IP=""
ENV CONN_TYPE="webrtc"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]

# Mac GUI dependencies: render in Linux and display through XQuartz.
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb xauth x11-utils && \
    arch="$(dpkg --print-architecture)" && \
    case "$arch" in \
      arm64) checksum=9ac238e8a18c06d84ef65444a591a7a5bb4c4ce9e8cfdd92f8a2aea35edbb5d7 ;; \
      amd64) checksum=df3f7788ce41b182a47c0d298e5cd6d2d63579522cb41825970b7726e825485e ;; \
      *) echo "Unsupported VirtualGL architecture: $arch" >&2; exit 1 ;; \
    esac && \
    curl -fsSL "https://github.com/VirtualGL/virtualgl/releases/download/3.1.5/virtualgl_3.1.5_${arch}.deb" -o /tmp/virtualgl.deb && \
    echo "$checksum  /tmp/virtualgl.deb" | sha256sum -c - && \
    apt-get install -y --no-install-recommends /tmp/virtualgl.deb && \
    rm -f /tmp/virtualgl.deb && rm -rf /var/lib/apt/lists/*
COPY --chmod=755 docker/mac/mac-gui /usr/local/bin/mac-gui
COPY docker/mac/colcon.yaml /etc/colcon.mac.yaml

