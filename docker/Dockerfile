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

FROM ros:humble-ros-base

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
# several GB of CUDA libraries. ultralytics (object_detection's YOLO) also
# depends on torch; because it is in the same requirements.txt and the same pip
# command, pip resolves it against these CPU wheels instead of pulling CUDA torch.
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

# Gazebo Fortress and its ROS bridge, for the go2_sim simulator. go2_sim's
# package.xml also lists the ros_gz_* keys (rosdep installs those); this pulls in
# the whole ros-humble-ros-gz set, including the `ign gazebo` command line tool.
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
CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
