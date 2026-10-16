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

# rosbridge_server and Julia are for go2_map_viewer_julia, a standalone
# dashboard script, not a colcon package — so there's no package.xml to
# declare rosbridge_server as a rosdep key, and it's installed directly here
# instead. Placed this late so editing either doesn't invalidate the slow
# pip/rosdep layers above. Only the Project.toml/Manifest.toml are copied in
# (not map_viewer.jl itself, which stays live-mounted with the rest of the
# repo), so this layer only reruns when the Julia dependency set changes.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://install.julialang.org | sh -s -- --yes
ENV PATH="/root/.juliaup/bin:${PATH}"
# Pinned to match go2_map_viewer_julia/Manifest.toml's julia_version - an
# unpinned install otherwise lets a future rebuild silently pick up
# whatever Julia version juliaup's "release" channel resolves to at build
# time, which can drift from the committed Manifest.toml.
RUN juliaup add 1.13.1 && juliaup default 1.13.1

COPY go2_map_viewer_julia/Project.toml go2_map_viewer_julia/Manifest.toml \
    /opt/go2_map_viewer_julia/
RUN cd /opt/go2_map_viewer_julia && \
    julia --project=. -e 'import Pkg; Pkg.instantiate(); Pkg.precompile()'

COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
COPY docker/dev_aliases.sh /etc/dev_aliases.sh
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /etc/dev_aliases.sh" >> /root/.bashrc

# Install piper voice
RUN python3 -m piper.download_voices en_US-lessac-medium --data-dir .

ENV ROBOT_IP=""
ENV CONN_TYPE="webrtc"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
