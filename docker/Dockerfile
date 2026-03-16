FROM ubuntu:22.04 AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG GZ_DISTRO=fortress
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000

SHELL ["/bin/bash", "-c"]

ENV LANG=en_GB.UTF-8
ENV LC_ALL=en_GB.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV GZ_DISTRO=${GZ_DISTRO}
ENV GZ_VERSION=${GZ_DISTRO}
ENV ROS_WS=/workspaces/innex1-rover

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    ca-certificates \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    bash-completion \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-numpy \
    libegl1 \
    libgl1 \
    libnss3 \
    mesa-utils \
 && locale-gen en_GB.UTF-8 \
 && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8 \
 && add-apt-repository -y universe \
 && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list \
 && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-teleop \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

RUN groupadd --gid ${GID} ${USERNAME} \
 && useradd --uid ${UID} --gid ${GID} -m -s /bin/bash ${USERNAME} \
 && mkdir -p ${ROS_WS} \
 && chown -R ${USERNAME}:${USERNAME} ${ROS_WS}

WORKDIR ${ROS_WS}

COPY src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep install \
    --rosdistro ${ROS_DISTRO} \
    --from-paths src \
    --ignore-src \
    --dependency-types buildtool \
    --dependency-types build \
    --dependency-types build_export \
    --dependency-types buildtool_export \
    --dependency-types exec \
    -r -y \
 && pip3 install --no-cache-dir open3d

COPY docker/scripts/dev_entrypoint.sh /usr/local/bin/dev_entrypoint.sh
RUN chmod +x /usr/local/bin/dev_entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/usr/local/bin/dev_entrypoint.sh"]
CMD ["bash"]

FROM base AS sim-dev

ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility
ENV NVIDIA_VISIBLE_DEVICES=all
