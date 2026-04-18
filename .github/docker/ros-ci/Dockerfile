FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Tooling used by CI and smoke tests; jq avoids actionlint failures when workflows run in-container.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    jq \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
  && rm -rf /var/lib/apt/lists/*

# Bake workspace rosdeps so CI jobs do not pull large dependency sets from scratch each run.
WORKDIR /tmp/innex1-ws
COPY src ./src
RUN apt-get update \
  && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
  && rosdep update --rosdistro humble \
  && rosdep install --from-paths src -y --ignore-src \
  && rm -rf /var/lib/apt/lists/* \
  && rm -rf /tmp/innex1-ws

WORKDIR /
