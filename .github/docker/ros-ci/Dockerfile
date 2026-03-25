FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
  && rm -rf /var/lib/apt/lists/*
