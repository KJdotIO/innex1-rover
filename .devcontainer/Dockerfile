FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    git \
    iproute2 \
    jq \
    less \
    openssh-client \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    sudo \
  && rm -rf /var/lib/apt/lists/*

ARG USERNAME=rover
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid "$USER_GID" "$USERNAME" \
  && useradd --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME" \
  && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME" \
  && chmod 0440 "/etc/sudoers.d/$USERNAME"

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
  && for attempt in 1 2 3; do \
    rosdep update --rosdistro humble && break; \
    if [ "$attempt" = 3 ]; then exit 1; fi; \
    sleep "$((attempt * 5))"; \
  done

USER $USERNAME
WORKDIR /workspace/innex1-rover

RUN echo "source /opt/ros/humble/setup.bash" >> "/home/$USERNAME/.bashrc" \
  && echo "if [ -f install/setup.bash ]; then source install/setup.bash; fi" >> "/home/$USERNAME/.bashrc"
