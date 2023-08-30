# syntax=docker/dockerfile:1
ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}-ros-base AS upstream
# Restate for later use
ARG ROS_DISTRO
ARG REPO

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# install development tools
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
        apt-utils \
        ccache \
        clang \
        clang-format \
        clang-tidy \
        cmake \
        git \
        lld \
        llvm \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        python3-colcon-common-extensions \
        python3-colcon-lcov-result \
        python3-colcon-coveragepy-result \
        python3-colcon-mixin \
        python3-rosdep \
        python3-vcstool \
        vim \
        wget \
        ssh-client \
    && rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
    pre-commit

# copy source to install repo dependencies
WORKDIR /ws
COPY . ./src/${REPO}
# install repo dependencies
RUN rosdep update && apt-get update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

FROM upstream AS development

ARG UIDGID
ARG USER

# fail build if args are missing
RUN if [ -z "$UIDGID" ]; then echo '\nERROR: UIDGID not set. Run \n\n \texport UIDGID=$(id -u):$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# chown working directory to user
RUN mkdir -p /home/${USER}/ws && chown -R ${UIDGID} /home/${USER}
