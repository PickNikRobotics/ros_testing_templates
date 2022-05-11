FROM ros:melodic
MAINTAINER Griswald Brooks griswald.brooks@gmail.com

# Prevent the interactive wizards from stopping the build
ARG DEBIAN_FRONTEND=noninteractive

ARG USER
ARG UID
ARG GID

# Get the basics
RUN apt update -y &&         \
    apt install -y           \
             build-essential \
             cmake           \
             curl            \
             git             \
             lsb-core        \
             python3         \
             python3-pip     \
             sudo            \
             wget

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -

RUN apt update -y &&             \
    apt install -y               \
             python-catkin-tools \
             clang-format-10     \
             clang-tidy-10

RUN pip3 install pre-commit

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash'
RUN addgroup --gid $GID $USER
RUN adduser --disabled-password --gecos '' --uid $UID --gid $GID $USER
USER $USER
RUN mkdir -p /home/$USER/ws
RUN cd /home/$USER/ws && \
    catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_CLANG_TIDY="clang-tidy-10;-extra-arg=-Wno-unknown-warning-option"
