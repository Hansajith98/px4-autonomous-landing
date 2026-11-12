FROM px4io/px4-dev-ros2-gazebo:main-jazzy

ENV GZ_CONFIG_PATH="/usr/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-foxglove-bridge \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/autonomous_landing_ws

CMD ["/bin/bash"]
