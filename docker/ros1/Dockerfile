FROM ros:noetic

RUN apt-get update \
  && apt-get install -y \
  ros-${ROS_DISTRO}-xacro \
  python3-catkin-tools

WORKDIR /app
