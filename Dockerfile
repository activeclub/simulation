FROM ros:jazzy

# install ros package
RUN apt-get update \
  && apt-get install -y \
  ros-${ROS_DISTRO}-xacro

WORKDIR /app
