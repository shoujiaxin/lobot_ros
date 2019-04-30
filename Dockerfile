FROM ros:melodic

RUN apt-get update && \
    apt-get install -y libudev-dev && \
    apt-get install -y ros-melodic-moveit && \
    apt-get install -y ros-melodic-ros-control && \
    apt-get install -y ros-melodic-ros-controllers && \
    apt-get install -y ros-melodic-moveit-visual-tools
