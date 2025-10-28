#FROM ros:humble-ros-base
FROM ghcr.io/kas-lab/suave:main

SHELL ["/bin/bash", "-c"]

ARG USERID=1000
ENV ROS_DISTRO=humble

USER root

# Add ROS repository and update the GPG keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update \
    && apt-get install -y python3-pip \
    && apt-get install -y ros-humble-cv-bridge \
    && apt-get clean

RUN pip install --no-cache-dir \
    --extra-index-url https://download.pytorch.org/whl/cpu \
    torch 

RUN pip install --no-cache-dir torchvision==0.21.0

WORKDIR /bt_workspace

RUN mkdir src

WORKDIR /bt_workspace/src

# Clone behaviour Tree cpp library
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && cd BehaviorTree.CPP && git checkout 4.6.2

RUN git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git

WORKDIR /bt_workspace/

RUN rosdep update

# Install ros dependencies
RUN rosdep install --from-paths src -y --ignore-src

# Build the library and source it
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

RUN source /bt_workspace/install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> /home/kasm-user/.bashrc  &&  echo "source /bt_workspace/install/setup.bash" >> /home/kasm-user/.bashrc

ADD ./ros_ws/src /ros_ws/src

WORKDIR /ros_ws

# Clear ENTRYPOINT of SUAVE base image
ENTRYPOINT [""]

CMD ["--tail-log"]
