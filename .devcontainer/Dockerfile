FROM althack/ros2:humble-dev 


ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-humble-gazebo-* \
   ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
   ros-humble-ackermann-steering-controller \
   ros-humble-control-* \
   ros-humble-rqt-robot-steering  \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
