FROM lmark1/uav_ros_simulation:focal-bin-0.2.1

ARG HOME=/root
ARG CATKIN_WORKSPACE=sim_ws
ARG USER=root

RUN apt-get install openssh-client git

# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
WORKDIR $HOME/$CATKIN_WORKSPACE/src
RUN --mount=type=ssh git clone git@github.com:JakobDomislovic/PredDiplProj2024.git

# --------------- build ROS packages ---------------
WORKDIR $HOME/$CATKIN_WORKSPACE/src
RUN catkin build --limit-status-rate 0.2 --jobs ${nproc-1}

# --------------- install programs ---------------
RUN sudo apt-get update && sudo apt-get install -q -y \
    nano \
    vim

ARG ROS_HOSTNAME=localhost.local
ARG ROS_MASTER_URI=http://localhost.local:11311
ARG ROS_IP=localhost.local

WORKDIR $HOME/$CATKIN_WORKSPACE/src/PredDiplProj2024