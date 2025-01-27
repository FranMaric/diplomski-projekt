FROM lmark1/uav_ros_simulation:focal-bin-0.2.1

ARG HOME=/root
ARG CATKIN_WORKSPACE=sim_ws
ARG USER=root

WORKDIR $HOME/$CATKIN_WORKSPACE/src/dipl_proj_2024
COPY . .

# --------------- build ROS packages ---------------
WORKDIR $HOME/$CATKIN_WORKSPACE/src
RUN catkin build --limit-status-rate 0.2 --jobs ${nproc-1}

# --------------- install programs ---------------
RUN sudo apt-get update && sudo apt-get install -q -y nano
RUN pip install -r dipl_proj_2024/requirements.txt

ARG ROS_HOSTNAME=localhost.local
ARG ROS_MASTER_URI=http://localhost.local:11311
ARG ROS_IP=localhost.local