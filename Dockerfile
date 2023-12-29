ARG FROM_IMAGE=ros:iron
ARG LAB_WS=lab1_ws
ARG ROS_WS=/opt/ros/${LAB_WS}/

# base image
FROM $FROM_IMAGE

# Copy folder to docker container
ARG ROS_WS
ARG LAB_WS
COPY  ./${LAB_WS}/ .${ROS_WS}

# Install the needed dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update \
    && rosdep install -y --from-paths ./${ROS_WS}/src --ignore-src

# Build the ROS packages
WORKDIR /${ROS_WS}/
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build 
    
# source entrypoint setup
ENV ROS_WS $ROS_WS
RUN sed --in-place --expression '$isource "/$ROS_WS/install/setup.bash"' /ros_entrypoint.sh

# Run the nodes
CMD ["ros2", "launch", "rc_car_bringup", "start_rc_car.launch.py"]