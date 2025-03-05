# Base image
FROM ros:humble

# Install dependencies
RUN apt update && apt install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Copy package into container
WORKDIR /root/ws
COPY ../../src/services/ros/actuators/thrusters /root/ws/src/thruster_controller

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    apt update && rosdep install --from-paths src --ignore-src -r -y

# Build the package
RUN . /opt/ros/humble/setup.sh && colcon build

# Source ROS and entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/ws/install/setup.bash && ros2 launch thruster_controller controller.launch.py"]
