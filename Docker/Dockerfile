# Use the official ROS 2 Docker image as base
FROM ros:foxy

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip

# Clone your ROS 2 package repository
WORKDIR /workspace/src
RUN git config --global user.email "kacpersobczak152@gmail.com" && \
    git config --global user.name "Kacper-So" && \
    git clone https://github.com/Kacper-So/f1thnth_obstacle_avoidance_MPCC && \
    git clone https://github.com/Kacper-So/ros2-obstacle-detection-on-Occupancy-Map

# Install TurtleBot3
RUN apt-get update && apt-get install -y \
    ros-foxy-turtlebot3*

# Set up environment variables for TurtleBot3
ENV TURTLEBOT3_MODEL=waffle

# Define the entrypoint
CMD ["bash"]