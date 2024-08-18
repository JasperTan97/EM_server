# Use the custom ROS 2 Humble base image as a parent image
FROM ros2_humble_base

# Set the working directory
WORKDIR /ros2_ws

# Copy the ROS 2 workspace
COPY src /ros2_ws/src

# Install ROS2 dependencies
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Install gazebo using a temporary terminal
RUN bash -c "\
    apt-get update && \
    apt-get install -y lsb-release gnupg && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null && \
    echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main' > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress && \
    apt-get clean && rm -rf /var/lib/apt/lists/*"

# Ensure Ignition Gazebo (Fortress) is in the PATH
ENV PATH="/usr/local/bin:/usr/bin:/opt/ros/humble/bin:${PATH}"

# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# Source the workspace setup script
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Add aliases
RUN echo "alias build_and_source='colcon build && source install/setup.bash'" >> ~/.bashrc
RUN echo "alias apt_update_upgrade='apt-get update && apt-get update -y'" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export PYTHONPATH=/opt/ros/humble/lib/python3/dist-packages" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp " >> ~/.bashrc # needed for rpi and nav2 communication, for which fastdds doesnt work
RUN echo "export ROS_DOMAIN_ID=10 " >> ~/.bashrc # same than the one on the mobile robot

# Set the entry point
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source ~/.bashrc && exec bash"]

