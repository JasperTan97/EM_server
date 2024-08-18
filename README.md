# EM_server

The goal of this repo is to set up a quick way to immediately start up the server for the EdyMobile. All files in this repo will be sufficient to set up the server.

## Pre set-up

Ensure docker is already installed
```sh
sudo apt update
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker
```
You can confirm installation with
```sh
docker --version
```

## First time set-up of this repo

Now simply build with (don't forget to give execute access)
```sh
./build.sh
```

OR we build our base container. Here, all relevant ros2 files and python libraries are installed.

```sh
cd docker_base
docker build -t ros2_humble .
cd ..
```

Next, we will set up the permanent volume (src folder), and open usb ports if necessary.
```sh
docker-compose build
```

## Running the docker container
We can start the container as:
```sh
cd ~/ros2_humble_ws && docker-compose up -d
```
We can attach the terminal to the container with:
```sh
docker-compose exec ros2_humble /bin/bash
```
Typing exit only detaches the terminal from the container. It will still run in the background. To fully end the container, do:
```sh
cd ~/ros2_humble_ws && docker-compose down
```

Some nice aliases I use in ~/.bashrc are:
```bash
alias docker-compose-up='cd ~/ros2_humble_ws && docker-compose up -d && docker-compose exec ros2_humble /bin/bash'
alias docker-compose-attach='docker-compose exec ros2_humble /bin/bash'
alias docker-compose-down='cd ~/ros2_humble_ws && docker-compose down'
```

### Using ign gazebo (for unix OS)
Gazebo fortress has been configured in this container as well. However, we must allow docker to access the X-server, which allows GUI on a linux system. This can be done by the following line:
```bash
xhost +local:docker
```
This can also be added to your local machine's bashrc file for convenience. 

Recall that gazebo now starts with the following command:
```bash
ign gazebo
```

### Using ign gazebo (for windows)
This is the work for someone who uses windows. You may have to remove the following lines in the docker compose, but I have not tested this:
```yml
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw  # Mount X11 socket for display forwarding
      - $HOME/.Xauthority:/root/.Xauthority:ro
    environment:
      - DISPLAY=${DISPLAY}  # Forward the DISPLAY environment variable
      - QT_X11_NO_MITSHM=1  # Necessary for Qt applications to run in the container
      - LIBGL_ALWAYS_SOFTWARE=1  # Force software rendering
      - XAUTHORITY=$XAUTH
```
If you do not need gazebo, remove those lines anyway, and also these in the Dockerfile in the present working directory.
```Dockerfile
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
```

## Running ROS2
In the container, the workspace should be fully build. Simply source it before using.
```bash
source install/setup.bash
```