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
alias docker-compose-up='cd ~/ros2_humble_ws && docker-compose up -d'
alias docker-compose-attach='docker-compose exec ros2_humble /bin/bash'
alias docker-compose-down='cd ~/ros2_humble_ws && docker-compose down'
```

## Running ROS2
In the container, the workspace should be fully build. Simply source it before using.
```bash
source install/setup.bash
```