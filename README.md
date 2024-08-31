# Master3_bt3


>**Note:** This repository is under construction. It has no working functionality yet.

## Overview

Some behavior tree v3.8 playground

## Prerequisites

* Installed ROS 2 humble distribution

## Installation

<!-- First install required development tools

``` bash
sudo apt install python3-vcstool python3-colcon-common-extensions git wget
``` -->

Then create a new workspace and load the git repositories which are required.

``` bash
mkdir -p ~/master3_bt3_ws/src
cd ~/master3_bt3_ws/src
wget https://raw.githubusercontent.com/cord-burmeister/master3_bt3/main/master3_bt3.yaml
vcs import < master3_bt3.yaml
```

### Install dependencies

``` bash
cd ~/master3_bt3_ws
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
```

### Build the project

``` bash
colcon build 
```

### Source the workspace

``` bash
. ~/master3_bt3_ws/install/setup.sh
```

## Starting commands

``` bash
ros2 run groot Groot
```

