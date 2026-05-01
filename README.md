## I am available for hire
Hello! With a background in software engineering, embedded programming, hardware design, and robotics, I'm on the lookout for new challenges.
If you're in search of someone with my skills, let's team up and create something amazing! https://x-tech.online/

# ROS Workspace

[![Build](https://github.com/ClemensElflein/open_mower_ros/actions/workflows/build-image.yaml/badge.svg)](https://github.com/ClemensElflein/open_mower_ros/actions/workflows/build-image.yaml)

This folder is the ROS workspace, which should be used to build the OpenMower ROS software.
This repository contains the ROS package for controlling the OpenMower.

There are references to other repositories (libraries) needed to build the software. This way, we can track the exact version of the packages used in each release to ensure package compatibility.
Currently, the following repositories are included:

- **slic3r_coverage_planner**: A coverage planner based on the Slic3r software for 3d printers. This is used to plan the mowing path.
- **teb_local_planner**: The local planner which allows the robot to avoid obstacles and follow the global path using kinematic constraints.
- **xesc_ros**: The ROS interface for the xESC motor controllers.

## Container images: Default vs Legacy

If your robot runs the latest OpenMower OS (v2): use the images without prefix or suffix (e.g. `latest`, `v1.2.3`).
These images only contain the OpenMower ROS stack and expect the OS to provide web and MQTT services (for example via your system’s compose setup).

If your robot runs an old version of OpenMower OS v1 (Legacy): use the legacy image.
The OS doesn't provide web and MQTT services, so the image contains nginx and mosquitto to provide these services inside the container.
The Docker images have a `-legacy` suffix or `releases-` prefix: (e.g. `releases-edge`, `v1.2.3-legacy`).

## Getting started

### Running on your machine

OpenMower requires ROS Noetic. ([installation instruction](http://wiki.ros.org/noetic/Installation)) There is no distributed release package yet, for development and test purpose it's best to build the workspace on your own.

For a Raspberry Pi running Ubuntu 20.04/Focal, this fork includes a helper script
which installs ROS Noetic, initializes `rosdep`, fetches submodules, installs
workspace dependencies, creates `mower_config.sh` when missing, and builds the
catkin workspace:

```bash
./utils/scripts/setup_raspberry_ubuntu.sh
```

For a lighter dependency-only pass, use:

```bash
./utils/scripts/setup_raspberry_ubuntu.sh --skip-build
```

The script also creates a local `.env` file when it does not exist. This file is
ignored by git and must be edited on the Raspberry with the Wi-Fi credentials
used to reach the internet/NTRIP caster:

```bash
MOWER_WIFI_SSID="your-wifi-ssid"
MOWER_WIFI_PASSWORD="your-wifi-password"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
```

It also installs `openmower-wifi.service`, which reads `.env` on boot and uses
NetworkManager to connect automatically. If the SSID/password are still empty or
left as `CHANGE_ME_*`, the service logs a reminder and exits without changing
Wi-Fi state:

```bash
journalctl -u openmower-wifi.service -b
```

### iOS app bridge

This fork starts a small ROS HTTP bridge for the current `LawnMowerControl` iOS
app when `open_mower.launch` runs. It exposes:

```text
GET  /api/health
GET  /api/status
GET  /api/telemetry
GET  /api/settings
POST /api/command
POST /api/manual
POST /api/settings
```

By default the bridge listens on port `8080` and sends UDP discovery beacons that
the app can pick up. If discovery is not available, set the app host manually:

```text
http://<raspberry-ip>:8080
```

The setup script writes these optional values to `.env`:

```bash
OM_IOS_BRIDGE_ENABLE="True"
OM_IOS_BRIDGE_HOST="0.0.0.0"
OM_IOS_BRIDGE_PORT="8080"
OM_IOS_BRIDGE_NAME="lawnmower"
OM_IOS_BRIDGE_TOKEN=""
OM_IOS_UDP_BEACON_ENABLE="True"
```

When launching manually, export `.env` values before `roslaunch`:

```bash
set -a
source .env
set +a
```

If `OM_IOS_BRIDGE_TOKEN` is filled, the app must use the same token in its
settings. The bridge translates app commands to OpenMower ROS actions/services;
sensor fields that do not exist in OpenMower yet, such as Mega sonar/bumper
telemetry, are returned as safe defaults until the Arduino/Raspberry bridge
publishes them.

#### Fetch Dependencies

Before building, you need to fetch this project's dependencies. The best way to do this is by using rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Run in the repository's root:

```bash

rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
```

If `rosdep install` reports unresolved keys (typical: `nmea_msgs`, `imu_tools`,
`twist_mux_msgs`, `paho-mqtt-cpp`, `pcl_conversions`, `serial`,
`rtcm_msgs`, `rosbridge_server`), install them directly via apt:

```bash
sudo apt install -y \
  ros-noetic-actionlib \
  ros-noetic-costmap-2d \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-grid-map \
  ros-noetic-grid-map-msgs \
  ros-noetic-imu-tools \
  ros-noetic-mbf-msgs \
  ros-noetic-move-base-flex \
  ros-noetic-nav-core \
  ros-noetic-nmea-msgs \
  ros-noetic-paho-mqtt-c \
  ros-noetic-paho-mqtt-cpp \
  ros-noetic-pcl-conversions \
  ros-noetic-pluginlib \
  ros-noetic-robot-localization \
  ros-noetic-rosbridge-server \
  ros-noetic-rtcm-msgs \
  ros-noetic-serial \
  ros-noetic-tf \
  ros-noetic-tf2-eigen \
  ros-noetic-twist-mux-msgs \
  libgeographic-dev \
  libopencv-dev \
  libwebsocketpp-dev \
  python3-opencv
```

#### Build workspace

To compile all ROS packages in this catkin workspace, run this from the
repository root:

```bash
cd ~/open_mower_ros
source /opt/ros/noetic/setup.bash
catkin_make
```

Once it's done, source the workspace env vars:

```bash
cd ~/open_mower_ros
source devel/setup.bash
```

#### Launch OpenMower

OpenMower ROS package is distributed with [roslaunch](http://wiki.ros.org/roslaunch) launch files.
There are few in: `src/open_mower/open_mower/launch`, however the `open_mower.launch` runs everything needed to mow.

```bash
roslaunch open_mower open_mower.launch
```

Before you launch `open_mower` package, env vars with configuration have to be set.

```bash
cp src/open_mower/config/mower_config.sh.example mower_config.sh
source mower_config.sh # it's expected to adjust the file
```

#### Inspect the running system

**Nodes and topics overview:**

```bash
# All active nodes
rosnode list

# All active topics
rostopic list

# Graph of nodes + topics (requires graphviz)
rosrun rqt_graph rqt_graph
```

**Monitor a topic:**

```bash
# Print messages on a topic (replace <topic> with the actual name)
rostopic echo <topic>

# Show publish rate and bandwidth
rostopic hz <topic>
rostopic bw <topic>

# Show message type and field definitions
rostopic type <topic>
rosmsg show $(rostopic type <topic>)
```

**Useful topics to watch:**

```bash
# Mower state machine
rostopic echo /mower_logic/current_state

# GPS position
rostopic echo /xbot_positioning/xb_pose

# Battery / power
rostopic echo /ll/power

# Emergency stop
rostopic echo /ll/emergency

# Velocity commands
rostopic echo /joy_vel

# iOS bridge status (when running)
rostopic echo /mower_logic/current_state
```

**Nodes detail:**

```bash
# Info on a specific node (subscriptions, publications, services)
rosnode info <node_name>

# All active services
rosservice list

# Call a service (example: pause mowing)
rosservice call /mower_logic/mowing/pause
```

### Running in a container

TBD (no automated image build yet)

## Contribution

### How to Build Using CLion IDE

First, launch CLion in a sourced environment. For this I use the following bash file:

```bash
#!/bin/zsh

source <your_absolute_path_to_repository>/devel/setup.zsh

# You can find this path in the Jetbrains Toolbox
nohup <your_absolute_path_to_clion>/clion.sh >/dev/null 2>&1 &
```


Then, open the `src` directory. CLion will prompt with the following screen:

![CLion CMake Settings](./img/clion_cmake_settings.png)

Copy the settings for **Build directory** and **CMake options**. Everything else can stay the same. This is all you need!


# Notes / ToDos

- For local navigation, I have tried to use the teb_local_planner. Unfortunately, it seems that (at least for me) the noetic version is VERY broken. Therefore I added the current melodic dev version as git submodule to this repo. It seems to work fine with ROS noetic and this setup here.
- If the map has no docking point set, planning crashes as soon as we try to approach the docking point. TODO: check, before even starting to mow.

# License

This work is licensed under the [GNU General Public License version 3](https://www.gnu.org/licenses/gpl-3.0.html). See the [LICENSE](LICENSE) file for details.
