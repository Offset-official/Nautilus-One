# Nautilus One AUV

> Status: created working gazebol models of the AUV and competition accurate pool world.

> Rulebook Edition :book:: [Rulebook 5.1.4](http://web.archive.org/web/20241231081446/https://sauvc.org/rulebook/)

Contains collection of [ROS2](https://www.ros.org/) packages that provide AUV functionality targeted 
towards the SAUVC 2025 competition for **Nautilus One**.

Nautilus One uses [ArduSub](http://www.ardusub.com/) as the flight controller and
[mavros](https://github.com/mavlink/mavros) as the GCS.

![Nautlius One Gazebo](images/nautilus_one.png)

Nautilus One runs in [Gazebo Garden](https://gazebosim.org/docs/garden/getstarted/) using the standard buoyancy, 
hydrodynamics and thruster
plugins. The connection between ArduSub and Gazebo is provided by [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo).

## Requirements

Please ensure that the following requirements have been met prior to installing the project:

* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
  Install the base version (ros-humble-ros-base)
* [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
  Use the following commands:
  ```
  sudo apt-get update
  sudo apt-get install curl lsb-release gnupg
  ```
  ```
  sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update
  sudo apt-get install gz-garden
  ```
  ```
  apt-get install ros-humble-ros-gzgarden
  ```
  
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
  ```
  sudo apt update
  sudo apt install libgz-sim7-dev rapidjson-dev
  sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
  ```
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)
  ```
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  ```
  Modify the below command according to your ardupilot directory location.
  ```
  export ARDUPILOT_HOME=~/ardupilot/
  ```
* [Rosdep](https://docs.ros.org/en/independent/api/rosdep/html)
  ```
  apt-get install python3-rosdep
  ```
  
* [vcstool](https://github.com/dirk-thomas/vcstool)
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl # if you haven't already installed curl
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install python3-vcstool
  ```

Set `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable to path of your `ardupilot_gazebo` plugin build folder.
Set `GZ_VERSION=garden` environment variable to ensure correct dependencies are installed.

The project assumes that you are using `zsh` like a sane human.

## Installation 

Build ArduSub for SITL:

```bash
cd $ARDUPILOT_HOME
./waf configure --board sitl
./waf sub
```

Add the results of ArduSub build onto the system path:
```bash
echo 'export PATH=$ARDUPILOT_HOME/build/sitl/bin:$PATH' >> ~/.zshrc
zsh
```

Configure rosdep (this only needs to be run once):
```bash
sudo rosdep init
rosdep update
```

Preparing the workspace:

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
git clone https://github.com/Offset-official/auv_ros2
vcs import . < auv_ros2/third_parties.repos
```

Installing package dependencies:

```bash
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -r -y
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Building the workspace:

```bash
cd ~/auv_ws
colcon build --symlink-install
source ~/auv_ws/install/setup.zsh
```

## Usage

### Manual Control
To launch the simulation environment:

```bash
ros2 launch auv_bringup sim_launch.py world:=pool mavros:=False
```

Launch MAVProxy in a 2nd Terminal:
```bash
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out udp:0.0.0.0:14550 --console
```
You can use MAVProxy to send commands directly to ArduSub:

```
arm throttle
rc 3 1450
rc 3 1500
mode alt_hold
disarm
```

RC channels:
* RC 3 -- vertical
* RC 4 -- yaw
* RC 5 -- forward

### Autonomously move the AUV for 10s
To launch the simulation environment:

```bash
ros2 launch auv_bringup sim_launch.py world:=pool
```

Launch the forward executable in a 2nd Terminal:
```bash
ros2 run auv_autonomy forward
```

## Packages

* [`auv_autonomy` Basic autonomous functions](auv_autonomy)
* [`auv_bringup` Launch files](auv_bringup)
* [`auv_description` SDF files](auv_description)
