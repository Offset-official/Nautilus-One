## Installation Steps

Please use `zsh` like a sane human.

Please ensure that the following requirements have been met prior to installing the project:

* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
  Install the base version (ros-humble-ros-base).
  
  Install colcon as well. (Do the following)
  ```bash
  pip install -U colcon-common-extensions
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
  zsh
  ```
* [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
  Use the following commands:
  ```bash
  sudo apt-get update
  sudo apt-get install curl lsb-release gnupg
  ```
  ```bash
  sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update
  sudo apt-get install gz-garden
  ```
  ```bash
  apt-get install ros-humble-ros-gzgarden
  ```
  ```bash
  echo 'export GZ_VERSION=garden' >> ~/.zshrc
  zsh
  ```
  
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
  ```bash
  sudo apt update
  sudo apt install libgz-sim7-dev rapidjson-dev
  sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
  ```
  ```bash
  git clone https://github.com/ArduPilot/ardupilot_gazebo
  cd ardupilot_gazebo
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j4
  ```
  Make sure the following location for the directory (`$HOME/ardupilot_gazebo`) is correct. Otherwise, modify it.
  ```bash
  echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.zshrc
  echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.zshrc
  ```
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)
  ```bash
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  ```
  Modify the below command according to your ardupilot directory location.
  ```bash
  echo 'export ARDUPILOT_HOME=~/ardupilot/' >> ~/.zshrc
  zsh
  ```
* [Rosdep](https://docs.ros.org/en/independent/api/rosdep/html)
  ```bash
  apt-get install python3-rosdep
  ```
  
* [vcstool](https://github.com/dirk-thomas/vcstool)
  ```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl # if you haven't already installed curl
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install python3-vcstool
  ```

## Installation 

Build ArduSub for SITL: (only once)

```bash
cd $ARDUPILOT_HOME
./waf configure --board sitl
./waf sub
```
If you get an error here, do 
```
pip3 install future
```

Add the results of ArduSub build onto the system path: (only once)
```bash
echo 'export PATH=$ARDUPILOT_HOME/build/sitl/bin:$PATH' >> ~/.zshrc
zsh
```

Configure rosdep (only once):
```bash
sudo rosdep init
rosdep update
```

Preparing the workspace: (only once)

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
git clone https://github.com/Offset-official/auv_ros2
vcs import . < auv_ros2/third_parties.repos
```

Installing package dependencies: (only once)

```bash
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -r -y
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Building the workspace: (anytime we write packages)

```bash
cd ~/auv_ws
colcon build --symlink-install
source ~/auv_ws/install/setup.zsh
```
