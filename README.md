# Offset AUV

Contains collection of ROS2 packages for the SAUVC competition AUV.

## Installation 

Install these packages:
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)
* [Rosdep](https://docs.ros.org/en/independent/api/rosdep/html)
* [vcstool](https://github.com/dirk-thomas/vcstool)

Set `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable to point to your gazebo ardupilot build folder.

Configure rosdep (this only needs to be run once)

```bash
sudo rosdep init
rosdep update
```

Preparing the workspace

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
git clone https://github.com/Offset-official/auv_ros2
vcs import . < auv_ros2/third_parties.repos
```

Installing package dependencies

```bash
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -r -y
```

Building the workspace

```bash
cd ~/auv_ws
colcon build --symlink-install
source ~/auv_ws/install/setup.zsh
```

## Usage

To launch the simulation environment:

```bash
ros2 launch auv_bringup sim_launch.py world:=pool
```

Then launch MAVProxy from the `$ARDUPILOT_HOME` folder.
```bash
Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub -f vectored --model=JSON --out=udp:0.0.0.0:14550 --console
```
