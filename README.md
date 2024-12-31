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

Please ensure that the following requirements have been met prior to installing the project

* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)
* [Rosdep](https://docs.ros.org/en/independent/api/rosdep/html)
* [vcstool](https://github.com/dirk-thomas/vcstool)

Set `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable to path of your `ardupilot_gazebo` plugin build folder.
Set `GZ_VERSION=garden` environment variable to ensure correct dependencies are installed.

The project assumes that you are using `zsh` like a sane human.

## Installation 

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
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
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

## Packages

* [`auv_autonomy` Basic autonomous functions](auv_autonomy)
* [`auv_bringup` Launch files](auv_bringup)
* [`auv_description` SDF files](auv_description)
