# Top Side Computing Manual

This guide instructs the operational commands for the AUV. In the current configuration all
nodes except inference nodes are running on the AUV.

## Prerequisites
- Top side laptop should be configured to have static IP address of 192.168.2.1/24.
- IP routing via Raspberry Pi to Jetson Nano should be configured.
- FastDDS configuration should be done on the laptop.
- The auv_ws workspace with `auv_interfaces` should be build and sourced on the top side laptop.

### IP Routing Configuration

To configure IP routing, run the following commands on the **top side laptop**:
```bash
sudo ip route add 192.168.2.4 via 192.168.2.2
```

### FastDDS Configuration

To configure FastDDS, run the following commands on the **top side laptop**:
```bash
export ROS_DISCOVERY_SERVER=192.168.2.2:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=~/auv_ws/src/auv_ros2/super_client_configuration_file.xml

ros2 daemon stop && ros2 daemon start
```

These commands would have to be run in every terminal session.

### Raspberry Pi (Navigator) Packages

- **auv_bringup** : Launch Files
- **auv_controller** : Controller Node
- **auv_interfaces** : Custom Messages,Srvs,Actions
- **auv_manipulators** : Arm actuation packages
- **auv_mav_utils** : Mavlink nodes

### Jetson Nano (Inference) Packages

- **auv_autonomy** : Autonomous Behaviours
- **auv_bringup** : Launch Files
- **auv_camera** : Custom Camera Nodes
- **auv_comms** : Underwater Communication Nodes
- **auv_diagnostics** : Diagnostic Nodes
- **auv_experiments** : Subham things 
- **auv_interfaces** : Custom Messages,Srvs,Actions

## Raspberry Pi Operation

### SSH
To begin operation, SSH into the Raspberry Pi using the following command:
```bash
ssh pi@192.168.2.2
```
Password: `raspberry`

### Docker container
To start the docker container, run the following command:
```bash
docker run --rm --network host --name pi-ros-main -it ghcr.io/offset-official/pi-ros-full
```

Sometimes an existing container may be running. To stop the container, run the following command:
```bash
docker container rm --force pi-ros-main
```
Then rerun the container.

This container will use the `auv_bringup` package to launch the necessary nodes.
A TMUX session will be available to operate inside the docker container.

The TMUX session will automatically launch the `pi.launch.py` launch file in the first window.

The first window can be accessed through `Ctrl + b` followed by `0`.

More information on the docker container can be found in the `docker/README.md` file.

### Rebuilding packages / Code changes

All code is present in the `/auv_ws/` directory. This is in the root directory. Neovim and Nano
are available to make any code changes. 

To rebuild the packages, run the following command:
```bash
cd /auv_ws
colcon build
```
Source the packages on all the running terminals
```bash
source /auv_ws/install/setup.bash
```

## Jetson Nano Operation

### SSH
To begin operation, SSH into the Jetson Nano using the following command:
```bash
ssh nano@192.168.2.4
```
Password: `nano`

### Docker container
To start the docker container, run the following command:
```bash
sudo docker run --rm --network host --name nano-ros-main -it --device=/dev/ttyACM0 --device=/dev/video0 --device=/dev/video2 ghcr.io/offset-official/nano-ros-full
```

Sometimes an existing container may be running. To stop the container, run the following command:
```bash
sudo docker container rm --force nano-ros-main
```
Then rerun the container.

This container will use the `auv_bringup` package to launch the necessary nodes.
A TMUX session will be available to operate inside the docker container.

The TMUX session will automatically launch the `nano.launch.py` launch file in the first window.

The first window can be accessed through `Ctrl + b` followed by `0`.

More information on the docker container can be found in the `docker/README.md` file.

### Rebuilding packages / Code changes

All code is present in the `/auv_ws/` directory. This is in the root directory. Neovim and Nano
are available to make any code changes.

To rebuild the packages, run the following command:
```bash
cd /auv_ws
colcon build
```

Source the packages on all the running terminals
```bash
source /auv_ws/install/setup.bash
```

## Top Side Laptop Operation

The top side laptop is used to control the AUV. The following commands are used to control the AUV.
All the commands are run on the **top side laptop**.


### Base Controller 

To issue movement commands run the following
```bash
ros2 run auv_controller control_cmd_cli
```

To check the pressure output
```bash
ros2 topic echo /current_depth
```

To issue a raw action to descend to a particular depth eg. -0.3 meters
```bash
ros2 action send_goal /depth_descent auv_interfaces/action/DepthDescent '{target_depth: -0.3}' --feedback 
```

If you need to manaully arm the thrusters,
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```


### AUV Mav Utils

To run heading test with different params.  
```bash
ros2 run auv_mav_utils heading_test --ros-args -p target_depth:=-0.8 -p linear_speed:=1.5 -p enable_angle_correction:=false -p movement_duration:=15.0
```
### Arm Controller

To actuate the arm, run the following command
```bash
ros2 service call /dropper_trigger auv_interfaces/srv/DropperTrigger "{enable: true}"` 
```

### Viewing Camera Feed
To view the camera feed, run the following command
```bash
ros2 run rqt_image_view rqt_image_view
```
And choose the appropriate topic.


### Diagnostics

To change the diagnostic LEDs colour (eg. white), run the following command
```bash
 ros2 service call /set_color auv_interfaces/srv/SetColor "{color: '#ffffff'}"
```
To turn off the LEDs, run the following command
```bash
 ros2 service call /set_color auv_interfaces/srv/SetColor "{color: 'off'}"
```

### Communication

To initiate an entire communication sequence, run the following command
```bash
ros2 action send_goal /read_comm_sequence auv_interfaces/action/ReadCommSequence {} --feedback
```

### Experiments
Run the appropriate experiment node to test the desired functionality.

eg. qualify node

```bash
ros2 run auv_experiments qualify
```
