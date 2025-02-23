# AUV Comms

Package responsible for recpetion of input states through optics. Signals are encoded in different colors of light.
Color detection is performed using OpenCV InRange function.

## States

The idx number of the flare colors are encoded as follows:

- `0` -> Red
- `1` -> Yellow
- `2` -> Blue

The color mapping from LED to flare color is as follows:

*LED Color -> Flare Color*

- Red -> Red
- Green -> Yellow
- Blue -> Blue

## Usage

Before running any of this nodes in this package, publish an image to the /image_raw/compressed topic.

To launch the debug detector to identify the correct HSV ranges
```bash
ros2 run auv_comms color_detector_debug --ros-args -r /input_image/compressed:=/image_raw/compressed
```
To launch a node which constantly published the detected color as defined in the params file
```bash
ros2 launch auv_bringup comms_launch.py debug:=True
```
To launch both the read sequence action node and color detector service
```bash
ros2 launch auv_bringup comms_launch.py debug:=True
```

To read a communication sequence
```bash
ros2 action send_goal /read_comm_sequence auv_interfaces/action/ReadCommSequence {} --feedback
```
Param file for the launch file is `auv_bringup/params/color_detector_params.yaml`
