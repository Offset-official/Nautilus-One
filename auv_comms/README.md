# AUV Comms

Package responsible for recpetion of input states through optics. Signals are encoded in different colors of light.
Color detection is performed using OpenCV InRange function.

## Usage

Before running any of this nodes in this package, publish an image to the /image_raw/compressed topic.

To launch the debug detector to identify the correct HSV ranges
```bash
ros2 run auv_comms color_detector_debug --ros-args -r /input_image/compressed:=/image_raw/compressed
```
