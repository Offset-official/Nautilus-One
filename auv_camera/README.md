## Recording Cameras

Run the following command to launch the node responsile for recording the camera topics:

```bash
ros2 run auv_camera camera_recorder auv_camera_down auv_camera_front
```

Use the following services to start and stop recording:

```bash
ros2 service call /start_recording std_srvs/srv/Trigger {}

ros2 service call /stop_recording std_srvs/srv/Trigger {}
```

