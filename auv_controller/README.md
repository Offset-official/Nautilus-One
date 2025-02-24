To execute depth_descent action:

```bash
ros2 action send_goal /depth_descent auv_interfaces/action/DepthDescent '{target_depth: -0.1}' --feedback
```

To launch the simulation environment:

```bash
ros2 launch auv_bringup sim_launch.py world:=pool
```

To run the base controller (subscribes to cmd_vel and imu to convert twist to rc commands and plot the controller.)
```bash
ros2 run auv_controller base_controller
```

To run the velocity input (values between 1 to -1 only. 1 being 1550 and -1 being 1450)
```bash
ros2 run auv_controller cmd_vel_publisher
```


To run the vel plotter

```bash
cd ~/auv_ws/src/auv_ros2/auv_controller/src/

python3 velocity_plotter.py
```

To change the params
```bash
ros2 param set /base_controller pid_ki 2.0 pid_kd 2.0 pid_kp 2.0
```
