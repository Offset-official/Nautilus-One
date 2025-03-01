Start the service
`ros2 launch auv_bringup dropper.launch.py`


Call the service
`ros2 service call /dropper_trigger auv_interfaces/srv/DropperTrigger "{enable: true}"`

