# deck-ros2

Use a Steam Deck as a controller for ROS2 robots. This is heavily dependant on the [L-CAS](https://github.com/lcas) ROS2 Humble ecosystem, and this repo has been inspired by the configuration of the [limo_platform](https://github.com/lcas/limo_platform).


## Controller support
device_id (int, default: 0)
The joystick device to use. `ros2 run joy joy_enumerate_devices` wil list the attached devices.