#!/bin/sh

source install/local_setup.bash
ros2 run minimal_timer_pub minimal_timer_pub_node
ros2 run led_serial led_serial_node 
ros2 launch gesture_recognizer gesture_recognizer.launch.py

