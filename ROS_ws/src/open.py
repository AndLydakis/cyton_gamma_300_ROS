#!/usr/bin/env python
import os
os.system("rostopic pub -1 /gripper_position_controller/command std_msgs/Float64 -- -4")

