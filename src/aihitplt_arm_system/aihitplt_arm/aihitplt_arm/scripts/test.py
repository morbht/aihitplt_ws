#!/usr/bin/env python3 
# coding = utf-8

from aihitArm_api import aihitArm
import time
import rospy
arm = aihitArm()




# arm.set_angles([20,20,20],50)

# angles = arm.get_angles_info()
# print(f"{angles}")

# arm.go_zero()
# arm.release_all_servers()
# arm.power_on

# arm.is_moving_end()
# moving_flag = arm.is_moving_end() 

# print(f"return {moving_flag}")


# coords = arm.get_coords_info()
# print(f"return {coords}")

# arm.set_gripper_zero()
# rospy.sleep(2)

# arm.set_gripper_state(100, 100)
arm.gripper_release()
# arm.set_coords()

