"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    search = 0
    approach = 1
    reverse = 2
    stop = 3

curr_state: State = State.search

def start():
    """
    This function is run once every time the start button is pressed
    """
    global aligned
    global speed
    aligned = False
    speed = 0
    curr_state = State.search
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Park the car 20 cm away from the closest wall with the car directly facing
    # the wall
    global aligned
    global speed
    global curr_state

    depth_image = rc.camera.get_depth_image()

    depth_image = cv.GaussianBlur(depth_image, (5,5), 0)
    
    left_point = depth_image[rc.camera.get_height() // 3, rc.camera.get_width() //4]
    center_point = depth_image[rc.camera.get_height() // 3, rc.camera.get_width() // 2]
    right_point = depth_image[rc.camera.get_height() // 3, 3 * rc.camera.get_width() // 4]

    print(f"L: {left_point}     C: {center_point}     R: {right_point}")
    
    if curr_state == State.search:
        speed = -0.5
        angle = 1
        if left_point != 0 or right_point != 0:
            curr_state = State.approach
    if curr_state == State.approach:
        if not aligned:
            if left_point == 0.0 or (left_point - right_point > 1 and right_point != 0):#(left_point - center_point >= 1 and center_point - right_point >= 1): #this means we are angled to the left
                angle = -1
                speed = -0.5
            elif right_point == 0.0 or  (right_point - left_point > 1 and left_point != 0): #(center_point - left_point >= 1 and right_point - center_point >= 1): #this means we are angled to the right
                angle = 1
                speed = -0.5
            else:
                aligned = True 
    
        if aligned:
            angle = 0.0
            distanceToWall = rc_utils.get_depth_image_center_distance(depth_image)
            speed = rc_utils.remap_range(distanceToWall, 0, 40, -0.3, 0.3)
            speed = rc_utils.clamp(speed, -0.3, 0.3)

            if distanceToWall > 19 and distanceToWall < 21:
                speed = 0.0

    print(speed)
    rc.drive.set_speed_angle(speed, angle)



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
