"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
"""

########################################################################################
# Imports
########################################################################################

import sys
from typing import List, Tuple

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import Enum

import math
import socket

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here

class State(Enum):
    FOLLOWING_LEFT = 1,
    FOLLOWING_RIGHT = 2,
    REVERSING_LEFT = 3,
    REVERSING_RIGHT = 4,
    FOLLOWING_OTHER_CAR = 5,
    FINISHED_HIGH_PRIORITY = 6,
    FINISHED_LOW_PRIORITY = 7


state : State = State.FOLLOWING_RIGHT

END_OF_PATH_ID : int = None  # TODO: Change this to the correct id number
RED = ((170, 100, 100), (10, 255, 255), "red")
GREEN = ((40, 60, 60), (90, 255, 255), "green")

KPA : float = 0.03
DIS_FROM_WALL : int = 100

cur_path : List[int] = []

found_ar_marker_last_update : bool = False

s : socket = None

########################################################################################
# Functions
########################################################################################


def define_server():
    global s 

    host = socket.gethostname()   # get local machine name
    port = 8080  # Make sure it's within the > 1024 $$ <65535 range
  
    sock = socket.socket()
    sock.bind((host, port))
  
    sock.listen(1)
    s, addr = sock.accept()
    print("Connection from: " + str(addr))


def get_left_angle(scan, forward: bool) -> float:
    left_dis = rc_utils.get_lidar_closest_point(scan, (-50, -40))
    angle = KPA * (DIS_FROM_WALL - left_dis)
    return angle * (1 if forward else -1)


def get_right_angle(scan, forward: bool) -> float:
    right_dis = rc_utils.get_lidar_closest_point(scan, (40, 50))
    angle = KPA * (right_dis - DIS_FROM_WALL)
    return angle * (1 if forward else -1)

def get_left_controller(scan, forward: bool) -> Tuple[float, float]:
    speed = 1 if forward else -1
    return speed, get_left_angle(scan, forward)

def get_right_controller(scan, forward: bool) -> Tuple[float, float]:
    speed = 1 if forward else -1
    return speed, get_right_angle(scan, forward)


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    global state, cur_path

    state = State.FOLLOWING_RIGHT
    cur_path = []

    define_server()


def update():
    global state, cur_path, found_ar_marker_last_update
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    color_image = rc.camera.get_color_image()
    ar_markers = rc_utils.get_ar_markers(color_image)

    scan = rc.lidar.get_samples()
    scan = (scan - 0.01) % 10000

    if len(ar_markers) > 0:
        if not found_ar_marker_last_update:
            found_ar_marker_last_update = True
            ar_marker = ar_markers[0]
            if ar_marker.get_id() == END_OF_PATH_ID:
                ar_marker.detect_colors(color_image, (RED, GREEN))
                if ar_marker.get_color() == 'red':
                    ''' Uncompleted todo '''
                    # TODO: Send info to the other racecar that it found the high priority situation
                    state = State.FINISHED_HIGH_PRIORITY
                elif ar_marker.get_color() == 'green':
                    ''' Uncompleted todo '''
                    # TODO: Send info to the other racecar that it found the low priority situation
                    state = State.FINISHED_LOW_PRIORITY
                else:
                    # TODO: Reverse while following the correct wall to back out
                    if state == State.FOLLOWING_LEFT:
                        state = State.REVERSING_LEFT
                    else:
                        state = State.REVERSING_RIGHT
            else:
                # TODO: This is an intersection
                if state == State.REVERSING_LEFT:
                    cur_path.pop()
                    if cur_path[-1] == 1:
                        state = State.REVERSING_RIGHT
                    else:
                        state = State.REVERSING_LEFT
                elif state == State.REVERSING_RIGHT:
                    cur_path[-1] = 0
                    state = State.FOLLOWING_LEFT
                else:
                    cur_path.append(1)
                    state = State.FOLLOWING_RIGHT
    else:
        found_ar_marker_last_update = False
    
    ''' Uncompleted todo '''
    # TODO: Check if the other car found the high priority position (go right away)
    # If the other car only found the low priority position, keep searching
    
    speed : float = 0.0
    angle : float = 0.0

    if state == State.FOLLOWING_LEFT:
        # TODO: Follow the left wall while going forwards
        speed, angle = get_left_controller(scan, True)
    elif state == State.FOLLOWING_RIGHT:
        # TODO: Follow the right wall while going forwards
        speed, angle = get_right_controller(scan, True)
    elif state == State.REVERSING_LEFT:
        # TODO: Follow the left wall while going backwards
        speed, angle = get_left_controller(scan, False)
    elif state == State.REVERSING_RIGHT:
        # TODO: Follow the right wall while going backwards
        speed, angle = get_right_controller(scan, False)
    elif state == State.FOLLOWING_OTHER_CAR:
        ''' Uncompleted todo '''
        # TODO: Follow the other car
        # Maybe use a function for checking if it made it back far enough 
        pass
    else:
        # TODO: Done (for now in low priority, completely finished in high priority)
        pass
    
    rc.drive.set_speed_angle(speed, angle)



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
