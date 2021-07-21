"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

class State(IntEnum):
    FOLLOW_LEFT = 0,
    FOLLOW_RIGHT = 1,
    FOLLOW_LINE = 2

state = State.FOLLOW_LEFT

BLUE = ((90, 100, 100), (120, 255, 255), "blue")
RED = ((170, 100, 100), (10, 255, 255), "red")

BLUE_LINE = ((90, 100, 100), (120, 255, 255))  # The HSV range for the color blue
RED_LINE = ((175, 100, 100), (10, 255, 255))
GREEN_LINE = ((40, 60, 60), (90, 255, 255))
PRIORITY_COLORS = (BLUE_LINE, GREEN_LINE, RED_LINE)

CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

MIN_CONTOUR_AREA = 30

########################################################################################
# Functions
########################################################################################

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)

        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        for COLOR_RANGE in PRIORITY_COLORS:
            # Find all of the blue contours
            contours = rc_utils.find_contours(image, COLOR_RANGE[0], COLOR_RANGE[1])

            # Select the largest contour
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)

                break

            else:
                contour_center = None

        # Display the image to the screen
        rc.display.show_color_image(image)

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    global state
    state = State.FOLLOW_LEFT

    # Print start message
    print(">> Lab 5 - AR Markers")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global state, PRIORITY_COLORS

    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)

    marker = None if (markers is None or len(markers) == 0) else markers[0]

    scan = rc.lidar.get_samples()

    if marker is not None:
        # TODO: Turn left if we see a marker with ID 0 and right for ID 1
        if marker.get_id() == 0:
            state = State.FOLLOW_LEFT
        elif marker.get_id() == 1:
            state = State.FOLLOW_RIGHT

        # TODO: If we see a marker with ID 199, turn left if the marker faces left and right
        # if the marker faces right
        elif marker.get_id() == 199:
            if marker.get_orientation() == rc_utils.Orientation.LEFT:
                state = State.FOLLOW_LEFT
            else:
                state = State.FOLLOW_RIGHT

        # TODO: If we see a marker with ID 2, follow the color line which matches the color
        # border surrounding the marker (either blue or red). If neither color is found but
        # we see a green line, follow that instead.
        elif marker.get_id() == 2:
            marker.detect_colors(color_image, [RED, BLUE])
            if marker.get_color() == 'red':
                PRIORITY_COLORS = (RED_LINE, GREEN_LINE)
                state = State.FOLLOW_LINE
            else:
                PRIORITY_COLORS = (BLUE_LINE, GREEN_LINE)
                state = State.FOLLOW_LINE

    kpa = 0.025
    if state == State.FOLLOW_RIGHT:
        _, right_point = rc_utils.get_lidar_closest_point(scan, (44,  46))
        midpoint = right_point - 65
        angle = kpa * midpoint
    elif state == State.FOLLOW_LEFT:
        _, left_point = rc_utils.get_lidar_closest_point(scan, (-46,  -44))
        midpoint = 65 - left_point
        angle = kpa * midpoint
    elif state == State.FOLLOW_LINE:
        update_contour()
        if contour_center is not None:
            kpa = 0.02
            angle = kpa * (contour_center[1] - rc.camera.get_width()/2)
        else:
            _, left_point = rc_utils.get_lidar_closest_point(scan, (-46,  -44))
            midpoint = 65 - left_point
            angle = kpa * midpoint
    
    angle = rc_utils.clamp(angle, -1, 1)
    speed = 1

    rc.drive.set_speed_angle(speed, angle)
        
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
