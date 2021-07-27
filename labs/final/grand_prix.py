"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
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


#color ranges
ORANGE = ((10, 100, 100), (20, 255, 255), "orange")
BLUE = ((90, 100, 100), (120, 255, 255), "blue")
RED = ((170, 100, 100), (10, 255, 255), "red")
GREEN = ((40,100,100),(80,255,255), "green")
PURPLE = ((132, 50, 50), (142, 255, 255), "purple")
MaxSpeed = 1
MIN_CONTOUR_AREA = 30

class State(IntEnum):
    green_line_break = -2
    ar_tag = -1
    green_line = 0
    wall_following = 1
    purple_line_following = 2
    ar_tag_slaloming = 3
    # cone_slaloming = 4
    # elevator = 5
    # trains = 6
    # flat_obstacle = 7
    # jump = 8

curr_state = State.green_line  
height = rc.camera.get_height()
width = rc.camera.get_width()
########################################################################################
# Functions
########################################################################################



def start():
    """
    This function is run once every time the start button is pressed
    """
    global MaxSpeed
    global curr_state

    MaxSpeed = 2
    curr_state = State.green_line
    # Have the car begin at a stop
    rc.drive.stop()

    # raise max speed
    rc.drive.set_max_speed(MaxSpeed)

    # Print start message
    print(">> Final Challenge - Grand Prix")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global MaxSpeed
    global curr_state
    #color camera
    color_img = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_img)
    marker = None if (markers is None or len(markers) == 0) else markers[0]
    #depth camera
    depth_img = rc.camera.get_depth_image()
    
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    
    #lidar
    lidar = rc.lidar.get_samples()
    right_front = rc_utils.get_lidar_average_distance(lidar, 45, 45)
    left_front = rc_utils.get_lidar_average_distance(lidar, 675, 45)
    #the right, left and front distance
    _, right_distance = rc_utils.get_lidar_closest_point(lidar, (80,100))
    _, left_distance = rc_utils.get_lidar_closest_point(lidar, (260,280))
    _, forward_distance = rc_utils.get_lidar_closest_point(lidar, (-5,5))

    if len(markers):
        #print(marker.get_id())
        if marker.get_id() == 0:
            curr_state = State.wall_following
        if marker.get_id() == 1:
            curr_state = State.purple_line_following


    if curr_state == State.green_line:
        print("State: Green line!")
        speed = MaxSpeed
        if color_img is not None:
            contours_green = rc_utils.find_contours(color_img, GREEN[0], GREEN[1])
            contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)
            print(contour_green)
            if contour_green is not None:
                contour_center_green = rc_utils.get_contour_center(contour_green)
                rc_utils.draw_contour(color_img, contour_green, rc_utils.ColorBGR.red.value)
                rc_utils.draw_circle(color_img, contour_center_green,rc_utils.ColorBGR.red.value)
                rc.display.show_color_image(color_img)
                angle = rc_utils.remap_range(contour_center_green[1], 0, rc.camera.get_width(), -1, 1)                
            else: 
                angle = 0

    if curr_state == State.wall_following:
        # code
        diff = right_front - left_front
        mult = 0.1
        if diff > 10:
            mult = 2
        if diff > 40:
            speed = 0
        angle = rc_utils.remap_range(diff, -20, 20, -1, 1) * mult

    if curr_state == State.purple_line_following:
        # img_left = rc_utils.crop(color_img, (height//2, 0), (height, width//4))
        # #print(img_left)
        # img_right = rc_utils.crop(color_img, (height//2, 3*width//4), (height, width))
        # # speed = MaxSpeed
        # # potential_colors = [PURPLE, ORANGE]
        # # detected_color = None
        # # greatest_area = 0        
        # # for (hsv_lower, hsv_upper, color_name) in potential_colors:
        # #     contours = rc_utils.find_contours(color_img, hsv_lower, hsv_upper)
        # #     largest_contour = rc_utils.get_largest_contour(contours)
        # #     if largest_contour is not None:
        # #         contour_area = rc_utils.get_contour_area(largest_contour)
        # #         if contour_area > greatest_area:
        # #             greatest_area = contour_area
        # #             detected_color = color_name

        # #if detected_color == "orange":
        #     #contours_left_orange = rc_utils.find_contours(img_left, ORANGE[0], ORANGE[1])
        #     #contours_right = rc_utils.find_contours(img_right, ORANGE[0], ORANGE[1])
        # #elif detected_color == "purple":
        #     #contours_left = rc_utils.find_contours(img_left, PURPLE[0], PURPLE[1])
        #     #contours_right = rc_utils.find_contours(img_right, PURPLE[0], PURPLE[1])



        # contours_left_PURPLE = rc_utils.find_contours(img_left, PURPLE[0], PURPLE[1])
        # contours_right_PURPLE = rc_utils.find_contours(img_right, PURPLE[0], PURPLE[1])
        # contour_left_PURPLE = rc_utils.get_largest_contour(contours_left_PURPLE, MIN_CONTOUR_AREA)
        # contour_right_PURPLE = rc_utils.get_largest_contour(contours_right_PURPLE, MIN_CONTOUR_AREA)
        # contour_center_left_PURPLE = rc_utils.get_contour_center(contour_left_PURPLE)
        # contour_center_right_PURPLE = rc_utils.get_contour_center(contour_right_PURPLE)

        # contours_left_ORANGE = rc_utils.find_contours(img_left, ORANGE[0], ORANGE[1])
        # contours_right_ORANGE = rc_utils.find_contours(img_right, ORANGE[0], ORANGE[1])
        # contour_left_ORANGE = rc_utils.get_largest_contour(contours_left_ORANGE, MIN_CONTOUR_AREA)        
        # contour_right_ORANGE = rc_utils.get_largest_contour(contours_right_ORANGE, MIN_CONTOUR_AREA)        
        # contour_center_left_ORANGE = rc_utils.get_contour_center(contour_left_ORANGE)        
        # contour_center_right_ORANGE = rc_utils.get_contour_center(contour_right_ORANGE)
        
        
        # kp = 0.02
        # kS = 0.02
        # if contour_center_left_ORANGE is not None and contour_center_right_ORANGE is not None:
        #     midpoint = contour_center_right_ORANGE[1] - contour_center_left_ORANGE[1]
        #     angle = kp*midpoint
        #     angle = rc_utils.clamp(angle, -1, 1)
        #     print(f"contour right: {contour_center_right_ORANGE[1]} countour left: {contour_center_left_ORANGE[1]}")
        # if contour_center_right_ORANGE is not None and contour_center_left_ORANGE is None:
        #     #angle = -0.25
        #     midpoint = contour_center_right_ORANGE[1] - 35
        #     angle = kp*midpoint
        #     angle = rc_utils.clamp(angle, -1, 1)
        # if contour_center_left_ORANGE is not None and contour_center_right_ORANGE is None:
        #     #angle = 0.25
        #     midpoint = contour_center_left_ORANGE[1] + 35
        #     angle = kp*midpoint
        #     angle = rc_utils.clamp(angle, -1, 1)

        # if contour_center_left_PURPLE is not None and contour_center_right_PURPLE is not None:
        #     midpoint = contour_center_right_PURPLE[1] - contour_center_left_PURPLE[1]
        #     angle = kp*midpoint
        #     angle = rc_utils.clamp(angle, -1, 1)
        #     print(f"contour right: {contour_center_right_PURPLE[1]} countour left: {contour_center_left_PURPLE[1]}")
        # if contour_center_right_PURPLE is not None and contour_center_left_PURPLE is None:
        #     #angle = -0.25
        #     midpoint = contour_center_right_PURPLE[1] - 35
        #     angle = kp*midpoint
        #     angle = rc_utils.clamp(angle, -1, 1)
        # if contour_center_left_PURPLE is not None and contour_center_right_PURPLE is None:
        #     #angle = 0.25
        #     midpoint = contour_center_left_PURPLE[1] + 35
        #     angle = kp*midpoint
        #     angle = rc_utils.clamp(angle, -1, 1)
        # speed = 0.1
    


        img_left = rc_utils.crop(color_img, (4* height//5, 0), (height, width//3))
        img_right = rc_utils.crop(color_img, (4* height//5, 2 * width//3), (height, width))

        potential_colors = [PURPLE, ORANGE]
        detected_color = None
        greatest_area = 0        
        for (hsv_lower, hsv_upper, color_name) in potential_colors:
            contours = rc_utils.find_contours(color_img, hsv_lower, hsv_upper)
            largest_contour = rc_utils.get_largest_contour(contours)
            if largest_contour is not None:
                contour_area = rc_utils.get_contour_area(largest_contour)
                if contour_area > greatest_area:
                    greatest_area = contour_area
                    detected_color = color_name

        if detected_color == "orange":
            contours_left_ORANGE = rc_utils.find_contours(img_left, ORANGE[0], ORANGE[1])
            contours_right_ORANGE = rc_utils.find_contours(img_right, ORANGE[0], ORANGE[1])
        elif detected_color == "purple":
            contours_left_PURPLE = rc_utils.find_contours(img_left, PURPLE[0], PURPLE[1])
            contours_right_PURPLE = rc_utils.find_contours(img_right, PURPLE[0], PURPLE[1])


        contours_left_PURPLE = rc_utils.find_contours(img_left, PURPLE[0], PURPLE[1])
        contours_right_PURPLE = rc_utils.find_contours(img_right, PURPLE[0], PURPLE[1])
        contour_left_PURPLE = rc_utils.get_largest_contour(contours_left_PURPLE, MIN_CONTOUR_AREA)
        contour_right_PURPLE = rc_utils.get_largest_contour(contours_right_PURPLE, MIN_CONTOUR_AREA)
        contour_center_left_PURPLE = rc_utils.get_contour_center(contour_left_PURPLE)
        contour_center_right_PURPLE = rc_utils.get_contour_center(contour_right_PURPLE)

        contours_left_ORANGE = rc_utils.find_contours(img_left, ORANGE[0], ORANGE[1])
        contours_right_ORANGE = rc_utils.find_contours(img_right, ORANGE[0], ORANGE[1])
        contour_left_ORANGE = rc_utils.get_largest_contour(contours_left_ORANGE, MIN_CONTOUR_AREA)        
        contour_right_ORANGE = rc_utils.get_largest_contour(contours_right_ORANGE, MIN_CONTOUR_AREA)        
        contour_center_left_ORANGE = rc_utils.get_contour_center(contour_left_ORANGE)        
        contour_center_right_ORANGE = rc_utils.get_contour_center(contour_right_ORANGE)

        kp = 0.02
        if contour_center_right_ORANGE is not None:
            midpoint = contour_center_right_ORANGE[1] - 200
            angle = kp * midpoint
            print("right found")
        if contour_center_right_PURPLE is not None:
            midpoint = contour_center_right_PURPLE[1] - 200
            angle = kp * midpoint
            print("right found")
        if contour_center_right_PURPLE is None and contour_center_right_ORANGE is None:
            # if contour_center_left_ORANGE is not None:
            #     midpoint = 200 - contour_center_left_ORANGE[1]
            #     angle = kp * midpoint
            #     print("left found")
            angle = 0.7
            # if contour_center_left_PURPLE is not None:
            #     midpoint = 200 - contour_center_left_PURPLE[1]
            #     angle = kp * midpoint
            #     print("left found")
        
    speed = 0.1
    #vis = np.concatenate((img_left, img_right), axis=1)
    #rc.display.show_color_image(vis)
    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_speed_angle(speed, angle) 
    


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
