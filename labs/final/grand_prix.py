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
from racecar_utils import Orientation
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
    
# Add any global variables here


#color ranges
ORANGE = ((10, 100, 100), (20, 255, 255), "orange")
BLUE = ((100,150,150), (120, 255, 255), "blue")
RED = ((170, 150, 100), (50, 255, 255), "red")
GREEN = ((40,100,100),(80,255,255), "green")
PURPLE = ((132, 50, 50), (142, 255, 255), "purple")
BLACK = ORANGE
MIN_CONTOUR_AREA = 30

class State(IntEnum):
    green_line_break = -2
    ar_tag = -1
    green_line = 0
    wall_following = 1
    purple_line_following = 2
    ar_tag_slaloming = 3
    cone_slaloming = 4
    elevator = 5
    trains = 6
    flat_obstacle = 7
    jump = 8
    stop = 9
    stop2 = 10

curr_state = State.green_line
height = rc.camera.get_height()
width = rc.camera.get_width()
counter_wall = 0
MaxSpeed = 2
counter = 0
just_finished_wall_following = False

#AR tag slaloming
class State_ARS(IntEnum):
    search = 0
    approach = 1
    avoid_left = 2
    avoid_right = 3

class State_COLOR(IntEnum):
    align_red = 0
    avoid_red = 1
    align_blue = 2
    avoid_blue = 3

curr_state_COLOR = State_COLOR.align_red
counter_COLOR = 0
counter_final = 0
var = False
counter_stop = 0
counter_stop_2 = 0

curr_state_ARS = State_ARS.search
counter_AR = 0
orientation = None
last_direction_left = False
last_direction_right = False

MIN_CONTOUR_AREA_COLOR = 750
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
color = 'none'

green_left = 2
green_var = False
allow = False
allow_2 = False
allow_3 = False
allow_3_exit_ticket = False
var2 = False
counter_train = 0
counter_final_stretch = 0
counter_train1 = 0

green_var_2 = False
green_left_2 = False

camefromtrain = False
counterfromtrain = 0

camefrompurple = False
camefromcone = False

cone_counter = 0
counter_jump = 0
jump = False
finallyfinal = False
########################################################################################
# Functions
########################################################################################

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global color
    
    image = rc.camera.get_color_image() 

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the red/blue contours
        contours_red = rc_utils.find_contours(image, RED[0], RED[1])
        contours_blue = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        
        # Select the largest contour
        contour_red = rc_utils.get_largest_contour(contours_red, MIN_CONTOUR_AREA_COLOR)
        contour_blue = rc_utils.get_largest_contour(contours_blue, MIN_CONTOUR_AREA_COLOR)
        
        if contour_red is not None and contour_blue is not None:
            if rc_utils.get_contour_area(contour_red) > rc_utils.get_contour_area(contour_blue):
                contour = contour_red
                color = 'red'                
            else:
                contour = contour_blue 
                color = 'blue' 
        elif contour_red is not None:
            contour = contour_red
            color = 'red'
        elif contour_blue is not None:
            contour = contour_blue
            color = 'blue'
        else:
            contour = None
            color = 'none'

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)

def start():
    """
    This function is run once every time the start button is pressed
    """
    global MaxSpeed
    #global curr_state

    #MaxSpeed = 2
    # curr_state = State.green_line
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
    global curr_state_ARS
    global counter
    global just_finished_wall_following
    global counter_wall
    global counter_AR
    global orientation
    global last_direction_left
    global last_direction_right
    global curr_state_COLOR
    global color
    global counter_COLOR
    global counter_final
    global var
    global counter_stop
    global counter_stop_2
    global green_left
    global green_var
    global allow
    global allow_2
    global allow_3
    global allow_3_exit_ticket
    global var2
    global counter_train
    global green_var_2
    global green_left_2
    global camefromtrain
    global counterfromtrain
    global camefrompurple
    global camefromcone
    global counter_final_stretch
    global counter_train1
    global cone_counter
    global counter_jump
    global jump
    global finallyfinal

    #color camera
    color_img = rc.camera.get_color_image()
    color_img_2 = color_img
    color_img_2 = rc_utils.crop(color_img_2, (0, 0), (height//2, width))
    markers = rc_utils.get_ar_markers(color_img)
    marker = None if (markers is None or len(markers) == 0) else markers[0]

    #depth camera
    depth_img = rc.camera.get_depth_image()
    ar_marker_near = True if depth_img[(height//4)][width//2] > 0 else False
    ar_marker_elevator = depth_img[(height//4)][width//2]
    depth_image_new = (depth_img - 0.01) % 10000

    depth_image_COLOR = rc.camera.get_depth_image()
    depth_image_COLOR = (depth_image_COLOR - 0.01) % 10000
    
    #Default Driving Controls
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    
    #LiDAR
    lidar = rc.lidar.get_samples()
    right_front = rc_utils.get_lidar_average_distance(lidar, 45, 45)
    left_front = rc_utils.get_lidar_average_distance(lidar, 675, 45)
    _, right_distance = rc_utils.get_lidar_closest_point(lidar, (80,100))
    _, left_distance = rc_utils.get_lidar_closest_point(lidar, (260,280))
    _, forward_distance = rc_utils.get_lidar_closest_point(lidar, (-5,5))

    contours_green = rc_utils.find_contours(color_img, GREEN[0], GREEN[1])
    if contours_green:
        contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)

    #State Machine:
      
    #AR_TAG
    if curr_state == State.ar_tag:
        #print(marker.get_id())
        counter = 0
        just_finished_wall_following = False
        if len(markers) > 0 and ar_marker_near:
            print(marker.get_id())
            if marker.get_id() == 0:
                curr_state = State.wall_following
            if marker.get_id() == 1:
                curr_state = State.purple_line_following
            if marker.get_id() == 3:
                curr_state = State.elevator
            if marker.get_id() == 4:
                marker_4 = True
                #curr_state = State.cone_slaloming
                curr_state = State.stop2
            if marker.get_id() == 5:
                curr_state = State.trains
            # if marker.get_id() == 6:
            #     curr_state = State.flat_obstacle
            if marker.get_id() == 8:
                curr_state = State.jump
        else:
            curr_state = State.green_line
            
    if curr_state == State.jump:
        allow = False
        curr_state = State.green_line
        jump = True
    

    #GREEN LINE
    if curr_state == State.green_line:
        if jump:
            print(counter_jump)
            counter_jump += rc.get_delta_time()
            if 0.1< counter_jump < 1.7 :
                speed = -1
                print("GOING 0.2 ALMOST DONE")
            elif counter_jump > 2:
                finallyfinal = True
                speed = 0.5
                print("FINSIHED")
        if len(markers) > 0 and ar_marker_near:
            curr_state = State.ar_tag
        else:
            MaxSpeed = 0.3
            speed = MaxSpeed  
            if camefromcone:
                speed = 0.1
            if camefromtrain == True:
                speed = 0.11
            if var2 == False: 
                if color_img is not None:
                    contours_green = rc_utils.find_contours(color_img, GREEN[0], GREEN[1])
                    contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)
                    #print(contour_green)
                    if contour_green is not None:
                        contour_center_green = rc_utils.get_contour_center(contour_green)
                        rc_utils.draw_contour(color_img, contour_green, rc_utils.ColorBGR.red.value)
                        rc_utils.draw_circle(color_img, contour_center_green,rc_utils.ColorBGR.red.value)
                        #rc.display.show_color_image(color_img)
                        angle = rc_utils.remap_range(contour_center_green[1], 0, rc.camera.get_width(), -1, 1)               
                    else: 
                        angle = 0
                    if just_finished_wall_following:
                        counter += rc.get_delta_time()
                        if 1 < counter < 1.25:
                            angle = 1
                    if counter_wall == 2:
                        counter += rc.get_delta_time()
                        if 4 < counter < 5:
                            angle = 1
                    if contour_green is None:
                        if camefrompurple == True:
                            angle = -0.5
                        else:
                            angle = 1
                        if camefromtrain == True:
                            angle = -1
                            speed = 0.125
                        if counter_final_stretch != 0:
                            angle = 0.5
                    if contour_green is None and allow_3 == True:
                        print("                             allow 3 trye")
                        angle = -1
                        allow_3_exit_ticket = True
                    elif contour_green is not None and allow_3 == True and allow_3_exit_ticket is True:
                        allow = True
                        allow_3 = False
                        
                        print("Hhgfshfias budhiudhgruivbsiu husba urebger re er gre er gerug er guer heuh areh euh gore gher he")
                # if camefromtrain == True:
                # counterfromtrain += rc.get_delta_time()
                # if counterfromtrain > 12 :
                #    allow = True
                
                # if var2 == True:
                #     if color_img_2 is not None:
                #         contours_green = rc_utils.find_contours(color_img_2, GREEN[0], GREEN[1])
                #         contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)
                #         #print(contour_green)
                #         if contour_green is not None:
                #             contour_center_green = rc_utils.get_contour_center(contour_green)
                #             rc_utils.draw_contour(color_img_2, contour_green, rc_utils.ColorBGR.red.value)
                #             rc_utils.draw_circle(color_img_2, contour_center_green,rc_utils.ColorBGR.red.value)
                #             #rc.display.show_color_image(color_img)
                #             angle = rc_utils.remap_range(contour_center_green[1], 0, rc.camera.get_width(), -1, 1)               
                #         else: 
                #             angle = 0
                #         if just_finished_wall_following:
                #             counter += rc.get_delta_time()
                #             if 1 < counter < 1.25:
                #                 angle = 1
                #         if counter_wall == 2:
                #             counter += rc.get_delta_time()
                #             if 4 < counter < 5:
                #                 angle = 1
                #         if contour_green is None:
                #             angle = 0.8
                
        if allow is True:
            if contour_green is not None:
                if green_var is True:
                    green_left += 1
                    green_var = False
            if green_left < 9:
                if contour_green is None and green_left %2 == 0:
                    print("turning right")
                    angle = 1
                    green_var = True
                elif contour_green is None and green_left %2 == 1:
                    print("left left")
                    angle = -1
                    green_var = True
                else:
                    green_var = False
            if green_left == 8:
                counter_final_stretch += rc.get_delta_time()
                if counter_final_stretch < 8:
                    print("PART BEFORE FINAL STRETCH")
                    speed = 0.13 # changeable
            if green_left == 9:
                counter_final_stretch += rc.get_delta_time()
                if counter_final_stretch > 9:
                    print("FINAL STRETCH")
                    speed = 0.3
            if green_left == 8 and contour_green is None:
                angle = 0.65
        if allow_2 is True:
            speed = 0.15 #changeable
            if contour_green is not None:
                if green_var is True:
                    green_left += 1
                    green_var_2 = False
            if green_left < 9:
                if contour_green is None and green_left %2 == 1:
                    print("right right")
                    angle = 1
                    green_var = True
                elif contour_green is None and green_left %2 == 0:
                    print("left left")
                    angle = -1
                    green_var = True
                else:
                    green_var = False
            if green_left == 7:
                counter_train += rc.get_delta_time()
                if counter_train < 0.5:
                    angle = -1
                    print("turning left")
                elif counter_train < 1.45:
                    angle = 0
                    print("going straight")
                elif counter_train < 3.1:
                    angle = 1.5
                    print("turning right")
                elif counter_train < 6.6:
                    green_var = False
                    green_left = 2
                    allow = True
                    allow_2 = False
                    allow_3 = True
                


    #CONE SLALOMING
    if curr_state == State.cone_slaloming:
        if var is False:
            speed = 0.07
            if counter_final < 3.6:
                angle = 0
            elif counter_final < 7.6:
                angle = 2.5 
            else:
                var = True
        
        else:
            if counter_final > 37 and contours_green is not None:
                curr_state = State.green_line
                camefromcone = True
                #allow_2 = True
            speed = 0.07
            if color_img is not None:
                cone_distance = 40
                contours_red = rc_utils.find_contours(color_img, RED[0], RED[1])
                contour_red = rc_utils.get_largest_contour(contours_red, 100)
                if contour_red is not None:
                    contour_center_red = rc_utils.get_contour_center(contour_red)
                    rc_utils.draw_contour(color_img, contour_red, rc_utils.ColorBGR.red.value)
                    rc_utils.draw_circle(color_img, contour_center_red,rc_utils.ColorBGR.yellow.value)
                if contour_red is None:
                    contour_center_red = None
                
                contours_blue = rc_utils.find_contours(color_img, BLUE[0], BLUE[1])
                contour_blue = rc_utils.get_largest_contour(contours_blue, 100)
                if contour_blue is not None:
                        contour_center_blue = rc_utils.get_contour_center(contour_blue)
                        rc_utils.draw_contour(color_img, contour_blue, rc_utils.ColorBGR.red.value)
                        rc_utils.draw_circle(color_img, contour_center_blue,rc_utils.ColorBGR.yellow.value)
                
                color_img_temp = rc_utils.crop(color_img, (2* height//5, 0), (height, width))
                rc.display.show_color_image(color_img_temp)


                if curr_state_COLOR == State_COLOR.align_blue:
                    angle = rc_utils.remap_range(contour_center_blue[1], 0, width, -1, 1) 
                    if depth_image_new[contour_center_blue[0]][contour_center_blue[1]] < 50:
                        counter_COLOR = 0
                        curr_state_COLOR = State_COLOR.avoid_blue
                        
                if curr_state_COLOR == State_COLOR.avoid_blue:
                    # if contour_green is not None:
                    #     curr_state = State.green_line
                    if counter_COLOR < 0.6:
                        print("turn 1")
                        angle = -2
                    elif counter_COLOR < 1:
                        print("turn 2")
                        angle = 0
                    elif counter_COLOR < 2.2:
                        print("turn 3")
                        angle = 2
                    else:
                        print("else - align red")
                        curr_state_COLOR = State_COLOR.align_red
                        cone_counter += 1
                        if cone_counter == 7:
                            curr_state = State.green_line
                    counter_COLOR += rc.get_delta_time()

                if curr_state_COLOR == State_COLOR.align_red:
                    speed = 0.07
                    if contour_center_red is not None:
                        angle = rc_utils.remap_range(contour_center_red[1], 0, width, -1, 1) 
                    #print(depth_image_new[contour_center_red[0]][contour_center_red[1]])
                    if depth_image_new is not None:
                        if depth_image_new[contour_center_red[0]][contour_center_red[1]] < 50:
                            counter_COLOR = 0
                            curr_state_COLOR = State_COLOR.avoid_red
                        
                if curr_state_COLOR == State_COLOR.avoid_red:
                    speed = 0.07
                    # if depth_image_new[contour_center_red[0]][contour_center_red[1]] > cone_distance and counter == 0:
                    #     curr_state_COLOR = State_COLOR.align_red
                    # else:
                    # if contour_green is not None:
                    #     curr_state = State.green_line
                    if True:
                        if counter_COLOR < 0.6:
                            angle = 2
                        elif counter_COLOR < 1:
                            angle = 0
                        elif counter_COLOR < 2.2:
                            angle = -2
                        else:
                            curr_state_COLOR = State_COLOR.align_blue
                            cone_counter += 1
                        counter_COLOR += rc.get_delta_time()
        counter_final += rc.get_delta_time()
            
    
    #WALL FOLLOWING
    if curr_state == State.wall_following:
        speed = 0.15
        if counter_wall == 1:
            #curr_state = State.ar_tag_slaloming
            curr_state = State.stop
        if counter < 1:
            # speed = 0.1
            angle = 0
            counter += rc.get_delta_time()
        else: 
            diff = right_front - left_front
            mult = 0.1
            if diff > 10:
                mult = 2
            if diff > 40:
                # speed = 0
                pass
            angle = rc_utils.remap_range(diff, -20, 20, -1, 1) * mult
        
        if forward_distance > 500:
            just_finished_wall_following = True
            counter = 0
            counter_wall += 1
            curr_state = State.green_line

    #PURPLE LINE FOLLOWING
    if curr_state == State.purple_line_following:
        speed = 0.16
        camefrompurple = True
        if len(markers) > 0 and ar_marker_near and marker.get_id() != 1:
            curr_state = State.ar_tag
    
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

        if detected_color is None:
            curr_state = State.green_line

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
            angle = 0.7

    
    contours_green_ar = rc_utils.find_contours(color_img, GREEN[0], GREEN[1])
    if contours_green_ar:
        cropped = rc_utils.crop(color_img, (rc.camera.get_height() // 4 * 3, 0),
            (rc.camera.get_height(), rc.camera.get_width()))
        contours = rc_utils.find_contours(cropped, GREEN[0], GREEN[1])
        # contour_green = rc_utils.get_largest_contour(cropped, 190)
        contour_green = rc_utils.get_largest_contour(contours_green, 100) # changed from MIN_CONTOUR_AREA
    
    #AR TAG SLALOMING
    if curr_state == State.ar_tag_slaloming:
        camefrompurple = False
        speed = 0.125
        # if counter_AR < 0.02:
        #     speed = 0.06
        #     counter_AR += rc.get_delta_time()
        
        contours_BLACK = rc_utils.find_contours(color_img, BLACK[0], BLACK[1])
        if contours_BLACK:
            contour_BLACK = rc_utils.get_largest_contour(contours_BLACK, MIN_CONTOUR_AREA)
            contour_center_BLACK = rc_utils.get_contour_center(contour_BLACK)
        else:
            contour_BLACK = None

        if curr_state_ARS == State_ARS.search: 
            if contours_green is not None and contour_BLACK is None:
                contours = rc_utils.find_contours(cropped, GREEN[0], GREEN[1])
                contour_green = rc_utils.get_largest_contour(contours_green, 100)
                if contour_green is not None:
                    print("here")
                    curr_state = State.green_line
            if last_direction_left == True:
                angle = 1
            elif last_direction_right == True:
                angle = -1
            if contour_BLACK is not None and len(markers) > 0: #sees ar tag
                if marker.get_orientation() == Orientation.LEFT:
                    orientation = "left"
                    curr_state_ARS = State_ARS.approach
                elif marker.get_orientation() == Orientation.RIGHT:
                    orientation = "right"
                    curr_state_ARS = State_ARS.approach
            else:
                #speed = 0.05
                pass
        
        pillar_distance = 65       
        if curr_state_ARS == State_ARS.approach:
            last_direction_left = False
            last_direction_right = False
            counter_AR = 0
            if depth_img[contour_center_BLACK[0]][contour_center_BLACK[1]] < pillar_distance:
                if orientation == "right":
                    curr_state_ARS = State_ARS.avoid_right
                if orientation == "left":
                    curr_state_ARS = State_ARS.avoid_left
            else:
                if contour_BLACK is not None:
                    if last_direction_left is False and last_direction_right is False:
                        kP = 1
                        angle = rc_utils.remap_range(contour_center_BLACK[1], 0, width, -1, 1) *kP
                    
        if curr_state_ARS == State_ARS.avoid_left:
            speed = 0.05
            if counter_AR < 0.7:
                print("angle .7")
                angle = -1.3
            elif counter_AR < 1.8:
                print("angle 0")
                angle = 0
            elif counter_AR < 3.2:
                angle = 0.8
                print("angle -0.7")
            else:
                print("end of avoid state")
                curr_state_ARS = State_ARS.search
            counter_AR += rc.get_delta_time()
            last_direction_left = True
            
        if curr_state_ARS == State_ARS.avoid_right:
            speed = 0.05
            if counter_AR < 0.7:
                print("angle .7")
                angle = 1.3
            elif counter_AR < 1.8:
                print("angle 0")
                angle = 0
            elif counter_AR < 3.2:
                angle = -0.8
                print("angle -0.7")
            else:
                print("end of avoid state")
                curr_state_ARS = State_ARS.search
            counter_AR += rc.get_delta_time()
            last_direction_right = True
                    
        print(curr_state_ARS)
        
    #Elevator
    if curr_state == State.elevator:
        contour_center_blue = width//2
        counter_final = 0
        potential_colors = [RED, BLUE]
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
        if True:
            if marker is not None:
                print(detected_color)
                print(ar_marker_elevator)
                if detected_color == "red":
                    
                    if not ar_marker_near:
                        speed = 0.3
                    else:
                        speed = 0
                elif detected_color == "blue":
                    contours_blue = rc_utils.find_contours(color_img, BLUE[0], BLUE[1])
                    if contours_blue:
                        contour_blue = rc_utils.get_largest_contour(contours_blue, MIN_CONTOUR_AREA)
                    if contour_blue is not None:
                        contour_center_blue = rc_utils.get_contour_center(contour_blue)
                    angle = rc_utils.remap_range(contour_center_blue[1]+50, 0, width, -1, 1)
                    speed = 2
            else:
                curr_state = State.green_line
                speed = 0.1

    if curr_state == State.stop:
        counter_stop += rc.get_delta_time()
        if counter_stop < 0.7:
            speed = -0.7
        else:
            counter_stop = 0
            curr_state = State.ar_tag_slaloming

    if curr_state == State.stop2:
        counter_stop_2 += rc.get_delta_time()
        if counter_stop_2 < 0.7:
            speed = -0.3
        else:
            counter_stop_2 = 0
            curr_state = State.cone_slaloming

    #Moving Trains
    if curr_state == State.trains:
        camefromtrain = True
        counter_train1 += rc.get_delta_time()
        if counter_train1 < 0.5:
            angle = 1
            speed = 0.1
        else:
            allow_2 = True
            allow = False
            curr_state = State.green_line
        # #allow_2 = True
        # allow = False
        # contours_green = rc_utils.find_contours(color_img_2, GREEN[0], GREEN[1])
        # contour_green = None
        # if contours_green:
        #     contour_green = rc_utils.get_largest_contour(contours_green, 30)
        # if right_front > 550:
        #     curr_state = State.green_line
        #     camefromtrain = True
        #     print("helooooooooooooooooooooooooo")
        #     #allow = True
        # else:
        #     depth_img_train = rc_utils.crop(depth_img, (0, 0), (height//2, width))
        #     print(depth_img_train[5* height//12][width//2])
        #     distance = 20
        #     left_point = depth_img_train[5*height//12][(width//2) - distance]
        #     middle_point = depth_img_train[5* height//12][width//2]
        #     right_point = depth_img_train[5*height//12][(width//2) + distance]
            
        #     if (left_point and middle_point and right_point) < 125:
        #         speed = 0
        #     else: 
        #         speed = 0.14
        # #curr_state = State.green_line
        # # print("hello")
        # # green_var = False
        # # green_left = 2
        # # allow_2 = True

        



    
    if finallyfinal == True:
        speed = 0.3
    # speed = 0.1
    if len(markers):
        print("Marker ID: " + str(marker.get_id()))
    print(f"curr_state: {curr_state}")
    print(curr_state_COLOR)
    #print(f"cone_counter: {cone_counter}")
    #print(just_finished_wall_following)
    print(speed)
    # print(allow)
    # print(f"allow2: {allow_2}")
    #print(green_var, "   ", green_left, "   ", angle)
    print(counter_jump)
    # print(len(contours_green))
    # print(f"front right window: {right_front}")
    #speed = 0
    #print(depth_img[(height//3)][width//2])
    if curr_state != State.cone_slaloming:
        angle = rc_utils.clamp(angle, -1, 1)
    

    rc.drive.set_speed_angle(speed, angle) 
    


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()