"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
counter = 0
isDrivingCircle = False
rc = racecar_core.create_racecar()

# Put any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()
    global counter
    global isDrivingCircle
    global isDrivingSquare
    global isDrivingFigure8
    global isDrivingTriangle
    counter = 0
    isDrivingCircle = False
    isDrivingSquare = False
    isDrivingFigure8 = False
    isDrivingTriangle = False

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
        "    Y button = drive in a triangle\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global counter
    global isDrivingCircle
    global isDrivingSquare
    global isDrivingFigure8
    global isDrivingTriangle
    # TODO (warmup): Implement acceleration and steering
    rc.drive.set_speed_angle(rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    - rc.controller.get_trigger(rc.controller.Trigger.LEFT), 
    rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0])

    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        counter = 0
        isDrivingCircle = True
    if isDrivingCircle:
        counter += rc.get_delta_time()
        if counter < 6.4:
            rc.drive.set_speed_angle(1, -1)
    
    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        counter = 0
        isDrivingSquare = True
    if isDrivingSquare:
        counter += rc.get_delta_time()
        if counter < 1:
            rc.drive.set_speed_angle(1, 0)
        elif counter < 2.6:
            rc.drive.set_speed_angle(1,-1)
        elif counter < 3.6:
            rc.drive.set_speed_angle(1, 0)
        elif counter < 5:
            rc.drive.set_speed_angle(1,-1)
        elif counter < 6:
            rc.drive.set_speed_angle(1, 0)
        elif counter < 7.2:
            rc.drive.set_speed_angle(1,-1)
        elif counter < 8.2:
            rc.drive.set_speed_angle(1, 0)
        elif counter < 9.5:
            rc.drive.set_speed_angle(1,-1)
        
    
    if rc.controller.was_pressed(rc.controller.Button.X):
        print("Driving in a Figure 8...")
        counter = 0
        isDrivingFigure8 = True
    if isDrivingFigure8:
        counter += rc.get_delta_time()
        if counter < 3:
            rc.drive.set_speed_angle(1,-1)
        elif counter < 8:
            rc.drive.set_speed_angle(1,1)
        elif counter < 12:
            rc.drive.set_speed_angle(1,-1)

    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Driving in a Triangle...")
        counter = 0
        isDrivingTriangle = True
    if isDrivingTriangle:
        counter += rc.get_delta_time()
        if counter < 1.5:
            rc.drive.set_speed_angle(1,-1)
        elif counter < 3.5:
            rc.drive.set_speed_angle(1,0)
        elif counter < 5:
            rc.drive.set_speed_angle(1,-1)
        elif counter < 6:
            rc.drive.set_speed_angle(1,0)
        elif counter < 7.7:
            rc.drive.set_speed_angle(1,-1)
        elif counter <8.5:
            rc.drive.set_speed_angle(1,0)








    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
