#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import subprocess

import roslaunch
import rospy

# we will use the pin numbering to match the pins on the Pi, instead of the
# GPIO pin outs (makes it easier to keep track of things)

GPIO.setmode(GPIO.BOARD)

# use the same pin that is used for the reset button (one button to rule them all!)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

oldButtonState1 = True

while True:
    # grab the current button state
    buttonState1 = GPIO.input(5)

    # check to see if button has been pushed
    if buttonState1 != oldButtonState1 and buttonState1 == False:

        rospy.init_node('livox_drone', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jiangchuan/catkin_ws/src/livox_drone/launch/livox_drone.launch"])
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/pi/catkin_ws/src/livox_drone/launch/livox_drone.launch"])
        launch.start()
        rospy.loginfo("started")

        oldButtonState1 = buttonState1

    time.sleep(.1)
