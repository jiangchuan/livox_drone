#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import subprocess

# we will use the pin numbering to match the pins on the Pi, instead of the
# GPIO pin outs (makes it easier to keep track of things)

GPIO.setmode(GPIO.BOARD)

# use the same pin that is used for the reset button (one button to rule them all!)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    # grab the current button state
    buttonState1 = GPIO.input(5)

    # check to see if button has been pushed
    if buttonState1 == False:
      print('Entered 1')
      subprocess.call('/ bin/bash - c "source / opt/ros/kinetic/setup.bash; source / home/pi/catkin_ws/devel/setup.bash; / usr/bin/python / opt/ros/kinetic/bin/roslaunch livox_drone livox_drone.$
      print('Entered 2')

    time.sleep(.1)
