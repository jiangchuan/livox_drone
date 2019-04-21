#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import subprocess


pinNum = 5  # use the same pin that is used for the reset button, one button to rule them all!

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(pinNum, GPIO.IN, pull_up_down=GPIO.PUD_UP)

maxCycle = 100
shortThresh = 10
longThresh = 30
while True:
    if GPIO.input(pinNum) == 0:
        i = 0
        for i in range(maxCycle):
            if GPIO.input(pinNum):  # not pressed any more
                break
            time.sleep(0.1)

        if i < shortThresh:  # short press
            subprocess.call('/bin/bash -c "source /opt/ros/kinetic/setup.bash; source /home/pi/catkin_ws/devel/setup.bash; /usr/bin/python /opt/ros/kinetic/bin/roslaunch livox_drone livox_drone.launch"', shell=True)
        elif i >= longThresh:
            subprocess.call('halt', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(5)  # sleep some time so that it does not go to next iteration immediately

    time.sleep(0.1)


#oldState = True
# while True:
#    newState = GPIO.input(pinNum)
#    # check to see if button has been pushed
#    if newState != oldState and newState == False:
#        print('the single press')
#        oldState = newState
#    time.sleep(.1)

# while True:
#    # set an interrupt on a falling edge and wait for it to happen
#    GPIO.wait_for_edge(pinNum, GPIO.FALLING)
#    # we got here because the button was pressed.
#    # wait for 3 seconds to see if this was deliberate

#    time.sleep(3)
#    # check the button level again
#    if GPIO.input(pinNum):
#        # still pressed, it must be a serious request; shutdown Pi
#        print('the short press')
#    else:
#	print('the long press')


#    longPress = True
#    for i in range(30):
#      if GPIO.input(pinNum): # not pressed any more
#	print('test gpio input')
#        longPress = False
#        break
#      time.sleep(0.1)

#    if longPress:
#      print('long press\n')
#    else:
#      print('single press\n')

#    time.sleep(0.5)


#     time.sleep(3)
#     # check the button level again
#     if GPIO.input(pinNum) == False:
#         # still pressed, it must be a serious request; shutdown Pi
#         subprocess.call(['halt'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


# while True:
#     # grab the current button state
#     buttonState1 = GPIO.input(pinNum)

#     # check to see if button has been pushed
#     if buttonState1 == False:
#         print('Entered 1')
#         subprocess.call('/bin/bash -c "source /opt/ros/kinetic/setup.bash; source /home/pi/catkin_ws/devel/setup.bash; /usr/bin/python /opt/ros/kinetic/bin/roslaunch livox_drone livox_drone.launch"', shell=True)
#         print('Entered 2')

#     time.sleep(.1)
