#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import subprocess

# we will use the pin numbering to match the pins on the Pi, instead of the
# GPIO pin outs (makes it easier to keep track of things)

pinNum = 5
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# use the same pin that is used for the reset button (one button to rule them all!)
GPIO.setup(pinNum, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    # set an interrupt on a falling edge and wait for it to happen
    GPIO.wait_for_edge(pinNum, GPIO.FALLING)
    # we got here because the button was pressed.
    # wait for 3 seconds to see if this was deliberate

    longPress = True
    for i in range(30):
      if GPIO.input(pinNum): # not pressed any more
        longPress = False
        break
      time.sleep(0.1)
    
    if longPress:
      print('long press')
    else:
      print('single press')

    time.sleep(0.1)




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
