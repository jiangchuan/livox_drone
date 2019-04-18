import roslaunch
import rospy

rospy.init_node('livox_drone', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
#launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jiangchuan/catkin_ws/src/livox_drone/launch/livox_drone.launch"])
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/pi/catkin_ws/src/livox_drone/launch/livox_drone.launch"])
launch.start()
rospy.loginfo("started")
