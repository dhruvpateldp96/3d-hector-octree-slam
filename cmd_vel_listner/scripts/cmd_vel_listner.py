#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
import time
def callback(cmdVelocity):
    baseVelocity = TwistStamped()
    baseVelocity.twist = cmdVelocity
    now = rospy.get_rostime()
    baseVelocity.header.stamp.secs = now.secs
    baseVelocity.header.stamp.nsecs = now.nsecs
    baseVelocityPub = rospy.Publisher('base_velocity', TwistStamped, queue_size=10)
    baseVelocityPub.publish(baseVelocity)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", baseVelocity)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cmd_vel_listener', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
