#!/usr/bin/env python  
import roslib

import rospy

import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

def callback(msg):
	
	#br = tf.TransformBroadcaster()
	#br.sendTransform((1, 1, msg.data),
	#	     tf.transformations.quaternion_from_euler(0, 0, 0),
	#	     rospy.Time.now(),
	#	     'base_frame',
	#	     "map")
	callback.height=msg.data
	return callback.height
	
	
def callback1(msg):
	A=msg.pose.position.x
	B=msg.pose.position.y
	C=msg.pose.orientation.z
	D=msg.pose.orientation.w
	broadcaster(A,B,callback.height,C,D)
	

def broadcaster(A,B,height,C,D):
	br = tf.TransformBroadcaster()
	br.sendTransform((A,B,height),
		     (0,0,C,D),
		     rospy.Time.now(),
		     'base_frame',
		     "map")
	rospy.loginfo(height)
	rospy.loginfo('yolo')

if __name__ == '__main__':
	height=0
	A=0
	B=0
	c=0
	D=0
	rospy.init_node('turtle_tf_broadcaster')		
	rospy.Subscriber('/mavros/global_position/rel_alt',
		     Float64,
		     callback)
	rospy.Subscriber('/slam_out_pose', PoseStamped, callback1)
	rospy.spin()
