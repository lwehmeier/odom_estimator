#!/usr/bin/python
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3, Vector3Stamped
import struct
import tf2_ros
import tf2_geometry_msgs
BASE_FRAME= "base_footprint"
ODOM_FRAME = "odom"
UPDATE_PERIOD = 0.1
global odom_pub
global estimated_position
global estimated_orientation
estimated_position = Vector3()
estimated_orientation = Vector3()
global tfBuffer
global last_cmdvel
last_cmdvel = Twist()
global has_odom
has_odom = False
def callback_cmdvel(twist):
    global last_cmdvel
    last_cmdvel = twist
    
def my_callback(event):
    try:
        transf = tfBuffer.lookup_transform(BASE_FRAME, "map", rospy.Time(),timeout=rospy.Duration(2))
        rotVec = PoseStamped()
        rotVec.header.frame_id="map"
        rotVec.pose.orientation.w = 1.0
        rotVec = tf2_geometry_msgs.do_transform_pose(rotVec, transf)
        quaternion = [rotVec.pose.orientation.x, rotVec.pose.orientation.y, rotVec.pose.orientation.z, rotVec.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        estimated_orientation.z = yaw
        travel_x = last_cmdvel.linear.x * UPDATE_PERIOD
        travel_y = last_cmdvel.linear.y * UPDATE_PERIOD
        travel_x = (travel_x * cos(estimated_orientation.z) + travel_y * sin(estimated_orientation.z));
        travel_y = (travel_x * sin(estimated_orientation.z) + travel_y * cos(estimated_orientation.z));
        estimated_position.x = estimated_position.x + travel_x/2
        estimated_position.y = estimated_position.y + travel_y/2
        omega = last_cmdvel.angular.z * UPDATE_PERIOD
        if not has_odom:
            estimated_orientation.z += omega
        
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = ODOM_FRAME
        odom_quat = tf.transformations.quaternion_from_euler(estimated_orientation.z,0,0,"rzyx")
        odom.pose.pose = Pose(Point(estimated_position.x, estimated_position.y, 0), Quaternion(*odom_quat))
        odom.child_frame_id = BASE_FRAME
        odom.twist.twist = Twist(Vector3(travel_x, travel_y, 0), Vector3(0,0,omega))
        odom_pub.publish(odom)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        print(ex)
        pass

rospy.init_node('odom_estimator')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
odom_pub = rospy.Publisher("/estimator/odom", Odometry, queue_size=3)
r = rospy.Rate(1.0/UPDATE_PERIOD)
rospy.Timer(rospy.Duration(UPDATE_PERIOD), my_callback)
rospy.Subscriber("/cmd_vel", Twist, callback_cmdvel)
rospy.spin()
