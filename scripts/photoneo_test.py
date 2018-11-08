#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from visualization_msgs.msg import Marker
import sys
import copy

def choose_sensor(sensor_type):
    if sensor_type == "xl":
        print "Photoneo XL"
        minimum_distance = 1.5
        maximum_distance = 4.0
        x = 2.2 # @2.4[m]
        y = 1.8 # @2.4[m]
        at = 2.4
        color = [1.0,0,0]
    elif sensor_type == "l":
        print "Photoneo L"
        minimum_distance = 0.9
        maximum_distance = 2.2
        x = 1.2 # @1.3[m]
        y = 1.0 # @1.3[m]
        at = 1.3
        color = [0,0,1.0]
    elif sensor_type == "m":
        print "Photoneo M"
        minimum_distance = 0.45
        maximum_distance = 1.13
        x = 0.6 # @0.7[m]
        y = 0.5 # @0.7[m]
        at = 0.7
        color = [0,1.0,0]
    elif sensor_type == "s":
        print "Photoneo S"
        minimum_distance = 0.23
        maximum_distance = 0.59
        x = 0.3 # @0.33[m]
        y = 0.25 # @0.33[m]
        at = 0.33
        color = [1.0,0,1.0]
    elif sensor_type == "xs":
        print "Photoneo XS"
        minimum_distance = 0.1
        maximum_distance = 0.14
        x = 0.1 # @0.13[m]
        y = 0.06 # @0.13[m]
        at = 0.13
        color = [0,1.0,1.0]
    else:
        minimum_distance = 0.0
        maximum_distance = 0.0
        x = 0.0
        y = 0.0
        at = 0.0
        color = [0,0,0]
        print "UNKNOWN"

    print "minimum_distance: "+str(minimum_distance)
    print "maximum_distance: "+str(maximum_distance)
    print "x: "+str(x)+" @ "+str(at)+"[m]"
    print "y: "+str(y)+" @ "+str(at)+"[m]"

    return minimum_distance, maximum_distance, x, y, at, color


if __name__ == '__main__':

    args = rospy.myargv(sys.argv)
    sensor_type = args[1]
    minimum_distance, maximum_distance, x, y, at, color = choose_sensor(args[1])

    sensor_frame_id = args[2]+"_rgb_optical_frame"

    rospy.init_node('TestPhotoneoRengeNode', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    sensor_range_pub = rospy.Publisher(args[2]+"/range", Marker, queue_size = 10)
    sensor_spec_pub = rospy.Publisher(args[2]+"/range/spec", Marker, queue_size = 10)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform("world",
                                                  sensor_frame_id,
                                                  rospy.Time(0),
                                                  rospy.Duration(1.0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        sensor_range_pose = geometry_msgs.msg.PoseStamped()
        sensor_range_pose.header.frame_id = sensor_frame_id
        sensor_range_pose.pose.orientation.w = 1.0
        sensor_range_pose.pose.position.z = minimum_distance + (maximum_distance-minimum_distance)/2.0
        sensor_range_pose_transformed = tf2_geometry_msgs.do_transform_pose(sensor_range_pose, transform)

        sensor_spec_pose = copy.deepcopy(sensor_range_pose)
        sensor_spec_pose.pose.position.z = at
        sensor_spec_pose_transformed = tf2_geometry_msgs.do_transform_pose(sensor_spec_pose, transform)

        sensor_range_marker = Marker()
        sensor_range_marker.header.frame_id = sensor_range_pose_transformed.header.frame_id
        sensor_range_marker.header.stamp = rospy.Time.now()
        sensor_range_marker.ns = args[2]+"/range/"
        sensor_range_marker.id = 0
        sensor_range_marker.type = 1
        sensor_range_marker.action = Marker.ADD
        sensor_range_marker.pose.position = sensor_range_pose_transformed.pose.position
        sensor_range_marker.pose.orientation = sensor_range_pose_transformed.pose.orientation
        sensor_range_marker.color.r = color[0]
        sensor_range_marker.color.g = color[1]
        sensor_range_marker.color.b = color[2]
        sensor_range_marker.color.a = 0.4
        sensor_range_marker.scale.x = x
        sensor_range_marker.scale.y = y
        sensor_range_marker.scale.z = maximum_distance - minimum_distance

        sensor_spec_marker = copy.deepcopy(sensor_range_marker)
        sensor_spec_marker.ns = args[2]+"/range/spec"
        sensor_spec_marker.pose.position = sensor_spec_pose_transformed.pose.position
        sensor_spec_marker.pose.orientation = sensor_spec_pose_transformed.pose.orientation
        sensor_spec_marker.color.r = 1.0
        sensor_spec_marker.color.g = 1.0
        sensor_spec_marker.color.b = 0.0
        sensor_spec_marker.scale.z = 0.025

        sensor_range_pub.publish(sensor_range_marker)
        sensor_spec_pub.publish(sensor_spec_marker)

        rate.sleep()
