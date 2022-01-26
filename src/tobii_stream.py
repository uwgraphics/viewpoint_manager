#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import tobii_research as tr
import time


class GazePublisher():
    def __init__(self):
        self.gaze_pub = rospy.Publisher(
            '/viewpoint_manager/gaze', Float32MultiArray, queue_size=10)
        self.gaze_data = Float32MultiArray()

    def gaze_data_callback(self, gaze_data):
        self.gaze_data = Float32MultiArray()
        self.gaze_data.data = [gaze_data['left_gaze_point_on_display_area'][0],
                               gaze_data['left_gaze_point_on_display_area'][1],
                               gaze_data['right_gaze_point_on_display_area'][0],
                               gaze_data['right_gaze_point_on_display_area'][1]]

    def publish(self):
        self.gaze_pub.publish(self.gaze_data)


found_eyetrackers = tr.find_all_eyetrackers()
my_eyetracker = found_eyetrackers[0]
print("Publishing gaze data for " + my_eyetracker.model + " ...")

rospy.init_node('gaze_pub')
r = rospy.Rate(300)
gaze_publisher = GazePublisher()

my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA,
                           gaze_publisher.gaze_data_callback, as_dictionary=True)

while not rospy.is_shutdown():
    gaze_publisher.publish()
    r.sleep()
    if rospy.is_shutdown():
        my_eyetracker.unsubscribe_from(
            tr.EYETRACKER_GAZE_DATA, gaze_publisher.gaze_data_callback)

"""
Debugging code

print("Address: " + my_eyetracker.address)
print("Model: " + my_eyetracker.model)
print("Name (It's OK if this is empty): " + my_eyetracker.device_name)
print("Serial number: " + my_eyetracker.serial_number)
eyetracker_address = 'tobii-ttp://TPNA1-030119056172'
eyetracker = tr.EyeTracker(eyetracker_address)

"""
