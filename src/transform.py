#!/usr/bin/python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int8, UInt8, Bool
import copy 

rospy.init_node('frame_transform')

# rostopic pub /viewpoint_manager/mode std_msgs/Int8 3

class GripperHandler():
    def __init__(self):
        self.grasp_raw_sub = rospy.Subscriber(
            '/robot_state/grasping', Bool, self.get_gripper_state)
        self.grasp_toggle_pub = rospy.Publisher(
            '/robot_state/grasping_toggle', Bool, queue_size=3)
        self.curr_gripper_state = True 
        self.prev_gripper_state = True
        self.toggle_state = Bool()
        self.toggle_state.data = False

    def get_gripper_state(self, msg):
        self.curr_gripper_state = msg.data
        if self.curr_gripper_state != self.prev_gripper_state and self.prev_gripper_state:
            self.toggle_state.data = not self.toggle_state.data
            self.grasp_toggle_pub.publish(self.toggle_state)
        
        self.prev_gripper_state = self.curr_gripper_state


class ActiveDisplayHandler():
    def __init__(self):
        self.display_bounds_sub = rospy.Subscriber(
            '/viewpoint_interface/display_bounds', Float32MultiArray, self.get_display_bounds)
        self.gaze_sub = rospy.Subscriber(
            '/viewpoint_manager/gaze', Float32MultiArray, self.get_gaze_data)
        self.mode_sub = rospy.Subscriber(
            '/viewpoint_manager/mode', Int8, self.get_mode)
        self.active_display_pub = rospy.Publisher(
            '/viewpoint_interface/active_display', UInt8, queue_size=3)
        self.win_size = 2
        self.gaze_window = [np.array([np.NaN, np.NaN])] 
        self.x_offset = 0
        self.y_offset = 28
        self.offset_array = np.array([self.x_offset, self.y_offset])
        self.x_max = 1920
        self.y_max = 1080
        self.gaze_data = (np.NaN, np.NaN, np.NaN, np.NaN)
        self.prev_active_display = 0
        self.active_display = UInt8()
        self.display_bounds = [None]
        self.mode = 1

    def get_mode(self, msg):
        self.mode = msg.data

    def get_display_bounds(self, msg):
        self.display_bounds = msg.data
        self.displays = int(len(self.display_bounds)/4.0)

    def get_gaze_data(self, msg):
        if msg.data:
            self.gaze_data = self.gaze_coordinates(msg.data)
            self.gaze_window.append(self.gaze_data)
            if len(self.gaze_window) > self.win_size:
                self.gaze_window = self.gaze_window[1:]

    def fix_active_display(self, d):
        self.active_display.data = d
        self.active_display_pub.publish(self.active_display)

    def get_active_display(self):
        coordinates = np.nanmean(np.array(self.gaze_window), axis=0)
        flag_found_active_display = False
        if not np.isnan(coordinates[0]) and not self.display_bounds[0] == None:
            display_bounds = np.array(
                self.display_bounds).reshape((self.displays*2, 2))
            diff  = display_bounds - coordinates
            for d in range(self.displays):
                if not flag_found_active_display:
                    if diff[d*2 + 0][0] < 0 and diff[d*2 + 0][1] < 0 and diff[d*2 + 1][0] > 0 and diff[d*2 + 1][1] > 0:
                        flag_found_active_display = True
                        self.active_display.data = d

        if not self.active_display.data == self.prev_active_display:
            self.active_display_pub.publish(self.active_display)
        self.prev_active_display = self.active_display.data

    def gaze_coordinates(self, gaze_tuple):
        left = False
        right = False
        if not np.isnan(gaze_tuple[0]):
            left = True
        if not np.isnan(gaze_tuple[2]):
            right = True
        if left or right:
            if left and right:
                coordinates = [int(((gaze_tuple[0] + gaze_tuple[2])/2.0)*self.x_max),
                               int(((gaze_tuple[1] + gaze_tuple[3])/2.0)*self.y_max)]
            else:
                if left:
                    coordinates = [int(gaze_tuple[0]*self.x_max),
                                   int(gaze_tuple[1]*self.y_max)]
                if right:
                    coordinates = [int(gaze_tuple[2]*self.x_max),
                                   int(gaze_tuple[3]*self.y_max)]
            coordinates = np.array(coordinates) - self.offset_array
        else:
            coordinates = np.array([np.NaN, np.NaN])
        return coordinates


class MatrixHandler():
    def __init__(self):
        self.matrix_sub = rospy.Subscriber(
            '/viewpoint_interface/frame_matrix', Float32MultiArray, self.get_matrix)
        self.matrix_pub = rospy.Publisher(
            '/viewpoint_manager/camera_frame_matrix', Float32MultiArray, queue_size=10)
        self.msg_matrix = Float32MultiArray()
        self.frame_matrix = np.identity(3)
        self.cam_matrix = np.identity(3)

    def get_matrix(self, msg):
        self.cam_matrix = np.matrix([[msg.data[2], msg.data[6], msg.data[10]], [
            msg.data[0], msg.data[4], msg.data[8]], [msg.data[1], msg.data[5], msg.data[9]]])

    def publish(self):
        self.frame_matrix = copy.deepcopy(self.cam_matrix)
        self.frame_matrix[2] = [0, 0, 1]
        third_vec = np.cross(self.frame_matrix[1], self.frame_matrix[2])
        self.frame_matrix[0] = third_vec/np.linalg.norm(third_vec)
        self.msg_matrix.data = list(np.asarray(self.frame_matrix).flatten())
        self.matrix_pub.publish(self.msg_matrix)


r = rospy.Rate(100)
matrix_handler = MatrixHandler()
active_display_handler = ActiveDisplayHandler()
gripper = GripperHandler()

while not rospy.is_shutdown():
    matrix_handler.publish()
    mode = active_display_handler.mode
    if mode == 1:
        active_display_handler.fix_active_display(0)
    elif mode == 2:
        continue
    elif mode == 3:
        active_display_handler.get_active_display()
    r.sleep()
