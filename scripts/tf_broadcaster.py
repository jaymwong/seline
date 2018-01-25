#!/usr/bin/env python

import rospy, rospkg
from transform_conversions.conversions import *
import yaml, os

class SelineTFBroadcaster:
    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        self.world_frame = rospy.get_param("/seline/world_frame")
        self.camera_link_name = rospy.get_param("/seline/camera_link_name")
        self.output_file = rospy.get_param("/seline/processed_calibration_filename")
        self.output_file = rospkg.RosPack().get_path("seline") +"/results/" + self.output_file

        print self.output_file
        with open(self.output_file, 'r') as stream:
            self.calib_result = yaml.load(stream)
        self.calib_result = homogeneous_vector_to_matrix(self.calib_result)

    def runOnce(self):
        broadcast_matrix_as_tf(self.calib_result, self.camera_link_name, self.world_frame)

if __name__ == '__main__':
    rospy.init_node('SelineTFBroadcaster')
    node = SelineTFBroadcaster()

    while not rospy.is_shutdown():
        node.runOnce()
        node.loop_rate.sleep()
