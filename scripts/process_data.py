#!/usr/bin/env python

import rospy, rospkg
from athena_transform.msg import HomogeneousTransform
from athena_transform.conversions import *
import yaml, os

class SelineProcessData:
    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        self.sub_tf = rospy.Subscriber('/seline/est_world_frame', HomogeneousTransform, self.tf_callback)
        self.listener = tf.TransformListener()

        self.camera_optical_frame = rospy.get_param("/seline/camera_optical_frame")
        self.world_frame = rospy.get_param("/seline/world_frame")
        self.camera_link_name = rospy.get_param("/seline/camera_link_name")
        self.output_file = rospy.get_param("/seline/processed_calibration_filename")
        self.output_file = rospkg.RosPack().get_path("seline") +"/results/" + self.output_file

        print self.output_file


    def tf_callback(self, msg):
        (xyz, quat) = self.listener.lookupTransform(self.camera_optical_frame, self.camera_link_name, rospy.Time(0))

        optical_to_base = transf.translation_matrix([xyz[0], xyz[1], xyz[2]])
        optical_to_base = np.dot(optical_to_base, transf.quaternion_matrix([quat[3], quat[0], quat[1], quat[2]]))
        #print optical_to_base

        cam_to_world = homogeneous_vector_to_matrix(msg.transform)
        world_to_cam = transf.inverse_matrix(cam_to_world)

        broadcast_matrix_as_tf(cam_to_world, self.camera_optical_frame, 'est_world')
        broadcast_matrix_as_tf(world_to_cam, self.world_frame, 'est_camera_optical_frame')

        world_to_cam_base = np.dot(world_to_cam, optical_to_base)
        broadcast_matrix_as_tf(world_to_cam_base, self.world_frame, 'est_camera_base_link')

        # Obtain the camera to world transform, sometimes in URDFs we want the camera_link
        # to be the root of the tree as described by the URDF
        cam_base_to_world = transf.inverse_matrix(world_to_cam_base)
        print '\n*** Est. world to camera frame ***'
        xyz = transf.translation_from_matrix(cam_base_to_world)
        rpy = transf.euler_from_matrix(cam_base_to_world)
        # print 'xyz:', xyz[0], ' ', xyz[1], ' ', xyz[2]
        # print 'rpy:', rpy[0], ' ', rpy[1], ' ', rpy[2]

        print '<link name=\"'+self.camera_link_name+'\"/>'
        print '<joint name=\"camera_to_robot\" type=\"fixed\">'
        print '  <parent link=\"'+self.camera_link_name+'\"/>'
        print '  <child link=\"'+self.world_frame+'\"/>'
        print '  <origin xyz=\"'+str(xyz[0])+' '+str(xyz[1])+' '+str(xyz[2])+'\" rpy=\"'+str(rpy[0])+' '+str(rpy[1])+' '+str(rpy[2])+'\" />'
        print '</joint>'

        with open(self.output_file, 'w') as outfile:
            yaml.dump(cam_base_to_world.flatten(), outfile, default_flow_style=False)


if __name__ == '__main__':
    rospy.init_node('SelineProcessData')
    node = SelineProcessData()

    while not rospy.is_shutdown():
        #node.runOnce()
        node.loop_rate.sleep()
