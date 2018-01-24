import rospy
from transform_conversions.msg import HomogeneousTransform
from transform_conversions.conversions import *

class SelineProcessData:
    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        self.sub_tf = rospy.Subscriber('/seline/est_world_frame', HomogeneousTransform, self.tf_callback)
        self.listener = tf.TransformListener()

    def tf_callback(self, msg):
        (xyz, quat) = self.listener.lookupTransform('/camera_rgb_optical_frame', '/camera_link', rospy.Time(0))

        optical_to_base = transf.translation_matrix([xyz[0], xyz[1], xyz[2]])
        optical_to_base = np.dot(optical_to_base, transf.quaternion_matrix([quat[3], quat[0], quat[1], quat[2]]))
        print optical_to_base

        cam_to_world = homogeneous_vector_to_matrix(msg.transform)
        world_to_cam = transf.inverse_matrix(cam_to_world)

        broadcast_matrix_as_tf(cam_to_world, 'camera_rgb_optical_frame', 'est_world')
        broadcast_matrix_as_tf(world_to_cam, 'world', 'est_camera_optical_frame')

        world_to_cam_base = np.dot(world_to_cam, optical_to_base)
        broadcast_matrix_as_tf(world_to_cam_base, 'world', 'est_camera_base_link')

        world_to_cam_base = transf.inverse_matrix(world_to_cam_base)
        print '*** Est. world to camera frame ***'
        xyz = transf.translation_from_matrix(world_to_cam_base)
        rpy = transf.euler_from_matrix(world_to_cam_base)
        print 'xyz:', xyz[0], ' ', xyz[1], ' ', xyz[2]
        print 'rpy:', rpy[0], ' ', rpy[1], ' ', rpy[2]




if __name__ == '__main__':
    rospy.init_node('SelineProcessData')
    node = SelineProcessData()

    while not rospy.is_shutdown():
        #node.runOnce()
        node.loop_rate.sleep()
