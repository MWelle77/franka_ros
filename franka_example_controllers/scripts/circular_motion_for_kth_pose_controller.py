#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import math
import tf

class CircularMotionPublisher:
    def __init__(self):
        self.sub = rospy.Subscriber('/panda/kth_cartesian_pose_effort_interface_controller/panda_x', PoseStamped, self.initial_pose_callback)
        self.pub = rospy.Publisher('/panda/kth_cartesian_pose_effort_interface_controller/panda/equilibrium_pose', PoseStamped, queue_size=10)
        self.sub_fs =rospy.Subscriber('/panda/franka_state_controller/franka_states', FrankaState, self.franka_state_callback)
        self.initial_pose_received = False
        self.initial_pose = PoseStamped()
        self.initial_pose_tf = PoseStamped()
        self.tf_listener = tf.TransformListener()

    def get_EE_pose(self, verbose=False, frame='/world', get_extended=True):
        """"
        frame is to define the reference frame over which we want to have the EE pose
        """
        try:
            # (trans,rot) = self.tf_listener.lookupTransform('/panda_link0', '/panda_EE', rospy.Time(0))
            (trans, rot) = self.tf_listener.lookupTransform(frame, '/panda_EE', rospy.Time(0))
            # print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("TF exception")
            return None, None
        
        # Accessing the translation components
        x = trans[0]
        y = trans[1]
        z = trans[2]

        # Accessing the rotation components
        qx = rot[0]
        qy = rot[1]
        qz = rot[2]
        qw = rot[3]

        if verbose:
            # Print the translation and rotation components
            print("Translation (world frame): x={}, y={}, z={}".format(x, y, z))
            print("Rotation (world frame): qx={}, qy={}, qz={}, qw={}".format(qx, qy, qz, qw))

        return [x, y, z], [qx, qy, qz, qw]

    def initial_pose_callback(self, msg):
        if not self.initial_pose_received:
            ee_pos, ee_or = self.get_EE_pose(frame='/panda_link0', get_extended=False)
            if not ee_pos is None:
                self.initial_pose_tf.pose.position.x=ee_pos[0]
                self.initial_pose_tf.pose.position.y=ee_pos[1]
                self.initial_pose_tf.pose.position.z=ee_pos[2]

                self.initial_pose_tf.pose.orientation.x=ee_or[0]
                self.initial_pose_tf.pose.orientation.y=ee_or[1]
                self.initial_pose_tf.pose.orientation.z=ee_or[2]
                self.initial_pose_tf.pose.orientation.w=ee_or[3]
                self.initial_pose = msg
                self.initial_pose_received = True
                rospy.loginfo("Initial pose received.")

    def franka_state_callback(self, msg):
        print(msg.O_T_EE)
    
    def publish_circular_motion(self):
        rate = rospy.Rate(10)  # 10 Hz
        elapsed_time = 0.0
        period = 0.08  # seconds

        while not rospy.is_shutdown():
            if not self.initial_pose_received:
                rospy.loginfo("Waiting for initial pose...")
                rate.sleep()
                continue



            radius = 0.1
            angle = math.pi / 4 * (1 - math.cos(math.pi / 5.0 * elapsed_time))
            delta_x = radius * math.sin(angle)
            delta_z = radius * (math.cos(angle) - 1)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.initial_pose.header.frame_id

            pose_msg.pose.position.x = self.initial_pose.pose.position.x + delta_x
            pose_msg.pose.position.y = self.initial_pose.pose.position.y
            pose_msg.pose.position.z = self.initial_pose.pose.position.z + delta_z

            print()
            quaternion = tf.transformations.quaternion_from_euler(self.initial_pose.pose.orientation.x,self.initial_pose.pose.orientation.y, self.initial_pose.pose.orientation.z, axes='sxyz')
            #print(quaternion)
            #print(pose_msg.pose.orientation)
            pose_msg.pose.orientation.x = self.initial_pose_tf.pose.orientation.x
            pose_msg.pose.orientation.y = self.initial_pose_tf.pose.orientation.y
            pose_msg.pose.orientation.z = self.initial_pose_tf.pose.orientation.z
            pose_msg.pose.orientation.w = self.initial_pose_tf.pose.orientation.w
            print(pose_msg)
            print("---------------------------- init: ")
            print(self.initial_pose)
            print("----------tf")
            print(self.initial_pose_tf)

            self.pub.publish(pose_msg)
            rospy.loginfo("Publishing updated pose with circular motion.")

            elapsed_time += period
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('circular_motion_publisher')
    circular_motion_publisher = CircularMotionPublisher()
    try:
        circular_motion_publisher.publish_circular_motion()
    except rospy.ROSInterruptException:
        pass
