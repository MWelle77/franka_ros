import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import math
import time

def get_current_pose():
    pose_msg = rospy.wait_for_message("/panda/kth_cartesian_pose_effort_interface_controller/panda_x", PoseStamped)
    return pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z

def calculate_trajectory(xc, yc, z, radius, total_time, omega, phi,pub_rate):
    timestep = pub_rate
    poses = []
    for t in range(int(total_time / timestep)):
        x = xc #+ radius * math.cos(omega * t * timestep + phi)
        y = yc #+ radius * math.sin(omega * t * timestep + phi)
        poses.append((x, y, z))
    return poses

def to_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='szyx')
    #quaternion[0]*=-1
    #quaternion[3]*=-1
    return quaternion

def get_EE_pose(tf_listener,verbose=False, frame='/panda_link0', get_extended=True):
        """"
        frame is to define the reference frame over which we want to have the EE pose
        """
        
        try:
            # (trans,rot) = self.tf_listener.lookupTransform('/panda_link0', '/panda_EE', rospy.Time(0))
            (trans, rot) = tf_listener.lookupTransform(frame, '/panda_EE', rospy.Time(0))
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

def main():
    rospy.init_node('circular_trajectory_test')
    tf_listener = tf.TransformListener()

    pub_rate=0.01

    # Get the current pose of the robot
    current_pose = get_current_pose()
    xc, yc, zc, roll, pitch, yaw = current_pose

    # Parameters for the circle
    radius = 0.1  # 10 cm radius
    omega = math.pi / 4  # Full circle in 8 seconds
    phi = 0
    total_time = 8.0

    # Calculate the circular trajectory
    poses = calculate_trajectory(xc, yc, zc, radius, total_time, omega, phi,pub_rate)

    # Publisher to send commands to the robot
    pub = rospy.Publisher('/panda/kth_cartesian_pose_effort_interface_controller/panda/equilibrium_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(1/pub_rate)  #  Hz

    while True:
        tf_pos, tf_quat= get_EE_pose(tf_listener)
        if not tf_pos is None:
            break

    for pose in poses:
        
        x, y, z = pose
        quaternion = to_quaternion(roll, pitch, yaw)  # Keeping the orientation constant

        # Create a PoseStamped message
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose = Pose()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publish the new pose
        print(len(poses))
        print(pose_msg)
        print(tf_quat)
        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
