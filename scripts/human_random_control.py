#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.stats import truncnorm
from std_msgs.msg import Float64
import random
import numpy as np

# Set fixed seeds for reproducibility
random.seed(0)
np.random.seed(0)

# Function to get random values from a truncated normal distribution
def get_truncated_normal(mean, std_dev, low, high):
    return truncnorm((low - mean) / std_dev, (high - mean) / std_dev, loc=mean, scale=std_dev)

# Joint limits for torso, left, and right arms
torso_joints = {
    "jL5S1_rotx": (-0.2, 0.2),   
    "jL5S1_roty": (0.0, 0.3),
    "jL4L3_rotx": (-0.1, 0.1),
    "jL4L3_roty": (0.0, 0.2),
    "jL1T12_rotx": (-0.2, 0.2), 
    "jL1T12_roty": (0.0, 0.2),
    "jT9T8_rotx": (-0.2, 0.2),
    "jT9T8_roty": (0.0, 0.2),
    "jT9T8_rotz": (-0.2, 0.2),
}

left_arm_joints = {
    "jLeftC7Shoulder_rotx": (-0.087, 0),  
    "jLeftShoulder_rotx": (-0.3, 0),
    "jLeftShoulder_roty": (-1.5, 0.3),
    "jLeftShoulder_rotz": (-1.3, -0.5),
    "jLeftElbow_roty": (-1.7, 0.3),       
    "jLeftElbow_rotz": (-1.2, -0.5),
    "jLeftWrist_rotx": (-0.5, 0.5),
    "jLeftWrist_rotz": (-0.4, 0.0),
}

right_arm_joints = {
    "jRightC7Shoulder_rotx": (0.0, 0.087), 
    "jRightShoulder_rotx": (-0.05, 1.15),
    "jRightShoulder_roty": (-1.3, -0.1),
    "jRightShoulder_rotz": (0.4, 1.7),
    "jRightElbow_roty": (-0.3, 1.1),     
    "jRightElbow_rotz": (1.2, 2.0),
    "jRightWrist_rotx": (-0.5, 0.5),
    "jRightWrist_rotz": (-0.5, 0.3),
}

# Pelvis prismatic and Z-rotation joint limits
pelvis_joints = {
    "world_to_pelvis_x": (-50.0, 50.0),
    "pelvis_x_to_pelvis_y": (-50.0, 50.0),
    "pelvis_z_rotation": (-1.57, 1.57)  # Z-axis rotation between -pi/2 and pi/2
}

# Function to update joint values based on the reset flag
def update_joint_values(joints, reset):
    updated_joints = {}
    for joint, (min_val, max_val) in joints.items():
        if reset:
            updated_joints[joint] = 0.0  # Reset all joints to 0
        else:
            mean = (min_val + max_val) / 2
            std_dev = (max_val - min_val) / 4
            dist = get_truncated_normal(mean, std_dev, min_val, max_val)
            updated_joints[joint] = round(dist.rvs(random_state=0), 2)  # Random value within the range
    return updated_joints

# Function to publish joint trajectories to a ROS topic
def publish_joint_trajectory(joint_set, topic):
    pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Check the reset flag from ROS parameters
        reset_flag = rospy.get_param('/reset_flag', False)

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()

        # Update joint values, reset to zero if the reset_flag is True
        updated_joints = update_joint_values(joint_set, reset_flag)

        trajectory_msg.joint_names = list(updated_joints.keys())
        point = JointTrajectoryPoint()
        point.positions = list(updated_joints.values())
        point.time_from_start = rospy.Duration(1.0)
        trajectory_msg.points = [point]

        pub.publish(trajectory_msg)
        rospy.loginfo(f"Published joint trajectory to {topic}: {updated_joints}")

        rate.sleep()

# Function to publish commands to prismatic joints (pelvis)
def publish_prismatic_joints(pelvis_joints):
    pub_x = rospy.Publisher('/human/human_x_position_controller/command', Float64, queue_size=10)
    pub_y = rospy.Publisher('/human/human_y_position_controller/command', Float64, queue_size=10)
    pub_z_rot = rospy.Publisher('/human/human_z_rotation_controller/command', Float64, queue_size=10) 
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Check the reset flag from ROS parameters
        reset_flag = rospy.get_param('/reset_flag', False)

        # Update joint values, reset to zero if the reset_flag is True
        updated_joints = update_joint_values(pelvis_joints, reset_flag)

        # Publish the updated joint values to the prismatic joint controllers
        pub_x.publish(updated_joints["world_to_pelvis_x"])
        pub_y.publish(updated_joints["pelvis_x_to_pelvis_y"])
        pub_z_rot.publish(updated_joints["pelvis_z_rotation"])  # Publish Z-rotation

        rospy.loginfo(f"Published prismatic joint values: X={updated_joints['world_to_pelvis_x']}, Y={updated_joints['pelvis_x_to_pelvis_y']}, Z_rot={updated_joints['pelvis_z_rotation']}")

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_trajectory_publisher', anonymous=True)

        # Initialize reset_flag parameter to False
        rospy.set_param('/reset_flag', False)

        # Timers to publish joint trajectories
        rospy.Timer(rospy.Duration(1.0), lambda event: publish_joint_trajectory(torso_joints, '/human/upper_torso_controller/command'))
        rospy.Timer(rospy.Duration(1.0), lambda event: publish_joint_trajectory(left_arm_joints, '/human/left_arm_controller/command'))
        rospy.Timer(rospy.Duration(1.0), lambda event: publish_joint_trajectory(right_arm_joints, '/human/right_arm_controller/command'))

        # Timer to publish pelvis prismatic and Z-rotation joint trajectories
        rospy.Timer(rospy.Duration(1.0), lambda event: publish_prismatic_joints(pelvis_joints))

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
