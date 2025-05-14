#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from ff_msgs.msg import MotionActionGoal, MotionActionResult, ArmActionGoal, ArmActionResult



# This function takes a grasp pose and converts it to coordinates understood by the body's frame.
# The new coordinates are necessary in order to use the existing path planner algorithm.
def update_coordinates(msg, boolean=False):
    # offset from the tf link "body" to the middle of the gripper
    offset = np.array([0.315, 0.0, 0.180])
    
    # orientation of the incoming grasp pose (world coordinates in the form (x, y, z, qx, qy, qz, qw))
    grasp_pose_rotation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    
    # 180 degree rotation around Astrobee's z-axis
    z_rotation = tf.transformations.quaternion_from_euler(0, 0, np.pi)
    
    # combine rotations: rotate the grasp pose by 180 degrees around the z-axis
    new_astrobee_orientation = tf.transformations.quaternion_multiply(grasp_pose_rotation, z_rotation)
    
    # rotation matrix representing how the robot is oriented in the world
    rotation_matrix = tf.transformations.quaternion_matrix(new_astrobee_orientation)[:3, :3]
    
    # rotates the offset vector according to the robot's orientation
    global_offset = np.dot(rotation_matrix, offset)
    
    # when boolean is True, the Astrobee will move a safe distance away from the object to avoid 
    # colliding with it while deploying the arm
    if boolean == True:
        safety_offset = np.dot(rotation_matrix, np.array([0.2, 0, 0]))    # 0.2 m safety offset

        extended_offset = global_offset + safety_offset

        target_position = np.array([msg.position.x, msg.position.y, msg.position.z])

        new_position = target_position + extended_offset
    else:
        new_position = np.array([msg.position.x, msg.position.y, msg.position.z]) + global_offset
    

    new_pose = Pose()
    new_pose.position.x = new_position[0]
    new_pose.position.y = new_position[1]
    new_pose.position.z = new_position[2]
    new_pose.orientation.x = new_astrobee_orientation[0]
    new_pose.orientation.y = new_astrobee_orientation[1]
    new_pose.orientation.z = new_astrobee_orientation[2]
    new_pose.orientation.w = new_astrobee_orientation[3]
    
    return new_pose


# This function takes a command and a percent value (0-100) and sends the command to the arm action server.
def control_arm(pub_arm, command, percent=0):
    arm_command = ArmActionGoal()

    # Deploy arm 
    if command == "deploy":
        arm_command.goal.command = 1

    # Stow arm
    elif command == "stow":
        arm_command.goal.command = 2
    
    # Open gripper
    elif command == "open":
        arm_command.goal.command = 7

    # Set gripper
    elif command == "set":
        arm_command.goal.command = 6
        arm_command.goal.gripper = percent

    # Close gripper
    elif command == "close":
        arm_command.goal.command = 8
    
    else:
        rospy.logerr("control_arm: Unknown command: %s", command)
        return

    arm_command.header.stamp = rospy.Time.now()
    pub_arm.publish(arm_command)


# This function takes a corrected pose and sends it to the motion action server.
def move_to_pose(pub, corrected_pose):
    goal_pose = MotionActionGoal()

    goal_pose.header.stamp = rospy.Time.now()

    goal_pose.goal.command = 3
    goal_pose.goal.flight_mode = "nominal"

    state = PoseStamped()
    state.header.stamp = rospy.Time.now()
    state.header.frame_id = 'world'

    state.pose = corrected_pose
    goal_pose.goal.states.append(state)

    pub.publish(goal_pose)


# This function takes a grasp pose as input, then runs the anomaly resolution pipeline
def callback(msg, args):
    rospy.loginfo("Starting anomaly resolution sequence")
    pub_motion, pub_arm = args


    rospy.loginfo("Move to offset pose") 
    offset_pose = update_coordinates(msg, True)
    move_to_pose(pub_motion, offset_pose)
    rospy.wait_for_message('/mob/motion/result', MotionActionResult)


    rospy.loginfo("Deploy arm")
    control_arm(pub_arm, "deploy")
    rospy.wait_for_message('/beh/arm/result', ArmActionResult)


    rospy.loginfo("Open gripper")
    control_arm(pub_arm, "open")
    rospy.wait_for_message('/beh/arm/result', ArmActionResult)


    rospy.loginfo("Move into grasp position")
    grasp_pose = update_coordinates(msg, False)
    move_to_pose(pub_motion, grasp_pose)
    rospy.wait_for_message('/mob/motion/result', MotionActionResult)


    rospy.loginfo("Close gripper")
    control_arm(pub_arm, "close")
    rospy.wait_for_message('/beh/arm/result', ArmActionResult)


    rospy.loginfo("Transporting object to a predefined safe area")
    safe_area = Pose()
    safe_area.position.x = 11.0
    safe_area.position.y = -7.0
    safe_area.position.z = 4.3
    safe_area.orientation.x = 0.0
    safe_area.orientation.y = 0.0
    safe_area.orientation.z = 1.0
    safe_area.orientation.w = 0.0
    move_to_pose(pub_motion, safe_area)
    rospy.wait_for_message('/mob/motion/result', MotionActionResult)
    rospy.loginfo("Anomaly resolution complete!")


if __name__ == "__main__":
    rospy.init_node("anomaly_resolution_node")

    pub_motion = rospy.Publisher('/mob/motion/goal', MotionActionGoal, queue_size=10)

    pub_arm = rospy.Publisher('/beh/arm/goal', ArmActionGoal, queue_size=10)

    rospy.Subscriber("/grasp_pose", Pose, callback, callback_args=(pub_motion, pub_arm))

    rospy.spin()  