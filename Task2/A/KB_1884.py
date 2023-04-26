#! /usr/bin/env python3

import rospy
import sys
import copy
import traceback
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import actionlib
import math

class Ur5Moveit_arm:

    # Constructor
    def __init__(self):

        rospy.init_node('arm_control', anonymous=True)

        self._planning_group = "arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "roll: {}\n".format(roll) +
                      "pitch: {}\n".format(pitch) +
                      "yaw: {}\n".format(yaw) +
                      '\033[0m')

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
           goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
    
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

class Ur5Moveit_gripper:

    # Constructor
    def __init__(self):

        rospy.init_node('arm_control', anonymous=True)

        self._planning_group = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
           goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    
    
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():

    ur5_arm = Ur5Moveit_arm()
    ur5_gripper = Ur5Moveit_gripper()

    # yellow_pick = [math.radians(94.5),
    #                     math.radians(60),
    #                     math.radians(-48),
    #                     math.radians(-3),
    #                     math.radians(0),
    #                     math.radians(-90)]

    yellow_pick = [math.radians(94),
                        math.radians(60),
                        math.radians(-48),
                        math.radians(-3),
                        math.radians(0),
                        math.radians(0)]

    red_pick = [math.radians(73),
                          math.radians(60),
                          math.radians(-90),
                          math.radians(0),
                          math.radians(0),
                          math.radians(90)]

    yellow_drop = [math.radians(20),
                          math.radians(15),
                          math.radians(-50),
                          math.radians(35),
                          math.radians(0),
                          math.radians(0)]

    # red_drop = [math.radians(-35),
    #                       math.radians(0),
    #                       math.radians(0),
    #                       math.radians(0),
    #                       math.radians(0),
    #                       math.radians(-15)]
    
    intermediate_loc_1 = [math.radians(94),
                          math.radians(45),
                          math.radians(-30),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
    
    intermediate_loc_2 = [math.radians(0),
                          math.radians(60),
                          math.radians(-90),
                          math.radians(0),
                          math.radians(0),
                          math.radians(90)]

    red_drop = [math.radians(-35),
                          math.radians(15),
                          math.radians(-60),
                          math.radians(45),
                          math.radians(0),
                          math.radians(0)]                      
    
    ur5_arm.go_to_predefined_pose("rest")
    print("===== Going to Intermediate Position 1 =====")
    ur5_arm.set_joint_angles(intermediate_loc_1)
    print("===== Going to Pick Yellow =====")
    ur5_arm.set_joint_angles(yellow_pick)
    ur5_gripper.go_to_predefined_pose("close")
    print("===== Going to Intermediate Position 1 =====")
    ur5_arm.set_joint_angles(intermediate_loc_1)
    print("===== Going to Drop Yellow =====")
    ur5_arm.set_joint_angles(yellow_drop)
    ur5_gripper.go_to_predefined_pose("open")
    print("===== Going to Pick Red =====")
    ur5_arm.set_joint_angles(red_pick)
    ur5_gripper.go_to_predefined_pose("close")
    print("===== Going to Drop Red =====")
    ur5_arm.set_joint_angles(red_drop)
    ur5_gripper.go_to_predefined_pose("open")
      
    del ur5_arm
    del ur5_gripper

    rospy.signal_shutdown("===== Task Completed =====")  


if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()
        
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")