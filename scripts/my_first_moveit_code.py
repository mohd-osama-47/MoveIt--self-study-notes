#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String

if __name__ == '__main__':

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('My_MoveIt_Test_Node')
    
    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()


    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## This interface can be used to plan and execute motions:
    group_names = ["panda_arm", "panda_hand", "panda_manipulator"]
    move_groups: list = [moveit_commander.MoveGroupCommander(group_name) for group_name in group_names]

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory, queue_size=5)
    
    # We can get the name of the reference frame for this robot in each move_group:
    planning_frames = [move_group.get_planning_frame() for move_group in move_groups]

    [print("============ Planning frame: %s" % planning_frame) for planning_frame in planning_frames]

    # We can also print the name of the end-effector link for this group:
    eef_links = [move_group.get_end_effector_link() for move_group in move_groups]
    [print("============ End effector link: %s" % eef_link) for eef_link in eef_links]

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
    print("GOING TO A BETTER INITIAL POSITION THROUGH MOVE GROUP JOINT MOVEMENT!")
    print("")

    ## Planning to a Joint Goal
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
    ## thing we want to do is move it to a slightly better configuration.

    # We get the joint values from the group and change some of the values:
    move_group = move_groups[0]
    joint_goal = move_group.get_current_joint_values()  # get panda_arm join states
    joint_goal[0] = 0
    joint_goal[1] = - tau / 8 # (45 degrees)
    joint_goal[2] = 0
    joint_goal[3] = - tau / 4 # (90 degrees)
    joint_goal[4] = 0
    joint_goal[5] =   tau / 6 # (60 degrees)
    joint_goal[6] = 0
    

    # # The go command can be called with joint values, poses, or without any
    # # parameters if you have already set the pose or joint target for the group
    # move_group.go(joint_goal, wait=True)

    # Alternatively, you can use the plan (which calls set_joint_value_target internally) command in move_group:
    success, traj, planning_time, error_code = move_group.plan(joint_goal)   # this also shows trajectory in topic
    input(
            "Go?....."
        )
    if success:
        move_group.execute(traj)
    
    print("TOOK TO PLAN:", planning_time)
    print("ERROR CODE:", error_code)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()


    input("============ Press `Enter` to execute a movement using a pose goal ...")
    ## Planning to a Pose Goal

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    move_group = move_groups[0] # just so that it is easier to change the group
    pose_goal: geometry_msgs.msg.Pose = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0   # no rotations
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    # Either use set_pose_target(pose_goal) + move_group.go(wait=True)
    # OR use the plan (which accepts JointState and Pose types) command as follows:
    success, traj, planning_time, error_code = move_group.plan(pose_goal)
    input(
            "Go?....."
        )
    if success:
        move_group.execute(traj)
    
    print("TOOK TO PLAN:", planning_time)
    print("ERROR CODE:", error_code)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()


    input("============ Press `Enter` to plan and display a Cartesian path ...")
    move_group = move_groups[0] # just so that it is easier to change the group
    ## Cartesian Paths
    waypoints = []

    # First pose
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= 0.1     # moves up a bit
    wpose.position.y += 0.2     # moves to the side a bit
    waypoints.append(copy.deepcopy(wpose))  # deepcopy used since we just want to use wpose as a template essentially for sub-sequent points.

    # Second pose
    wpose.position.x += 0.1     # move in x direction
    waypoints.append(copy.deepcopy(wpose))

    # Third pose
    wpose.position.y -= 0.1     # move in y direction
    waypoints.append(copy.deepcopy(wpose))

    # Using the waypoints list, we will interpolate a cartesian path at 
    # a resolution of 1 cm (0.01 m), which is passed as eef-step 
    # for cartesian translation.
    path_res = 0.01

    # Here. jump threshold is disabled (by passing 0.0), this ignores 
    # infeasible jump checks in joint space!
    plan, fraction = move_group.compute_cartesian_path(waypoints, path_res, 0.0)
    
    # plan is of type moveit_msgs.msg.RobotTrajectory, same result from 
    # the plan command used previously for both joint and pose movement!
    
    # fraction represents "how much of the path was followed of the RobotTrajectory"
    ##! The robot's current joint state must be within some tolerance of the
    ##! first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail!
    input(
            "Go?....."
        )
    move_group.execute(plan, wait=True)




    input("============ Press `Enter` to add a box to the planning scene ...")
    ## Adding Objects to the Planning Scene
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = group_names[1]   # panda-hand
    box_pose.pose.orientation.w = 1.0   # Flat without any rotations
    box_pose.pose.position.z = 0.11     # Place above the panda_hand frame by 1.1 cm
    box_name = "box"
    
    # Check the planning_scene_interface file for other add commands!
    # add a 7.5 cm^3 box 
    scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))



    input("============ Press `Enter` to attach a Box to the Panda robot ...")
    ## Attaching Objects to the Robot
    # Manipulating objects by a robot requires touching them WITHOUT the planning scene reporting collision.
    # This is achieved by adding any robot link of interest into the 'touch_links' 
    # array, effectively telling the planning scene to ignore 
    # any collisions between those links and a collision box/source

    # For the panda robot, the grasping group used is 
    # the group panda_hand with the gripper
    grasping_group = group_names[1]
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_links[0], box_name, touch_links=touch_links)



    input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
    # This will move the robot with the box attached!
    waypoints = []
    # First pose
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= 0.1     # moves up a bit
    wpose.position.y += 0.2     # moves to the side a bit
    waypoints.append(copy.deepcopy(wpose))
    # Second pose
    wpose.position.x += 0.1     # move in x direction
    waypoints.append(copy.deepcopy(wpose))
    # Third pose
    wpose.position.y -= 0.1     # move in y direction
    waypoints.append(copy.deepcopy(wpose))

    plan, fraction = move_group.compute_cartesian_path(waypoints, path_res, 0.0)
    input("Go?.....")
    move_group.execute(plan, wait=True)


    input("============ Press `Enter` to detach the box from the Panda robot ...")
    ## Detaching Objects from the Robot
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_links[0], name=box_name)
    
    
    input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
    ## Removing Objects from the Planning Scene

    ## We can remove the box from the world.
    ## **Note:** The object must be detached before we can remove it from the world
    scene.remove_world_object(box_name)