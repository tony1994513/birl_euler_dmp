#!/usr/bin/env python


import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import ik
import numpy as np

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_ik', anonymous=True)
        robot = moveit_commander.RobotCommander()            
        # Connect to the right_arm move group
        right_arm = MoveGroupCommander('right_arm')
        
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)

        # Allow some leeway in position(meters) and orientation (radians)
        # right_arm.set_goal_position_tolerance(0.01)
        # right_arm.set_goal_orientation_tolerance(0.1)
        
        # Get the name of the end-effector link
        # end_effector_link = right_arm.get_end_effector_link()
        
        # Get the current pose so we can add it as a waypoint
        # start_pose = right_arm.get_current_pose(end_effector_link).pose
                
        # Initialize the waypoints list
        waypoints = []
        valid_pose = np.load("valid_pose.npy")
        for i in valid_pose:
             waypoints.append(i.pose)
                
        no_pros_data = np.load("dmp_euler_traj.npy")
        pose_list = []
        for idx in no_pros_data:
            pose = Pose()
            pose.position.x = idx[0]
            pose.position.y = idx[1]
            pose.position.z = idx[2]
            pose.orientation.x= idx[3]
            pose.orientation.y= idx[4]
            pose.orientation.z= idx[5]
            pose.orientation.w= idx[6]
            pose_list.append(pose)
        # Set the internal state to the current state
        right_arm.set_start_state_to_current_state()

        (plan, fraction) = right_arm.compute_cartesian_path (
                                    pose_list,   # waypoint poses
                                    0.01,        # eef_step
                                    0.0        # jump_threshold
                                     )        # avoid_collisions
                                                                                                                                            
        print "fraction is %s" %fraction

        right_arm.execute(plan)
                        

        
        # Shut down MoveIt cleanly                                                                                                                                                                                                                                                                                                                                                                              
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

