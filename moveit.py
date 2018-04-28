#!/usr/bin/env python


import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import ik
import moveit_msgs.msg
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

        display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
        waypoints = []
        tmp = np.load("valid_pose.npy")
        for i in tmp:
             waypoints.append(i.pose)
                

       
        print "============ Generating plan 1"
        pose_target = Pose()
        pose_target = waypoints[0]

        right_arm.set_pose_target(pose_target)
        right_arm.go(wait=True)

        (plan, fraction) = right_arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses
                                    0.01,        # eef_step
                                    0.0        # jump_threshold
                                     )        # avoid_collisions
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory);
        rospy.sleep(2)
        print "fraction is %s" %fraction

        rospy.on_shutdown(my_hook)
        #raw_input()
        
        right_arm.execute(plan)
                        

        
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)
def my_hook():
    print "shut down nodes"
if __name__ == "__main__":
    MoveItDemo()

