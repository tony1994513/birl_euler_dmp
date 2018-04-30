import numpy as np
import matplotlib.pyplot as plt

import pydmps
import pydmps.dmp_discrete
import euler_quat_transform
from ar_track_alvar_msgs.msg import AlvarMarkers
from  visualization_msgs.msg import Marker 
import rospy
from geometry_msgs.msg import Pose
import ipdb
import copy
import pandas as pd
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
)
import moveit_commander
import sys


hover_height = 0.15
y_des = np.load('home_to_pre_pick.npy')

def get_start():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    end_link = group.get_end_effector_link()
    pose = group.get_current_pose().pose
    start_pose = [pose.position.x,pose.position.y,pose.position.z,
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    return start_pose

def plot_euler(euler_data,dmp_data):
    orig_t = np.linspace(0,5,euler_data.shape[0])
    resample_t = np.linspace(0,5,dmp_data.shape[0])
    resample_euler_data = [None,None,None,None,None,None] 
    for i in range(euler_data.shape[1]):
        resample_euler_data[i] = np.interp(resample_t, orig_t, euler_data[:,i])
    df = pd.DataFrame()
    df['pos_x'] = resample_euler_data[0]
    df['pos_y'] = resample_euler_data[1]
    df['pos_z'] = resample_euler_data[2]
    df['euler_x'] = resample_euler_data[3]
    df['euler_y'] = resample_euler_data[4]
    df['euler_z'] = resample_euler_data[5]

    df['dmp_pos_x'] = dmp_data[:,0]
    df['dmp_pos_y'] = dmp_data[:,1]
    df['dmp_pos_z'] = dmp_data[:,2]
    df['dmp_euler_x'] = dmp_data[:,3]
    df['dmp_euler_y'] = dmp_data[:,4]
    df['dmp_euler_z'] = dmp_data[:,5]
    
    fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(13,10))
    df[['pos_x', 'dmp_pos_x']].plot(ax=axes[0], color=['red', 'green'])
    df[['pos_y', 'dmp_pos_y']].plot(ax=axes[1], color=['red', 'green'])
    df[['pos_z', 'dmp_pos_z']].plot(ax=axes[2], color=['red', 'green'])
    df[['euler_x', 'dmp_euler_x']].plot(ax=axes[3], color=['red', 'green'])
    df[['euler_y', 'dmp_euler_y']].plot(ax=axes[4], color=['red', 'green'])
    df[['euler_z', 'dmp_euler_z']].plot(ax=axes[5], color=['red', 'green'])

    plt.show()

def euler_dmp_traj(goal):

    euler_data_orig = euler_quat_transform.quat_to_euler_list(y_des)
    euler_data = copy.deepcopy(euler_data_orig)
    euler_x = []
    for num in euler_data[:,3]:
        if num < 0 :
            num = num + 6.28
        euler_x.append(num)

    euler_data[:,3] = np.array(euler_x)
    # plot_euler_in_matplotlib(euler_data)
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=6, n_bfs=100, ay=np.ones(6)*10.0)
    y_track = []
    dy_track = []
    ddy_track = []

    dmp.imitate_path(y_des=euler_data.T)
    dmp.y0 = euler_data[0]
    quat_start = get_start()
    dmp_start = euler_quat_transform.quat_to_euler_point(quat_start)
    if dmp_start[3] < 0 :
        dmp_start[3] = dmp_start[3] + 6.28
    dmp.y0 = dmp_start
    print "euler start is %s" % dmp_start

    quat_goal = [goal.position.x,goal.position.y,goal.position.z,
    goal.orientation.x, goal.orientation.y,goal.orientation.z,goal.orientation.w]
    dmp_goal = euler_quat_transform.quat_to_euler_point(quat_goal)
    if dmp_goal[3] < 0 :
        dmp_goal[3] = dmp_goal[3] + 6.28
    dmp.goal = dmp_goal
    print "euler goal is %s" % dmp.goal
    y_track, dy_track, ddy_track = dmp.rollout()
    y_track_orig = copy.deepcopy(y_track)
    tmp = []
    for num in y_track[:,3]:
        if num > 3.14 :
            num = num - 6.28
        tmp.append(num)
    y_track[:,3] = np.array(tmp)
    # plot_euler(euler_data_orig,y_track) # orig
    plot_euler(euler_data,y_track_orig) # changed
    # plot_euler_in_matplotlib(y_track)
    quat_data = euler_quat_transform.euler_to_quat_list(y_track)
    # ipdb.set_trace()
    quat_data = filter_static_points(quat_data)
    return np.array(quat_data)

def quat_dmp_traj():
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, ay=np.ones(7)*10.0)
    y_track = []
    dy_track = []
    ddy_track = []
    dmp.imitate_path(y_des=y_des.T)
    dmp.goal = [0.78810801, -0.36576736,  0.09484807, -0.65631082,  0.75341847,-0.0379652 ,  0.01324234]
    #dmp.goal = [0.78810801,-0.36576736,0.09484807, 0.845016179201, -0.531922894576,0.0532656152783, -0.0129794199876]
    y_track, dy_track, ddy_track = dmp.rollout()
    quat_data = filter_static_points(y_track)
    return np.array(quat_data)

def filter_static_points(mat):
    last = mat[0]
    new_mat = [last]
    for idx in range(mat.shape[0]):
        if np.linalg.norm(mat[idx][0:3]-last[0:3]) < 0.005:
            pass
        else:
            new_mat.append(mat[idx])
            last = mat[idx] 
    return np.array(new_mat)

def send_traj_point_marker(marker_pub, pose, id, rgba_tuple):
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "traj_point" 
    marker.id = id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = rgba_tuple[0]
    marker.color.g = rgba_tuple[1]
    marker.color.b = rgba_tuple[2]
    marker.color.a = rgba_tuple[3]
    marker.lifetime = rospy.Duration()
    marker_pub.publish(marker)  

def plot_cmd_matrix(command_matrix):
    import random
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
    rospy.sleep(1)
    rgba_tuple = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0.5, 1), 1]
    for idx,row in enumerate(command_matrix):
        pose = Pose() 
        pose.position.x = row[0]
        pose.position.y = row[1]
        pose.position.z = row[2]
        pose.orientation.x = row[3]
        pose.orientation.y = row[4]
        pose.orientation.z = row[5]
        pose.orientation.w = row[6]

        send_traj_point_marker(marker_pub=marker_pub, pose=pose, id=idx, rgba_tuple=rgba_tuple)
    
def plot_command_matrix_in_matplotlib(command_matrix):
    from mpl_toolkits.mplot3d import axes3d
    import matplotlib.pyplot as plt
    from tf.transformations import (
        quaternion_inverse,
        quaternion_multiply,
    )

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    pxyz_idx = [0,1,2]
    qxyzw_idx = [3,4,5,6]
   
    for i in range(command_matrix.shape[0]):
        qxyzw = command_matrix[i][qxyzw_idx]
        uvw = quaternion_multiply(
            qxyzw,
            quaternion_multiply(
                [1,0,0,0], quaternion_inverse(qxyzw),
            )
        )
        uvw_ = quaternion_multiply(
            qxyzw,
            quaternion_multiply(
                [0,1,0,0], quaternion_inverse(qxyzw),
            )
        )
        uvw__ = quaternion_multiply(
            qxyzw,
            quaternion_multiply(
                [0,0,1,0], quaternion_inverse(qxyzw),
            )
        )
        x, y, z = command_matrix[i][pxyz_idx]
        u, v, w = uvw[:3]
        u_, v_, w_ = uvw_[:3]
        u__, v__, w__ = uvw__[:3]
        ax.quiver(x, y, z, u, v, w, length=0.01,color='red')
        ax.quiver(x, y, z, u_, v_, w_, length=0.01,color='green')
        ax.quiver(x, y, z, u__, v__, w__, length=0.01,color='blue')
    fig.show()
    plt.show()

def plot_euler_in_matplotlib(command_matrix):
    from mpl_toolkits.mplot3d import axes3d
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)
    pxyz_idx = [0,1,2]
    qxyz_idx = [3,4,5]
   
    for i in range(command_matrix.shape[0]):
        qxyz = command_matrix[i][qxyz_idx]
       
        x, y, z = command_matrix[i][pxyz_idx]

        ax.quiver(x, y, z, qxyz[0], 0, 0, length=0.01,color='red')
        ax.quiver(x, y, z, 0, qxyz[1], 0, length=0.01,color='green')
        ax.quiver(x, y, z, 0, 0, qxyz[2], length=0.01,color='blue')
    fig.show()
    plt.show()    
if __name__ == "__main__":
    rospy.init_node("dmp_traj", anonymous=True) 
    msg = rospy.wait_for_message("baxter_available_picking_pose", AlvarMarkers) # listen once
    for marker in msg.markers:
        pos = marker.pose.pose.position
        ori = marker.pose.pose.orientation
        base_to_marker_mat = np.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))

        if rospy.is_shutdown():
            break
        goal = copy.deepcopy(marker.pose.pose)
        goal.position.x -= hover_height*base_to_marker_mat[0, 2]
        goal.position.y -= hover_height*base_to_marker_mat[1, 2]
        goal.position.z -= hover_height*base_to_marker_mat[2, 2]
        train_dmp = "euler_angle"
        if train_dmp == "euler_angle"  : 
            data = euler_dmp_traj(goal)
            np.save('dmp_euler_traj', data) 
            # plot_command_matrix_in_matplotlib(data)
        else:
            data = quat_dmp_traj()
            np.save('dmp_quat_traj', data) 
            plot_command_matrix_in_matplotlib(data)
