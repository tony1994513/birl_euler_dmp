import numpy as np
import matplotlib.pyplot as plt

import pydmps
import pydmps.dmp_discrete
import euler_quat_transform

from  visualization_msgs.msg import Marker 
import rospy
from geometry_msgs.msg import Pose
import ipdb

y_des = np.load('rotate_90.npy')

def euler_dmp_traj():
    euler_data = euler_quat_transform.quat_to_euler(y_des)
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=6, n_bfs=500, ay=np.ones(6)*10.0)
    y_track = []
    dy_track = []
    ddy_track = []

    dmp.imitate_path(y_des=euler_data.T)
    dmp.y0 = euler_data[0]
    # ipdb.set_trace()
    #dmp.goal = [0.12410378, -0.86553541,  0.15693952, -2.07395909, 0.00913348,-1.65267855]
    # dmp.goal = [0.83027307, -0.01455831,  0.10096982, -3.11037425, 0.12514308,0.04512591]
    y_track, dy_track, ddy_track = dmp.rollout()
    quat_data = euler_quat_transform.euler_to_quat(y_track)
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
    
if __name__ == "__main__":
    rospy.init_node("dmp_traj", anonymous=True) 
    train_dmp = "euler_angl"
    if train_dmp == "euler_angle"  : 
        data = euler_dmp_traj()
        np.save('dmp_euler_traj', data) 
        plot_command_matrix_in_matplotlib(data)
    else:
        data = quat_dmp_traj()
        np.save('dmp_quat_traj', data) 
        plot_command_matrix_in_matplotlib(data)
