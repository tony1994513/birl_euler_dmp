import numpy as np
import matplotlib.pyplot as plt

import pydmps
import pydmps.dmp_discrete
import euler_quat_transform

from  visualization_msgs.msg import Marker 
import rospy
from geometry_msgs.msg import Pose
import ipdb
def dmp_traj():
    y_des = np.load('pre_pick_to_pre_place.npy')
    euler_data = euler_quat_transform.quat_to_euler(y_des)
    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=6, n_bfs=500, ay=np.ones(6)*10.0)
    y_track = []
    dy_track = []
    ddy_track = []

    dmp.imitate_path(y_des=euler_data.T)
    dmp.y0 = euler_data[0]
    # ipdb.set_trace()
    # dmp.goal = [ 0.83072889, -0.01540935,  0.09491312, -3.12276843, -0.0671215, 1.04747556]
    y_track, dy_track, ddy_track = dmp.rollout()
    quat_data = euler_quat_transform.euler_to_quat(y_track)
    return quat_data

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
    
    
if __name__ == "__main__":
    rospy.init_node("dmp_traj", anonymous=True)    
    data = dmp_traj()
    np.save('dmp_traj', data)
    # plot_cmd_matrix(data)