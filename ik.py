import rospy
from geometry_msgs.msg import PoseStamped
from trac_ik_baxter.srv import *
from sensor_msgs.msg import JointState
from pyquaternion import Quaternion
import numpy as np 


def main():
   
    data = np.load("dmp_euler_traj.npy")
    list_of_pose = make_req(data)
    invalid_index,valid_pose = trak_ik(list_of_pose)
    invalid_traj = []
    for i in invalid_index:
        invalid_traj.append(data[i])

    plot(data,invalid_index)
    np.save("valid_pose",np.array(valid_pose))
            
def trak_ik(list_of_pose):  
    num_len = len(list_of_pose)
    list_of_ik = []  
    ik = rospy.ServiceProxy('trac_ik_right', GetConstrainedPositionIK)
    req = GetConstrainedPositionIKRequest()
    req.pose_stamp.append(list_of_pose[0])
    resp = ik(req)
    if resp.isValid[0] == True:
        list_of_ik.append(resp)
    else:
        rospy.logfatal("No valide solution at 1 point")


    valid_index = 0
    valid_pose = []
    invalid_num = 0
    invalid_index = []
    for idx in range(num_len):
        req = GetConstrainedPositionIKRequest()
        joint_seed = JointState()
        joint_seed.name = list_of_ik[valid_index].joints[0].name
        joint_seed.position = list_of_ik[valid_index].joints[0].position

        req.seed_angles.append(joint_seed)
        req.pose_stamp.append(list_of_pose[idx])
        resp1 = ik(req)
        
        if resp1.isValid[0] == True:
            valid_pose.append(list_of_pose[idx])
            list_of_ik.append(resp1)
            valid_index = idx - invalid_num 
        
        else:
            print "No valid solution at %s point" %idx
            invalid_num +=1
            invalid_index.append(idx)
    rate = len(list_of_ik)/float(num_len)
    print ("success rate %s"  %str(rate))
    return invalid_index,valid_pose
    # print ("invalid index is %s" %str(invalid_index))
    # plot(data,invalid_index)


def make_req(data):
    num_len = data.shape[0]
    list_of_pose = []
    for row in range(num_len):
        pose = PoseStamped()
        pose.pose.position.x = data[row][0]
        pose.pose.position.y = data[row][1]
        pose.pose.position.z = data[row][2]
        pose.pose.orientation.x = data[row][3]
        pose.pose.orientation.y = data[row][4]
        pose.pose.orientation.z = data[row][5]
        pose.pose.orientation.w = data[row][6]
        list_of_pose.append(pose)
    return list_of_pose   

# def interplot_traj(data,invalid_index):

#     interplot_start = data[invalid_index[0]]
#     interplot_end = data[invalid_index[-1]]
#     num_of_invalid = len(invalid_index)

#     new_x = np.linspace(interplot_start[0],interplot_end[0],num_of_invalid)
#     new_y = np.linspace(interplot_start[1],interplot_end[1],num_of_invalid)
#     new_z = np.linspace(interplot_start[2],interplot_end[2],num_of_invalid)
#     new_data = np.array([new_x,new_y,new_z])
#     for i,idx in enumerate(invalid_index):
#         data[idx][0:3] = new_data[:,i]
#     return data

def interpolate_pose_using_slerp(command_matrix):
    pxyz_idx = [0,1,2]
    qxyzw_idx = [3,4,5,6]
    pos_mat = command_matrix[:, pxyz_idx].copy()
    quat_mat = command_matrix[:, qxyzw_idx].copy()


    distance_mat = np.linalg.norm(pos_mat[1:]-pos_mat[:-1], axis=1)
    steps = np.cumsum(distance_mat, axis=0)
    steps /= steps[-1]

    qxyzw0 = np.asarray(quat_mat[0]).reshape(-1)
    qxyzw1 = np.asarray(quat_mat[-1]).reshape(-1)

    if numpy.dot(qxyzw0, qxyzw1) < 0:
        qxyzw0 = -qxyzw0
    
    q0 = Quaternion(
        qxyzw0[3],
        qxyzw0[0],
        qxyzw0[1],
        qxyzw0[2],
    )
    q1 = Quaternion(
        qxyzw1[3],
        qxyzw1[0],
        qxyzw1[1],
        qxyzw1[2],
    )
    interpolated_q = [q0]
    for i in steps:
        interpolated_q.append(Quaternion.slerp(q0, q1, i))
    interpolated_mat = [[
        i.elements[1], 
        i.elements[2], 
        i.elements[3], 
        i.elements[0]] for i in interpolated_q]

    command_matrix[:, qxyzw_idx] = interpolated_mat
    return command_matrix

def plot(data,invalid_index=None):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    data_orig = np.copy(data)
    x = data_orig[:,0]
    y = data_orig[:,1]
    z = data_orig[:,2]
    ax.plot(x, y, z, label='orig_data')

    if invalid_index !=[]:
        invalid_data = []
        for i in invalid_index:     
            invalid_data.append(data[i])
        invalid_data = np.array(invalid_data)
        ax.scatter(invalid_data[:,0], invalid_data[:,1], invalid_data[:,2], c="r", alpha=0.5,label="invalid_points")
        # new_traj =  interplot_traj(data,invalid_index)
        # ax.plot(new_traj[:,0] , new_traj[:,1] , new_traj[:,2] , label='interploted_data')
    ax.legend()
    plt.show()


if __name__ == '__main__':
    rospy.init_node("trac_ik_dmp_node")
    main()
