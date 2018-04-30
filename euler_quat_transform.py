import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler,unit_vector
from math import ceil

def quat_to_euler_point(data):
    '''
    data: a point 
    '''
    quat = data[3:7]
    pos = data[0:3]

    euler = euler_from_quaternion(quat)       
    stack = np.hstack((pos,euler))
    return stack

def quat_to_euler_list(data):
    '''
    data: numpy matrix
    '''
    data = filter_static_points(data)
    quat_list = data[:,3:7]
    p_list = data[:,0:3]
    euler_list = []
    for rows in quat_list:
        #rows = [ round(rows, 4) for elem in rows]
        rows =  unit_vector(rows)
        tmp = euler_from_quaternion(rows)       
        euler_list.append(tmp)
    stack = np.hstack((p_list,euler_list))
    return stack

def euler_to_quat_list(data):
    euler_list = data[:,3:6]
    p_list = data[:,0:3]
    quat_list = []
    for rows in euler_list: 
        tmp = quaternion_from_euler(rows[0],rows[1],rows[2])
        # tmp = [ round(elem, 4) for elem in tmp ]
        tmp = unit_vector(tmp)
        quat_list.append(tmp)
    stack = np.hstack((p_list,quat_list))
    return stack

def filter_static_points(mat):
    last = mat[0]
    new_mat = [last]
    for idx in range(mat.shape[0]):
        if np.linalg.norm(mat[idx]-last) < 0.01:
            pass
        else:
            new_mat.append(mat[idx])
            last = mat[idx] 

    return np.array(new_mat)
    
def main():
    data =np.load("home_to_pre_pick.npy")
    euler_data = quat_to_euler(data)
    # print euler_data
    quat_data = euler_to_quat(euler_data)
    print quat_data

if __name__ == "__main__":
    main()
