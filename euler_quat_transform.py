import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler,unit_vector

def quat_to_euler(data):
    '''
    data: numpy matrix
    '''
    quat_list = data[:,3:7]
    p_list = data[:,0:3]
    euler_list = []
    for rows in quat_list:
        rows =  unit_vector(rows)
        tmp = euler_from_quaternion(rows)
        euler_list.append(tmp)
    stack = np.hstack((p_list,euler_list))
    return stack

def euler_to_quat(data):
    euler_list = data[:,3:6]
    p_list = data[:,0:3]
    quat_list = []
    for rows in euler_list: 
        tmp = quaternion_from_euler(rows[0],rows[1],rows[2])
        tmp = unit_vector(tmp)
        quat_list.append(tmp)
    stack = np.hstack((p_list,quat_list))
    return stack

def main():
    data =np.load("home_to_pre_pick.npy")
    euler_data = quat_to_euler(data)
    # print euler_data
    quat_data = euler_to_quat(euler_data)
    print quat_data

if __name__ == "__main__":
    main()
