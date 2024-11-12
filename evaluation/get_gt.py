import numpy as np
from scipy.spatial.transform import Rotation as R

def create_transformation_matrix(x, y, z, qx, qy, qz, qw):
    """将位置信息和四元数转换为4x4变换矩阵"""
    # 使用Scipy将四元数转换为3x3旋转矩阵
    rotation_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()
    
    # 创建4x4的变换矩阵
    transformation_matrix = np.eye(4)  # 初始化为单位矩阵
    transformation_matrix[:3, :3] = rotation_matrix  # 填充旋转部分
    transformation_matrix[:3, 3] = [x/1000, y/1000, -z/1000]        # 填充平移部分
    
    return transformation_matrix

R_x_180 = np.array([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

R_z_180 = np.array([
    [-1, 0, 0, 0],
    [0,  -1, 0, 0],
    [0,  0, 1, 0],
    [0,  0, 0, 1]
])

R_y_180 = np.array([[-1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])



# 读取txt文件并逐行转换
file_path = '/home/mingtao/THU/SLAM_DATA/Loop2/pose_data_sd.txt'
transformation_matrices = []
with open(file_path, 'r') as file:
    i = 0
    for line in file:
        data = list(map(float, line.split()))
        x, y, z = data[0], data[1], data[2]
        qx, qy, qz, qw = data[3], data[4], data[5], data[6]
        
        # 创建4x4的变换矩阵
        if i == 0:
            transformation_matrix = np.identity(4)
            trans_first = create_transformation_matrix(x, y, z, qx, qy, qz, qw)
        else:
            transformation_matrix = create_transformation_matrix(x, y, z, qx, qy, qz, qw)
            transformation_matrix = R_z_180 @ R_x_180 @ R_y_180 @ np.linalg.inv(trans_first) @ transformation_matrix
            transformation_matrix = transformation_matrix
        transformation_matrices.append(transformation_matrix)

        i += 1

print(transformation_matrices[0])
