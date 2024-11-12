import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from compar import poses

def create_transformation_matrix(x, y, z, qx, qy, qz, qw):
    """将位置信息和四元数转换为4x4变换矩阵"""
    # 使用Scipy将四元数转换为3x3旋转矩阵
    rotation_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()
    
    # 创建4x4的变换矩阵
    transformation_matrix = np.eye(4)  # 初始化为单位矩阵
    transformation_matrix[:3, :3] = rotation_matrix  # 填充旋转部分
    transformation_matrix[:3, 3] = [x/1000, y/1000, -z/1000]        # 填充平移部分
    
    return transformation_matrix

# 读取txt文件并逐行转换
file_path = '/home/mingtao/THU/SLAM_DATA/Loop2/pose_data_sd.txt'
matrices = []

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

# 绘制3D轨迹和小坐标系
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

max_range = np.array([poses[:, 0].max() - poses[:, 0].min(),
                      poses[:, 1].max() - poses[:, 1].min(),
                      poses[:, 2].max() - poses[:, 2].min()]).max() / 2.0

mid_x = (poses[:, 0].max() + poses[:, 0].min()) * 0.5
mid_y = (poses[:, 1].max() + poses[:, 1].min()) * 0.5
mid_z = (poses[:, 2].max() + poses[:, 2].min()) * 0.5

# 设置坐标轴范围
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# 遍历每20帧进行小坐标系绘制
for i, matrix in enumerate(transformation_matrices):
    if i % 20 == 0:
        # 提取旋转矩阵和位移向量
        R_mat = matrix[:3, :3]
        t_vec = matrix[:3, 3]
        
        # 绘制小坐标系的三个轴
        scale = 0.1  # 缩放因子，控制小坐标系的长度
        origin = t_vec

        # x轴：红色
        ax.quiver(origin[0], origin[1], origin[2], R_mat[0, 0], R_mat[1, 0], R_mat[2, 0], color='r', length=scale)
        # y轴：绿色
        ax.quiver(origin[0], origin[1], origin[2], R_mat[0, 1], R_mat[1, 1], R_mat[2, 1], color='g', length=scale)
        # z轴：蓝色
        ax.quiver(origin[0], origin[1], origin[2], R_mat[0, 2], R_mat[1, 2], R_mat[2, 2], color='b', length=scale)

# 设置图形标题和坐标标签
ax.set_title('3D Trajectory with Local Coordinate Systems')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
