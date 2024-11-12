import numpy as np
from compar import poses
import matplotlib.pyplot as plt

def create_transformation_matrix_from_pose(data):
    """根据位姿数据创建4x4的变换矩阵"""
    # 提取3x3旋转矩阵
    rotation_matrix = np.array([
        [data[0], data[1], data[2]],
        [data[4], data[5], data[6]],
        [data[8], data[9], data[10]]
    ])
    
    # 提取平移向量
    translation_vector = np.array([data[3], data[11], data[7]])

    # 创建4x4的变换矩阵
    transformation_matrix = np.eye(4)  # 初始化为单位矩阵
    transformation_matrix[:3, :3] = rotation_matrix  # 填充旋转矩阵
    transformation_matrix[:3, 3] = translation_vector  # 填充平移向量

    return transformation_matrix

# 读取txt文件并逐行转换
file_path = '/home/mingtao/ORB_SLAM3_tianmou/CameraTrajectory.txt'
matrices = []  # 用于存储每一帧的变换矩阵
positions = []  # 用于存储平移向量

with open(file_path, 'r') as file:
    for line in file:
        # 将一行数据转换为浮点数列表
        data = list(map(float, line.split()))
        
        # 创建变换矩阵
        transformation_matrix = create_transformation_matrix_from_pose(data)
        matrices.append(transformation_matrix)


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
for i, matrix in enumerate(matrices):
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

