import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 导入3D工具
from get_gt import transformation_matrices

pos_GT = []
for mat in transformation_matrices:
   pos_GT.append([mat[0, -1], mat[1, -1], mat[2, -1]])

# 读取txt文件
file_path = '/home/mingtao/ORB_SLAM3_tianmou/CameraTrajectory.txt'
poses = []

with open(file_path, 'r') as file:
    for line in file:
        # 分割行中的数据并转换为浮点数
        data = list(map(float, line.split()))
        # 提取tx, ty, tz (第4, 8, 12个值)
        tx, ty, tz = data[3], data[11], data[7]
        poses.append([tx, ty, tz])

# 将列表转换为numpy数组，方便处理
poses = np.array(poses)
pos_GT = np.array(pos_GT)
# 绘制tx, ty, tz轨迹
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')  # 确保projection='3d'可以使用

ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], label='SD_Trajectory', marker='x', markersize=2)  # 调整图案大小
ax.plot(pos_GT[:, 0], pos_GT[:, 1], pos_GT[:, 2], label='GT', marker='o', markersize=2)  # 调整图案大小
ax.set_xlabel('tx')
ax.set_ylabel('ty')
ax.set_zlabel('tz')
ax.set_title('3D Trajectory of tx, ty, tz')

# 获取坐标轴范围并设置相同尺度
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


errs = np.linalg.norm(poses - pos_GT, axis=1)
frame_n = np.arange(0, len(errs))


plt.legend()
plt.show()

plt.plot(frame_n, errs, c='orange')
plt.title(f"Euclidean Distance mean={np.mean(errs)}")
plt.show()
