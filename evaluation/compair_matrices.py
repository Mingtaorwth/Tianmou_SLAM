import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def rotation_error(gt_matrix, slam_matrix):
    """
    计算GT矩阵与SLAM矩阵在x, y, z轴上的旋转误差。
    参数:
        gt_matrix: GT的4x4变换矩阵
        slam_matrix: SLAM的4x4变换矩阵
    返回:
        rot_errors: 沿x, y, z轴的旋转误差（单位：度）
    """
    # 提取旋转矩阵
    R_gt = gt_matrix[:3, :3]
    R_slam = slam_matrix[:3, :3]

    # 计算GT和SLAM的旋转差
    R_diff = R_gt.T @ R_slam

    # 将旋转差转换为欧拉角（使用scipy）
    r = R.from_matrix(R_diff)
    rot_errors = np.abs(r.as_euler('xyz', degrees=True))  # 转换为x, y, z轴上的角度误差

    return rot_errors

def translation_error(gt_matrix, slam_matrix):
    """
    计算GT矩阵与SLAM矩阵的欧几里德距离误差。
    参数:
        gt_matrix: GT的4x4变换矩阵
        slam_matrix: SLAM的4x4变换矩阵
    返回:
        distance_error: 欧几里德距离误差
    """
    # 提取平移向量
    t_gt = gt_matrix[:3, 3]
    t_slam = slam_matrix[:3, 3]

    # 计算欧几里德距离误差
    distance_error = np.linalg.norm(t_gt - t_slam)

    return distance_error

def evaluate_matrices(gt_matrices, slam_matrices):
    """
    评估GT与SLAM之间的旋转误差和欧几里德距离误差。
    参数:
        gt_matrices: GT的变换矩阵列表
        slam_matrices: SLAM的变换矩阵列表
    返回:
        rotation_errors: 每帧的旋转误差
        translation_errors: 每帧的欧几里德距离误差
    """
    assert len(gt_matrices) == len(slam_matrices), "GT和SLAM矩阵数量必须相同"

    rotation_errors = []
    translation_errors = []

    for gt_matrix, slam_matrix in zip(gt_matrices, slam_matrices):
        # 计算旋转误差
        rot_error = rotation_error(gt_matrix, slam_matrix)
        rotation_errors.append(rot_error)

        # 计算欧几里德距离误差
        trans_error = translation_error(gt_matrix, slam_matrix)
        translation_errors.append(trans_error)

    return np.array(rotation_errors), np.array(translation_errors)


from GT_viz import transformation_matrices
from predict_viz import matrices

rotation_errors, translation_errors = evaluate_matrices(transformation_matrices, matrices)

def plot_errors(rotation_errors, translation_errors):
    """
    绘制旋转误差和欧几里德距离误差的折线图
    参数:
        rotation_errors: 旋转误差 (shape: [n_frames, 3])
        translation_errors: 欧几里德距离误差 (shape: [n_frames])
    """
    num_frames = len(rotation_errors)
    frames = np.arange(num_frames)

    # 创建子图
    fig, axs = plt.subplots(4, 1, figsize=(10, 15))
    fig.suptitle('Rotation and Translation Errors per Frame')

    # 绘制x轴旋转误差
    axs[0].plot(frames, rotation_errors[:, 0], label=f'Rotation Error mean_X={np.mean(rotation_errors[:, 0]):.3f}', color='r')
    axs[0].set_ylabel('Rotation Error X (degrees)')
    axs[0].set_xlabel('Frame')
    axs[0].grid(True)

    # 绘制y轴旋转误差
    axs[1].plot(frames, rotation_errors[:, 1], label=f'Rotation Error mean_Y={np.mean(rotation_errors[:, 1]):.3f}', color='g')
    axs[1].set_ylabel('Rotation Error Y (degrees)')
    axs[1].set_xlabel('Frame')
    axs[1].grid(True)

    # 绘制z轴旋转误差
    axs[2].plot(frames, rotation_errors[:, 2], label=f'Rotation Error mean_Z={np.mean(rotation_errors[:, 1]):.3f}', color='b')
    axs[2].set_ylabel('Rotation Error Z (degrees)')
    axs[2].set_xlabel('Frame')
    axs[2].grid(True)

    # 绘制欧几里德距离误差
    axs[3].plot(frames, translation_errors, label=f'Translation Error={np.mean(translation_errors):.3f}', color='purple')
    axs[3].set_ylabel('Translation Error (meters)')
    axs[3].set_xlabel('Frame')
    axs[3].grid(True)

    # 显示图例
    for ax in axs:
        ax.legend()

    # 显示图形
    plt.tight_layout()
    plt.show()

# 使用示例
# rotation_errors = np.array([[x1_error, y1_error, z1_error], [x2_error, y2_error, z2_error], ...])
# translation_errors = np.array([dist1_error, dist2_error, ...])

# 假设我们已经从evaluate_matrices函数得到了rotation_errors和translation_errors
rotation_errors, translation_errors = evaluate_matrices(transformation_matrices, matrices)

# 调用绘图函数
plot_errors(rotation_errors, translation_errors)