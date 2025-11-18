import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import os

# ==================== 配置 ====================

transformed_flag = False

# 读取 CSV 文件
file_name = "20250829_161326.csv"
# file_name = "20250929_152433.csv"
CSV_FILE = os.path.join(os.path.dirname(__file__), '..', 'data', f"{file_name}")


def load_data_from_csv(csv_file):
    poses = {'head_pose': [], 'left_arm_pose': [], 'right_arm_pose': []}

    # 读取 CSV 文件
    with open(csv_file, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # 跳过表头

        for row in reader:
            # 解析每列数据为 4x4 矩阵
            head_pose = np.array(eval(row[0]))  # 解析矩阵
            left_arm_pose = np.array(eval(row[1]))  # 解析矩阵
            right_arm_pose = np.array(eval(row[2]))  # 解析矩阵

            poses['head_pose'].append(head_pose)
            poses['left_arm_pose'].append(left_arm_pose)
            poses['right_arm_pose'].append(right_arm_pose)

    return poses

def pose_wrapper(poses):
    # 定义从 OpenXR 到 Robot 的变换矩阵
    T_robot_openxr = np.array([[0, 0, -1, 0],
                               [-1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 0, 1]])

    # 计算 T_robot_openxr 的逆矩阵
    T_robot_openxr_inv = np.linalg.inv(T_robot_openxr)

    # 用于存储转换后的姿态
    transformed_poses = {'head_pose': [], 'left_arm_pose': [], 'right_arm_pose': []}

    # 对每个姿态矩阵集合进行相似变换
    for pose_type in poses:
        for pose in poses[pose_type]:
            # 对每个姿态矩阵应用相似变换
            transformed_pose = T_robot_openxr @ pose @ T_robot_openxr_inv
            transformed_poses[pose_type].append(transformed_pose)

    return transformed_poses


# 加载数据
poses = load_data_from_csv(CSV_FILE)
transformed_poses = pose_wrapper(poses)


# 可视化函数 (动画)
def animate_pose(i, poses, ax, head_x, head_y, head_z, left_x, left_y, left_z, right_x, right_y, right_z):
    # 获取当前的位姿矩阵
    head_pose = poses['head_pose'][i]
    left_arm_pose = poses['left_arm_pose'][i]
    right_arm_pose = poses['right_arm_pose'][i]

    # 提取旋转矩阵 (前 3x3) 和平移向量 (最后一列)
    head_translation = head_pose[:3, 3]
    left_translation = left_arm_pose[:3, 3]
    right_translation = right_arm_pose[:3, 3]

    head_rotation = head_pose[:3, :3]
    left_rotation = left_arm_pose[:3, :3]
    right_rotation = right_arm_pose[:3, :3]

    axis_length = 0.5  # 坐标轴长度

    def draw_axis(line, translation, rotation_vec):
        direction = rotation_vec / np.linalg.norm(rotation_vec) * axis_length
        line.set_data([translation[0], translation[0] + direction[0]],
                      [translation[1], translation[1] + direction[1]])
        line.set_3d_properties([translation[2], translation[2] + direction[2]])

    # 绘制头部坐标系
    draw_axis(head_x, head_translation, head_rotation[:, 0])
    draw_axis(head_y, head_translation, head_rotation[:, 1])
    draw_axis(head_z, head_translation, head_rotation[:, 2])

    # 绘制左臂坐标系
    draw_axis(left_x, left_translation, left_rotation[:, 0])
    draw_axis(left_y, left_translation, left_rotation[:, 1])
    draw_axis(left_z, left_translation, left_rotation[:, 2])

    # 绘制右臂坐标系
    draw_axis(right_x, right_translation, right_rotation[:, 0])
    draw_axis(right_y, right_translation, right_rotation[:, 1])
    draw_axis(right_z, right_translation, right_rotation[:, 2])

    return head_x, head_y, head_z, left_x, left_y, left_z, right_x, right_y, right_z



# 绘制初始图形
def visualize_pose(poses):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 设置坐标轴范围
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    # 设置轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_title('3D Pose Animation')

    # 初始化坐标系的三个轴
    head_x, = ax.plot([], [], [], color='r', label='Head X')
    head_y, = ax.plot([], [], [], color='g', label='Head Y')
    head_z, = ax.plot([], [], [], color='b', label='Head Z')

    left_x, = ax.plot([], [], [], color='c', label='Left Arm X')
    left_y, = ax.plot([], [], [], color='m', label='Left Arm Y')
    left_z, = ax.plot([], [], [], color='y', label='Left Arm Z')

    right_x, = ax.plot([], [], [], color='orange', label='Right Arm X')
    right_y, = ax.plot([], [], [], color='purple', label='Right Arm Y')
    right_z, = ax.plot([], [], [], color='pink', label='Right Arm Z')

    # 创建动画
    ani = FuncAnimation(fig, animate_pose, frames=len(poses['head_pose']), fargs=(poses, ax, head_x, head_y, head_z, left_x, left_y, left_z, right_x, right_y, right_z), interval=50, blit=True)

    # 显示图例
    ax.legend()

    plt.show()


# 可视化数据（动画）
if transformed_flag:
    visualize_pose(transformed_poses)
else:
    visualize_pose(poses)
