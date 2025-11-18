import numpy as np
import matplotlib.pyplot as plt

def plot_frame(ax, T, name, color_prefix=""):
    """在3D坐标系中画出齐次变换矩阵 T 对应的坐标轴"""
    origin = T[:3, 3]
    x_axis = origin + T[:3, 0]
    y_axis = origin + T[:3, 1]
    z_axis = origin + T[:3, 2]

    ax.quiver(*origin, *(x_axis-origin), color=color_prefix+"r", length=1.0, normalize=True)
    ax.quiver(*origin, *(y_axis-origin), color=color_prefix+"g", length=1.0, normalize=True)
    ax.quiver(*origin, *(z_axis-origin), color=color_prefix+"b", length=1.0, normalize=True)
    ax.text(*origin, name, fontsize=10, color="k")

def main():
    # ====== 定义坐标系变换 ======
    # OpenXR: x右, y上, z背
    # Robot : x前, y左, z上
    T_robot_openxr = np.array([
        [ 0, 0,-1, 0],  # Robot x <- -OpenXR z
        [-1, 0, 0, 0],  # Robot y <- -OpenXR x
        [ 0, 1, 0, 0],  # Robot z <- OpenXR y
        [ 0, 0, 0, 1]
    ])

    # ====== 定义一个测试矩阵 VuerMat (OpenXR Convention 下) ======
    theta = np.deg2rad(45)
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])
    VuerMat = np.eye(4)
    VuerMat[:3,:3] = R
    VuerMat[:3,3] = [1, 0, 0]  # 平移到 x=1

    # ====== 相似变换到 Robot Convention ======
    WristMat = T_robot_openxr @ VuerMat @ np.linalg.inv(T_robot_openxr)

    # ====== 可视化 ======
    fig = plt.figure(figsize=(10,5))
    ax1 = fig.add_subplot(121, projection="3d")
    ax2 = fig.add_subplot(122, projection="3d")

    # 左边：OpenXR Convention
    plot_frame(ax1, np.eye(4), "OpenXR Basis")
    plot_frame(ax1, VuerMat, "VuerMat")
    ax1.set_title("OpenXR Convention")
    ax1.set_xlim([-2,2]); ax1.set_ylim([-2,2]); ax1.set_zlim([-2,2])

    # 右边：Robot Convention
    plot_frame(ax2, np.eye(4), "Robot Basis")
    plot_frame(ax2, WristMat, "WristMat")
    ax2.set_title("Robot Convention")
    ax2.set_xlim([-2,2]); ax2.set_ylim([-2,2]); ax2.set_zlim([-2,2])

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
