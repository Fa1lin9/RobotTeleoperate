import time
import threading
import numpy as np
from televuer import TeleVuer
from multiprocessing import shared_memory

# ==================== 配置 ====================
IMG_SHAPE = (480, 640, 3)  # 图像形状
IMG_SHM_NAME = "demo"       # 共享内存名字

# ==================== 创建共享内存 ====================
try:
    shm = shared_memory.SharedMemory(name=IMG_SHM_NAME)
except FileNotFoundError:
    shm = shared_memory.SharedMemory(create=True, size=np.prod(IMG_SHAPE), name=IMG_SHM_NAME)

# 创建 numpy 数组映射到共享内存
img_array = np.ndarray(IMG_SHAPE, dtype=np.uint8, buffer=shm.buf)
img_array[:] = 0  # 初始化全黑

# ==================== 创建 TeleVuer 实例 ====================
tv = TeleVuer(
    binocular=False,
    use_hand_tracking=True,
    img_shape=IMG_SHAPE,
    img_shm_name=IMG_SHM_NAME,
    ngrok=False,
    webrtc=False
)

# ==================== 数据读取线程 ====================
import numpy as np

def read_tv_data(tv: TeleVuer):
    prev_head_pose = None
    prev_left_arm_pose = None
    prev_right_arm_pose = None

    while True:
        try:
            # 读取当前数据
            head_pose = tv.head_pose
            left_arm_pose = tv.left_arm_pose
            right_arm_pose = tv.right_arm_pose

            changed = False

            if prev_head_pose is None or not np.array_equal(prev_head_pose, head_pose):
                print("Head Pose:\n", head_pose)
                changed = True

            if prev_left_arm_pose is None or not np.array_equal(prev_left_arm_pose, left_arm_pose):
                print("Left Arm Pose:\n", left_arm_pose)
                changed = True

            if prev_right_arm_pose is None or not np.array_equal(prev_right_arm_pose, right_arm_pose):
                print("Right Arm Pose:\n", right_arm_pose)
                changed = True

            if changed:
                print("-" * 50)

            # 更新上一帧
            prev_head_pose = head_pose.copy()
            prev_left_arm_pose = left_arm_pose.copy()
            prev_right_arm_pose = right_arm_pose.copy()

        except Exception as e:
            print("Error reading TeleVuer data:", e)

        time.sleep(0.01)  # 100 Hz
# ==================== 启动线程 ====================
data_thread = threading.Thread(target=read_tv_data, args=(tv,))
start_thread = threading.Thread(target=tv.start(), args=())
data_thread.daemon = True  # 主线程退出时自动结束
data_thread.start()
# 主线程阻塞
while True:
    time.sleep(1)
