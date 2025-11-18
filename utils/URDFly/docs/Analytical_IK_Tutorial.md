# 机械臂逆运动学解析解教程

## YAIK库

有了MDH参数之后，可以使用[yaik](https://github.com/weigao95/yaik)库进行解析解的生成。

### 构建机器人模型

在`yaik`库下的`fk/robot_models.py`中新建MDH的机器人模型，如下：

```python
def puma_robot() -> RobotDescription:
    n_dofs = 6
    robot = RobotDescription("puma")
    robot.unknowns = default_unknowns(n_dofs)
    a_2 = sp.Symbol('a_2')
    a_3 = sp.Symbol('a_3')
    d_1 = sp.Symbol('d_1')
    d_3 = sp.Symbol('d_3')
    d_4 = sp.Symbol('d_4')
    dh_0 = DHEntry(0, 0, d_1, robot.unknowns[0].symbol) # alpha, a, d, theta
    dh_1 = DHEntry(-sp.pi / 2, 0, 0, robot.unknowns[1].symbol)
    dh_2 = DHEntry(0, a_2, d_3, robot.unknowns[2].symbol)
    dh_3 = DHEntry(-sp.pi / 2, a_3, d_4, robot.unknowns[3].symbol)
    dh_4 = DHEntry(-sp.pi / 2, 0, 0, robot.unknowns[4].symbol)
    dh_5 = DHEntry(sp.pi / 2, 0, 0, robot.unknowns[5].symbol)
    robot.dh_params = [dh_0, dh_1, dh_2, dh_3, dh_4, dh_5]

    # Make the robot
    robot.symbolic_parameters = {d_1, a_2, a_3, d_3, d_4}
    robot.parameters_value = {d_1: 0.6, a_2: 0.432, a_3: 0.0203, d_3: 0.1245, d_4: 0.432}
    robot.parameters_bound = dict()
    return robot
```

其中`unknowns`即为MDH参数中的$\theta$，注意当$\theta$默认不是0的时候，需要增加offset，如下：

```python
# Add auxiliary data
pi_float = float(np.pi)
robot.auxiliary_data = RobotAuxiliaryData()
robot.auxiliary_data.unknown_offset = [0.0, pi_float, 0.0, -pi_float, 0.0, -pi_float, pi_float]
```

### 解析解求解

运行`yaik`库下的`ik_solve.py`进行求解，注意使用`run_robot_from_script`函数：

```python
def run_robot_from_script():
    robot_to_solve = robot_models.puma_robot() # 替换为自己的机器人模型
    test_case_path = None
    option = RunIKOption()
    option.use_all_intersection_pair_axis_equation = True
    option.try_intersecting_axis_equation = True
    # test_case_path = './gallery/test_data/franka_panda_numerical_test.yaml'
    # test_case_path = test_case_path if os.path.exists(test_case_path) else None
    run_ik(robot_to_solve, test_case_path, option)
```

运行完（需要花费较长时间，耐心等待）之后目录下会最终生成一个`*_ik.yaml`文件，然后运行目录下`py_codegen.py`生成python代码，注意修改里面的yaml文件路径。运行完之后会生成`*_ik_generated.py`文件，可以直接运行进行求解。
