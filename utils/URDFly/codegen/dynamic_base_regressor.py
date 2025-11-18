import re
import os
from .sympybotics import RobotDef, RobotDynCode, robotcodegen

def create_gx7(mdh_list):
    mdh_list = [[str(m[3]), str(m[2]), str(m[1]), str(m[0]) + '+q'] for m in mdh_list]

    rbtdef = RobotDef(
                "rbt_name",
                mdh_list,
                dh_convention="mdh"
            )

    rbtdef.frictionmodel = {'Coulomb', 'viscous', 'offset'}
    rbtdef.driveinertiamodel = 'simplified'

    rbt = RobotDynCode(rbtdef)
    rbt.calc_base_parms()

    regressor_func_str = robotcodegen.robot_code_to_func('C', rbt.H_code, 'regressor', 'H_func', rbtdef)
    
    # 使用正则表达式匹配所有 regressor[数字] 并提取数字
    numbers = re.findall(r'regressor\[(\d+)\]', regressor_func_str)
    # 将提取的字符串数字转换为整数
    numbers = [int(num) for num in numbers]
    
    NUM_BASE_REGRESSOR = max(numbers) + 1
    
    with open(os.path.join(os.path.dirname(__file__), 'usage/calc_dynamics.py'), 'r', encoding='utf-8') as f:
        calc_dynamics_usage = f.read()

    calc_dynamics_usage = calc_dynamics_usage.replace('$NUM_BASE_REGRESSOR', str(NUM_BASE_REGRESSOR))
    calc_dynamics_usage = calc_dynamics_usage.replace('$BASE_IDXS', str(rbt.dyn.base_idxs))

    return regressor_func_str, calc_dynamics_usage
    












