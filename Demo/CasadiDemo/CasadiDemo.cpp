#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;
using namespace casadi;

// 二维机械臂前向运动学
SX forward_kinematics(const SX& q) {
    SX x = cos(q(0)) + cos(q(0) + q(1));
    SX y = sin(q(0)) + sin(q(0) + q(1));
    return SX::vertcat({x, y});
}

int main() {
    cout << "2D Robot IK using CasADi" << endl;

    // 优化变量：两个关节角
    SX q = SX::sym("q", 2);

    // 目标末端位置
    DM target_position = DM::vertcat({1.5, 1.0});

    // 前向运动学
    SX fk = forward_kinematics(q);

    // 目标函数：最小化末端误差平方
    SX error = fk - SX(target_position);
    SX objective = dot(error, error);

    // 关节上下界
    vector<double> lbx = {-M_PI/2, -M_PI/2};
    vector<double> ubx = { M_PI/2,  M_PI/2};

    // 将关节约束也作为非线性约束 g
    SX g = SX::vertcat({q - lbx[0], ubx[0] - q});  // 这里对每个关节需要展开

    // 非线性约束上下界
    vector<double> lbg = {0, 0, 0, 0};
    vector<double> ubg = {1e20, 1e20, 1e20, 1e20}; // 大数表示无穷上界

    // NLP
    SXDict nlp = {{"x", q}, {"f", objective}, {"g", g}};

    // 创建求解器
    Dict opts;
    opts["ipopt.tol"] = 1e-8;
    opts["ipopt.max_iter"] = 100;
    opts["print_time"] = 0;
    opts["calc_lam_p"] = 0;
    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // 求解器输入
    map<string, DM> arg, res;
    arg["x0"] = DM::zeros(2);  // 初值
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;

    // 求解
    res = solver(arg);

    // 输出结果
    DM q_sol = res.at("x");
    DM end_pos = forward_kinematics(q_sol);

    cout << "--------------------------------" << endl;
    cout << setw(30) << "Optimal joint angles (rad): " << q_sol << endl;
    cout << setw(30) << "End-effector position: " << end_pos << endl;
    cout << setw(30) << "Objective: " << res.at("f") << endl;
    cout << setw(30) << "Dual solution (x): " << res.at("lam_x") << endl;
    cout << setw(30) << "Dual solution (g): " << res.at("lam_g") << endl;

    return 0;
}
