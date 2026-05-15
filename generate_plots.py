import numpy as np
import scipy.linalg
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib
import sys

# 设置中文字体，防止中文显示乱码
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
matplotlib.rcParams['axes.unicode_minus'] = False

# --- 1. 系统参数预设 ---
m1, m2 = 1.0, 1.0
l1, l2 = 1.0, 2.0
lc1, lc2 = 0.5, 1.0
g = 9.81

x_star = np.array([np.pi, 0.0, 0.0, 0.0])
E_star = m1 * g * lc1 + m2 * g * (l1 + lc2)

# --- 2. 动力学矩阵求取 ---
def get_matrices(x):
    theta1, theta2, dtheta1, dtheta2 = x
    d11 = m1 * lc1**2 + m2 * (l1**2 + lc2**2 + 2 * l1 * lc2 * np.cos(theta2))
    d22 = m2 * lc2**2
    d12 = m2 * (lc2**2 + l1 * lc2 * np.cos(theta2))
    M = np.array([[d11, d12], [d12, d22]])
    
    h = -m2 * l1 * lc2 * np.sin(theta2)
    C = np.array([[h * dtheta2, h * dtheta2 + h * dtheta1], 
                  [-h * dtheta1, 0.0]])
    
    phi1 = -m1 * g * lc1 * np.sin(theta1) - m2 * g * (l1 * np.sin(theta1) + lc2 * np.sin(theta1 + theta2))
    phi2 = -m2 * g * lc2 * np.sin(theta1 + theta2)
    G = np.array([phi1, phi2])
    
    B = np.array([0, 1])
    return M, C, G, B

def calc_energy(x):
    """计算系统当前动能与势能之和"""
    theta1, theta2, dtheta1, dtheta2 = x
    V = -m1 * g * lc1 * np.cos(theta1) - m2 * g * (l1 * np.cos(theta1) + lc2 * np.cos(theta1 + theta2))
    M, _, _, _ = get_matrices(x)
    dq = np.array([dtheta1, dtheta2])
    T = 0.5 * dq.T @ M @ dq
    return T + V

# --- 3. 求解 LQR 增益 ---
def design_lqr():
    d11 = m1 * lc1**2 + m2 * (l1**2 + lc2**2 + 2 * l1 * lc2)
    d22 = m2 * lc2**2
    d12 = m2 * (lc2**2 + l1 * lc2)
    M_star = np.array([[d11, d12], [d12, d22]])
    M_inv = np.linalg.inv(M_star)
    
    g11 = m1*g*lc1 + m2*g*(l1 + lc2)
    g12 = m2*g*lc2
    g21 = m2*g*lc2
    g22 = m2*g*lc2
    dG_dq = np.array([[g11, g12], [g21, g22]])
    
    A = np.zeros((4, 4))
    A[0:2, 2:4] = np.eye(2)
    A[2:4, 0:2] = M_inv @ dG_dq
    
    B_sys = np.zeros((4, 1))
    B_sys[2:4, 0] = M_inv @ np.array([0, 1])
    
    Q = np.diag([100, 100, 1, 1])
    R = np.array([[1.0]])
    
    P = scipy.linalg.solve_continuous_are(A, B_sys, Q, R)
    K = np.linalg.inv(R) @ B_sys.T @ P
    return K

K_lqr = design_lqr()

# --- 4. 双模切换混合控制器 ---
# 为了方便作图，记录控制输入
u_history = []
t_u_history = []

def dual_mode_controller(t, x):
    err = x - x_star
    err[0] = (err[0] + np.pi) % (2 * np.pi) - np.pi
    err[1] = (err[1] + np.pi) % (2 * np.pi) - np.pi
    
    E_current = calc_energy(x)
    E_tilde = E_current - E_star
    
    delta_x = 0.5
    epsilon_E = 0.2
    
    if np.linalg.norm(err) < delta_x and abs(E_tilde) < epsilon_E:
        u = - (K_lqr @ err)[0]
    else:
        k_energy = 2.0
        u = k_energy * E_tilde * x[3] 
        
    u_max = 30.0
    u_clip = np.clip(u, -u_max, u_max)
    
    return u_clip

def acrobot_dynamics(t, x):
    theta1, theta2, dtheta1, dtheta2 = x
    dq = np.array([dtheta1, dtheta2])
    
    u = dual_mode_controller(t, x)
    
    # 记录 u 用于绘图 (注意这里被微分迭代器高频调用，需要后期对齐)
    t_u_history.append(t)
    u_history.append(u)
    
    M, C, G, B = get_matrices(x)
    M_inv = np.linalg.inv(M)
    ddq = M_inv @ (B * u - C @ dq - G)
    
    return [dtheta1, dtheta2, ddq[0], ddq[1]]

if __name__ == "__main__":
    t_span = (0, 10.0)
    t_eval = np.linspace(t_span[0], t_span[1], 1000)
    x0 = [0.1, 0.0, 0.0, 0.0]
    
    print("正在生成报告所需图片...")
    sol = solve_ivp(acrobot_dynamics, t_span, x0, t_eval=t_eval, method='RK45')
    
    # 获取各个时刻的正式平滑u和Energy
    u_eval = []
    E_eval = []
    for i in range(len(sol.t)):
        x_i = sol.y[:, i]
        u_eval.append(dual_mode_controller(sol.t[i], x_i))
        E_eval.append(calc_energy(x_i))
        
    E_eval = np.array(E_eval)
    u_eval = np.array(u_eval)
    
    # ========= 生成 图 5-1 状态响应曲线 =========
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(sol.t, sol.y[0,:], label=r'$\theta_1$', color='b')
    plt.plot(sol.t, sol.y[1,:], label=r'$\theta_2$', color='g')
    plt.axhline(np.pi, color='r', linestyle='--', label=r'Target $\theta_1=\pi$')
    plt.axhline(0, color='gray', linestyle='--', label=r'Target $\theta_2=0$')
    plt.ylabel('Angle [rad]')
    plt.title('图 5-1: 角度状态响应曲线')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(sol.t, sol.y[2,:], label=r'$\dot{\theta}_1$', color='b')
    plt.plot(sol.t, sol.y[3,:], label=r'$\dot{\theta}_2$', color='g')
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [rad/s]')
    plt.title('角速度响应曲线')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('fig5_1_state_response.png', dpi=300)
    print("图 5-1 (fig5_1_state_response.png) 生成成功！")
    
    # ========= 生成 图 5-2 控制输入 u(t) 曲线 =========
    plt.figure(figsize=(10, 3.5))
    plt.plot(sol.t, u_eval, color='purple', label='Control Torque u(t)')
    plt.axhline(30, color='r', linestyle=':', label='Max Torque Limit')
    plt.axhline(-30, color='r', linestyle=':')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('图 5-2: 控制输入 u(t) 响应曲线')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('fig5_2_control_input.png', dpi=300)
    print("图 5-2 (fig5_2_control_input.png) 生成成功！")
    
    # ========= 生成 图 5-3 能量曲线 =========
    plt.figure(figsize=(10, 4))
    plt.plot(sol.t, E_eval, color='orange', label='Total Energy $E(t)$')
    plt.axhline(E_star, color='r', linestyle='--', label=r'Target Energy $E^*$')
    plt.xlabel('Time [s]')
    plt.ylabel('Energy [J]')
    plt.title('图 5-3: 系统总能量随时间演化曲线')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('fig5_3_energy.png', dpi=300)
    print("图 5-3 (fig5_3_energy.png) 生成成功！")
    
    # ========= 生成 图 相平面轨迹 =========
    plt.figure(figsize=(6, 6))
    plt.plot(sol.y[0,:], sol.y[2,:], color='b', label=r'$\theta_1$ Phase')
    plt.plot(sol.y[1,:], sol.y[3,:], color='g', alpha=0.6, label=r'$\theta_2$ Phase')
    plt.plot(np.pi, 0, 'r*', markersize=12, label='Target Equilibrium')
    plt.xlabel(r'Angle $\theta$ [rad]')
    plt.ylabel(r'Velocity $\dot{\theta}$ [rad/s]')
    plt.title('图 5-3 (附加): 相平面轨迹')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('fig5_3B_phase_plane.png', dpi=300)
    print("图 5-3B 相平面图 (fig5_3B_phase_plane.png) 生成成功！")
    
    import matplotlib.animation as animation
    # ========= 生成 图 5-4 Acrobot 系统起摆动画 =========
    print("正在生成图 5-4 Acrobot 起摆动图 (GIF)，这可能需要数十秒时间，请稍候...")
    fig_ani = plt.figure(figsize=(6, 6))
    # 半径分别为 1.0 和 2.0，所以空间范围大约是 -3 到 3
    ax_ani = fig_ani.add_subplot(111, autoscale_on=False, xlim=(-3.5, 3.5), ylim=(-3.5, 3.5))
    ax_ani.set_aspect('equal')
    ax_ani.grid()
    ax_ani.set_title('图 5-4: Acrobot 起摆倒立平衡动画')
    
    line, = ax_ani.plot([], [], 'o-', lw=4, markersize=8, color='#FF5733')
    time_template = 'Time = %.1f s'
    time_text = ax_ani.text(0.05, 0.9, '', transform=ax_ani.transAxes, fontsize=12)
    
    # 提取时间与角度轨迹，使用高密度采样来彻底解决抽搐卡顿问题
    skip = 2
    t_frames = sol.t[::skip]
    th1_frames = sol.y[0, ::skip]
    th2_frames = sol.y[1, ::skip]
    
    def init():
        line.set_data([], [])
        time_text.set_text('')
        return line, time_text
        
    def animate(i):
        th1 = th1_frames[i]
        th2 = th2_frames[i]
        
        # 运动学正解计算坐标
        # 根据重力势能模型，$\theta_1=0$ 为竖直向下
        x0, y0 = 0, 0
        x1 = l1 * np.sin(th1)
        y1 = -l1 * np.cos(th1)
        x2 = x1 + l2 * np.sin(th1 + th2)
        y2 = y1 - l2 * np.cos(th1 + th2)
        
        line.set_data([x0, x1, x2], [y0, y1, y2])
        time_text.set_text(time_template % (t_frames[i]))
        return line, time_text
        
    ani = animation.FuncAnimation(
        fig_ani, animate, np.arange(0, len(t_frames)),
        interval=20, blit=True, init_func=init
    )
                                  
    # 保存为 gif 格式：设定 fps=50 匹配高密度采样，实现真正的丝滑效果
    ani.save('fig5_4_acrobot_animation.gif', writer='pillow', fps=50)
    print("图 5-4 动图 (fig5_4_acrobot_animation.gif) 生成成功！")

