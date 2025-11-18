import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

class FixedDifferentialDriveMPC:
    def __init__(self):
        # 控制参数
        self.T = 0.1  # 控制周期
        self.N =5   # 预测时域
        
        # 机器人参数
        self.radius = 2.0  # 圆形轨迹半径
        self.amplitude = 1.0  # 正弦波振幅
        self.frequency =0.3  # 正弦波频率
        # MPC权重矩阵
        self.Q = np.diag([500, 500, 50])   # 状态权重
        self.R = np.diag([0.8, 0.8])     # 控制权重
        self.Qf = np.diag([30, 30, 10]) # 终端状态权重
        self.theta_init = 0.0
        # 控制约束
        self.v_max = 2
        self.v_min = -2.0
        self.w_max = 1
        self.w_min = -1
        self.trajectory_type = "polynomial"
        self.poly_coeffs = [2.5, 0.3, 0.02, 0.0045]  # 多项式系数 [a0, a1, a2, a3]
        self.poly_speed = 0.2  # 多项式轨迹前进速度
        # 初始状态 [x, y, theta]
        self.state = np.array([1, 1.0, 0.0])
        
        # 存储轨迹
        self.actual_trajectory = [self.state.copy()]
        self.reference_trajectory = []
        
        # 控制历史
        self.control_history = []
        
    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    def polynomial_trajectory(self, t):
        """多项式轨迹: y = a0 + a1*x + a2*x^2 + a3*x^3, x = speed * t"""
        x = self.poly_speed * t
        y = (self.poly_coeffs[0] + 
             self.poly_coeffs[1] * x + 
             self.poly_coeffs[2] * x**2 + 
             self.poly_coeffs[3] * x**3)
        return x, y
    
    def polynomial_velocity(self, t, dt=0.01):
        """多项式轨迹的速度计算"""
        x1, y1 = self.polynomial_trajectory(t)
        x2, y2 = self.polynomial_trajectory(t + dt)
        dx = x2 - x1
        dy = y2 - y1
        return dx, dy
    
    def polynomial_curvature(self, t, dt=0.01):
        """多项式轨迹的曲率计算"""
        # 计算三个点的位置
        x0, y0 = self.polynomial_trajectory(t - dt)
        x1, y1 = self.polynomial_trajectory(t)
        x2, y2 = self.polynomial_trajectory(t + dt)
        
        # 计算一阶导数（速度）
        dx1 = x1 - x0
        dy1 = y1 - y0
        dx2 = x2 - x1
        dy2 = y2 - y1
        
        # 计算二阶导数（加速度）
        d2x = dx2 - dx1
        d2y = dy2 - dy1
        
        # 曲率公式: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        denominator = (dx1**2 + dy1**2)**1.5
        if denominator > 1e-6:
            curvature = abs(dx1*d2y - dy1*d2x) / denominator
        else:
            curvature = 0
            
        return curvature
    
    def get_reference_trajectory(self, current_step):
        """生成正弦波参考轨迹 - 不闭合曲线"""
        ref_traj = []
        ref_controls = []
        
        prev_theta = 0.0  # 初始角度
        
        for k in range(self.N + 1):
            t = (current_step + k) * self.T

            # 根据轨迹类型计算参考位置
            if self.trajectory_type == "sinusoidal":
                # 正弦波轨迹
                x_ref = self.speed * t
                y_ref = self.amplitude * np.sin(self.frequency * t)
            else:
                # 多项式轨迹
                x_ref, y_ref = self.polynomial_trajectory(t)
            
            # 计算角度（切线方向）
            if k == 0:
                # 第一个点使用速度向量计算角度
                if self.trajectory_type == "sinusoidal":
                    dx_dt = self.speed
                    dy_dt = self.amplitude * self.frequency * np.cos(self.frequency * t)
                else:
                    dx_dt, dy_dt = self.polynomial_velocity(t)
                
                if abs(dx_dt) < 1e-6 and abs(dy_dt) < 1e-6:
                    theta_ref = prev_theta
                else:
                    theta_ref = np.arctan2(dy_dt, dx_dt)
                    
                if t == 0:
                    self.theta_init = theta_ref
            else:
                # 后续点使用有限差分计算角度
                t_prev = (current_step + k - 1) * self.T
                
                if self.trajectory_type == "sinusoidal":
                    x_prev = self.speed * t_prev
                    y_prev = self.amplitude * np.sin(self.frequency * t_prev)
                else:
                    x_prev, y_prev = self.polynomial_trajectory(t_prev)
                
                dx = x_ref - x_prev
                dy = y_ref - y_prev
                
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    theta_ref = prev_theta  
                else:
                    theta_ref = np.arctan2(dy, dx)

            # 角度归一化
            theta_ref = self.normalize_angle(theta_ref)
            
            ref_traj.append(np.array([x_ref, y_ref, theta_ref]))
            
            # 计算参考控制量
            if k < self.N:
                # 计算曲率
                if self.trajectory_type == "sinusoidal":
                    # 正弦波曲率计算
                    t_next = (current_step + k + 1) * self.T
                    x_next = self.speed * t_next
                    y_next = self.amplitude * np.sin(self.frequency * t_next)
                    
                    dx1 = x_ref - x_prev if k > 0 else self.speed
                    dy1 = y_ref - y_prev if k > 0 else self.amplitude * self.frequency * np.cos(self.frequency * t)
                    dx2 = x_next - x_ref
                    dy2 = y_next - y_ref
                    
                    theta1 = np.arctan2(dy1, dx1)
                    theta2 = np.arctan2(dy2, dx2)
                    delta_theta = self.normalize_angle(theta2 - theta1)
                    delta_s = np.sqrt(dx2**2 + dy2**2)
                    
                    if delta_s > 1e-6:
                        curvature = delta_theta / delta_s
                    else:
                        curvature = 0
                else:
                    # 多项式曲率计算
                    curvature = self.polynomial_curvature(t)
                
                v_ref = self.speed if self.trajectory_type == "sinusoidal" else self.poly_speed
                w_ref = v_ref * curvature
            else:
                v_ref = self.speed if self.trajectory_type == "sinusoidal" else self.poly_speed
                w_ref = 0
            
            # 应用控制约束
            v_ref = np.clip(v_ref, self.v_min, self.v_max)
            w_ref = np.clip(w_ref, self.w_min, self.w_max)
            
            ref_controls.append(np.array([v_ref, w_ref]))
            prev_theta = theta_ref
            
        return np.array(ref_traj), np.array(ref_controls)
    
    def set_trajectory_type(self, traj_type):
        """设置轨迹类型"""
        self.trajectory_type = traj_type
        print(f"轨迹类型设置为: {traj_type}")
    
    def set_polynomial_coeffs(self, coeffs):
        """设置多项式系数 [a0, a1, a2, a3]"""
        if len(coeffs) == 4:
            self.poly_coeffs = coeffs
            print(f"多项式系数设置为: {coeffs}")
        else:
            print("错误: 多项式系数必须是4个元素")
    
    def set_polynomial_speed(self, speed):
        """设置多项式轨迹速度"""
        self.poly_speed = speed
        print(f"多项式轨迹速度设置为: {speed}")
    
    def normalize_angle_continuous(self, angle, prev_angle):
        """将角度归一化并确保连续性"""
        # 基本归一化
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        
        # 确保与前一角度连续（避免2π跳变）
        while angle - prev_angle > np.pi:
            angle -= 2 * np.pi
        while angle - prev_angle < -np.pi:
            angle += 2 * np.pi
        
        return angle
    
    def get_linearized_matrices(self, state_ref, control_ref):
   
        x_r, y_r, theta_r = state_ref
        v_r, w_r = control_ref
        
        # 矩阵 A
        A = np.array([
            [0, 0, -v_r * np.sin(theta_r)],
            [0, 0, v_r * np.cos(theta_r)],
            [0, 0, 0]
        ])
        
        # 矩阵 B
        B = np.array([
            [np.cos(theta_r), 0],
            [np.sin(theta_r), 0],
            [0, 1]
        ])
        
        # 向量 O - 修复：确保是1D数组
        O = -A @ np.array([x_r, y_r, theta_r])
        
        # 离散化
        I = np.eye(3)
        A_discrete = I + self.T * A
        B_discrete = self.T * B
        O_discrete = self.T * O
        
        # 确保返回的是正确维度的数组
        return A_discrete, B_discrete, O_discrete.flatten()
    
    def solve_mpc(self, current_state, ref_traj, ref_controls):
        """求解MPC优化问题 - 完全修复版本"""
        try:
            # 优化变量
            states = cp.Variable((self.N + 1, 3))
            controls = cp.Variable((self.N, 2))
            
            # 初始状态约束
            constraints = [states[0] == current_state]
            
            cost = 0
            
            for k in range(self.N):
                if(k==0):
                    A_k, B_k, O_k = self.get_linearized_matrices(current_state, ref_controls[k])
                else:
                    A_k, B_k, O_k = self.get_linearized_matrices( ref_traj[k], ref_controls[k])
                constraints.append(states[k+1] == A_k @ states[k] + B_k @ controls[k] + O_k)

                constraints.append(controls[k, 0] <= self.v_max)
                constraints.append(controls[k, 0] >= self.v_min)
                constraints.append(controls[k, 1] <= self.w_max)
                constraints.append(controls[k, 1] >= self.w_min)

                state_error = states[k] - ref_traj[k]
               
                control_error = controls[k] - ref_controls[k]
                
                cost += cp.quad_form(state_error, self.Q)
                cost += cp.quad_form(control_error, self.R)
            

            terminal_error = states[self.N] - ref_traj[self.N]
            cost += cp.quad_form(terminal_error, self.Qf)

            problem = cp.Problem(cp.Minimize(cost), constraints)
            problem.solve(solver=cp.OSQP, verbose=False, max_iter=5000)
            
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                if controls[0].value is not None:
                    control_value = np.array([float(controls[0].value[0]), float(controls[0].value[1])])
                    return control_value, states.value
                else:
                    print("MPC求解成功但控制量为None")
                    return ref_controls[0].copy(), None
            else:
                print(f"MPC求解失败: {problem.status}")
                return ref_controls[0].copy(), None
                
        except Exception as e:
            print(f"MPC求解异常: {e}")
            import traceback
            traceback.print_exc()
            return ref_controls[0].copy(), None
    
    def update_robot_dynamics(self, state, control, dt=None):
        """更新机器人非线性动力学"""
        if dt is None:
            dt = self.T
            
        x, y, theta = state
        v, w = control
        

        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + w * dt
     
        theta_new = self.normalize_angle(theta_new)
        
        return np.array([x_new, y_new, theta_new])
    def update_robot_dynamics2(self, state, control, dt=None):
        """更新机器人非线性动力学"""
        if dt is None:
            dt = self.T
            
        x, y, theta = state
        v, w = control
        
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + w * dt

        theta_new = self.normalize_angle(theta_new)
        
        return np.array([x_new, y_new, theta_new])
    
    def run_simulation(self, simulation_steps=200):
        """运行仿真"""
        print("开始修复的MPC轨迹跟踪仿真...")
        
        success_count = 0
        failure_count = 0
        
        for step in range(simulation_steps):
            
            ref_traj, ref_controls = self.get_reference_trajectory(step)
            self.reference_trajectory.append(ref_traj[0])

          
            control, predicted_states = self.solve_mpc(self.state, ref_traj, ref_controls)
            
            if control is not None:
                success_count += 1
            else:
                failure_count += 1
                control = ref_controls[0].copy()

            self.control_history.append(control.copy())
            
            # 应用控制输入
            self.state = self.update_robot_dynamics(self.state, control)
            self.actual_trajectory.append(self.state.copy())
            
            # 计算跟踪误差
            pos_error = np.linalg.norm(self.state[:2] - ref_traj[0, :2])
            angle_error = abs(self.normalize_angle(self.state[2] - ref_traj[0, 2]))
            if step % 20 == 0:
                print(f"步数: {step:3d}, 位置: ({float(self.state[0]):.3f}, {float(self.state[1]):.3f}), "
                      f"控制: ({float(control[0]):.3f}, {float(control[1]):.3f}), "
                      f"位置误差: {pos_error:.3f}, 角度误差: {angle_error:.3f}")

        
        print(f"\nMPC求解统计: 成功 {success_count} 次, 失败 {failure_count} 次")
    
    def plot_results(self):
        """绘制结果"""
        actual_traj = np.array(self.actual_trajectory)
        ref_traj = np.array(self.reference_trajectory)
        control_history = np.array(self.control_history)
        
        plt.figure(figsize=(15, 10))
        
        # 轨迹图
        plt.subplot(2, 2, 1)
        plt.plot(ref_traj[:, 0], ref_traj[:, 1], 'r--', label='参考轨迹', linewidth=2)
        plt.plot(actual_traj[:, 0], actual_traj[:, 1], 'b-', label='实际轨迹', linewidth=2)
        plt.plot(actual_traj[0, 0], actual_traj[0, 1], 'go', markersize=8, label='起点')
        plt.plot(actual_traj[-1, 0], actual_traj[-1, 1], 'ro', markersize=8, label='终点')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('机器人轨迹')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # 跟踪误差
        plt.subplot(2, 2, 2)
        min_len = min(len(actual_traj), len(ref_traj))
        position_error = np.linalg.norm(actual_traj[:min_len, :2] - ref_traj[:min_len, :2], axis=1)
        
        plt.plot(position_error, 'b-', linewidth=2)
        plt.xlabel('时间步')
        plt.ylabel('位置误差 (m)')
        plt.title('跟踪误差')
        plt.grid(True)
        
        # 控制量
        plt.subplot(2, 2, 3)
        plt.plot(control_history[:, 0], 'g-', label='线速度 v', linewidth=2)
        plt.plot(control_history[:, 1], 'm-', label='角速度 w', linewidth=2)
        plt.xlabel('时间步')
        plt.ylabel('控制量')
        plt.title('控制输入')
        plt.legend()
        plt.grid(True)
        
        # 角度变化
        plt.subplot(2, 2, 4)
        actual_angles = np.array([self.normalize_angle(angle) for angle in actual_traj[:, 2]])
        ref_angles = np.array([self.normalize_angle(angle) for angle in ref_traj[:len(actual_traj), 2]])
        
        plt.plot(np.degrees(actual_angles), 'b-', label='实际角度', linewidth=2)
        plt.plot(np.degrees(ref_angles), 'r--', label='参考角度', linewidth=2)
        plt.xlabel('时间步')
        plt.ylabel('角度 (度)')
        plt.title('机器人朝向')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # 计算性能指标
        min_len = min(len(actual_traj), len(ref_traj))
        final_error = np.linalg.norm(actual_traj[-1, :2] - ref_traj[-1, :2])
        avg_error = np.mean(position_error[:min_len])
        
        print(f"\n性能指标:")
        print(f"最终位置误差: {final_error:.4f} m")
        print(f"平均位置误差: {avg_error:.4f} m")
        print(f"最大位置误差: {np.max(position_error[:min_len]):.4f} m")

def main():

    mpc_controller = FixedDifferentialDriveMPC()
    

    mpc_controller.run_simulation(simulation_steps=500)
    

    mpc_controller.plot_results()

if __name__ == "__main__":
    main()