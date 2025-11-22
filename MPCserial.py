
import numpy as np
import cvxpy as cp
import serial
import time
import threading
from queue import Queue
import struct

class STM32MPCController:
    def __init__(self, port='/dev/ttyS1', baudrate=115200):
        # 串口通信参数
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.is_connected = False
        self.T = 0.1  
        self.N = 5    
        self.amplitude = 0.8
        self.frequency = 0.3
        self.speed = 0.3
        self.poly_speed = 0.2
        self.receive_buffer = bytearray()
    
        self.Q = np.diag([50, 50, 10])   
        self.R = np.diag([1.0, 1.0])     
        self.Qf = np.diag([100, 100, 20]) 
        self.poly_coeffs = [0, 1, 0, 0]  # 多项式系数 [a0, a1, a2, a3]
        self.v_max = 2
        self.v_min = -2.0
        self.w_max = 1
        self.w_min = -1
        self.trajectory_type = "polynomial"


        self.estimated_state = np.array([0.0, 0.0, 0.0]) 
        self.state_lock = threading.Lock()

        self.control_queue = Queue()

        self.reference_trajectory = []
        self.actual_trajectory = []
        self.control_history = []

        self.running = False
        
    def connect_serial(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.is_connected = True
            print(f"成功连接到串口 {self.port}")
            return True
        except Exception as e:
            print(f"串口连接失败: {e}")
            return False
    
    def disconnect_serial(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False
        print("串口连接已关闭")
    
    def send_control_command(self, v, w):
        """发送控制命令到STM32"""
        if not self.is_connected or not self.ser:
            print("串口未连接")
            return False
        
        try:

            cmd_bytearray = bytearray()

            cmd_bytearray.append(0xAA)

            v_byte = int(abs(v) * 100) 
            cmd_bytearray.append(min(v_byte, 255))
            
            w_byte = int(abs(w) * 50)  
            cmd_bytearray.append(min(w_byte, 255))

            if v >= 0 and w == 0:
                direction = 0  # 纯前进
            elif v >= 0 and w > 0:
                direction = 1  # 前进+左转
            elif v >= 0 and w < 0:
                direction = 2  # 前进+右转
            elif v < 0 and w > 0:
                direction = 3  # 后退+左转
            elif v < 0 and w < 0:
                direction = 4  # 后退+右转
            else:
                direction = 5  # 纯后退
                
            cmd_bytearray.append(direction)

            cmd_bytearray.extend([0, 0, 0, 0])

            module = 1  
            cmd_bytearray.append(module)

            checksum = 0
            for i in range(1, 9):  
                checksum ^= cmd_bytearray[i]
            cmd_bytearray.append(checksum)

            cmd_bytearray.append(0xBB)

            self.ser.write(cmd_bytearray)

            self.control_history.append((v, w))
            
            return True
            
        except Exception as e:
            print(f"发送控制命令失败: {e}")
            return False
    
    def receive_state_data(self):
        """接收STM32的状态数据 (需要您根据实际协议实现)"""
        if not self.is_connected or not self.ser:
             return None
    
        try:
            # 读取所有可用数据到缓冲区
            data_available = self.ser.in_waiting
            if data_available > 0:
                new_data = self.ser.read(data_available)
                self.receive_buffer.extend(new_data)
            
            # 在缓冲区中查找完整的数据帧
            while len(self.receive_buffer) >= 22:
                # 查找帧头 0xCC
                frame_start = -1
                for i in range(len(self.receive_buffer) - 21):
                    if self.receive_buffer[i] == 0xCC:
                        frame_start = i
                        break
                
                # 如果没有找到帧头，清空无效数据
                if frame_start == -1:
                    # 保留最后21个字节，可能包含部分帧头
                    self.receive_buffer = self.receive_buffer[-21:]
                    break
                
                # 检查帧尾 0xDD
                if self.receive_buffer[frame_start + 21] == 0xDD:
                    # 提取完整数据帧
                    data_frame = self.receive_buffer[frame_start:frame_start + 22]
                    
                    # 解析数据
                    x = struct.unpack('f', data_frame[1:5])[0] 
                    y = struct.unpack('f', data_frame[5:9])[0]  
                    theta = struct.unpack('f', data_frame[9:13])[0]
                    theta = theta / 180 * np.pi
                    self.v = struct.unpack('f', data_frame[13:17])[0]
                    self.W = struct.unpack('f', data_frame[17:21])[0]
                    
                    with self.state_lock:
                        self.estimated_state = np.array([x, y, theta])
                        self.actual_trajectory.append(self.estimated_state.copy())
                    
                    # 移除已处理的数据
                    self.receive_buffer = bytearray()
                    
                    return self.estimated_state.copy()
                else:
                    # 帧尾不匹配，跳过这个帧头继续查找
                    self.receive_buffer = self.receive_buffer[frame_start + 1:]
            
            return None
        
        except Exception as e:
            print(f"接收状态数据失败: {e}")
            # 发生异常时清空缓冲区
            self.receive_buffer = bytearray()
            return None
    
    def state_estimation_thread(self):
        """状态估计线程"""
        while self.running:
            self.receive_state_data()
            time.sleep(0.01)  
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
        """计算线性化矩阵"""
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
        
        # 向量 O
        O = -A @ np.array([x_r, y_r, theta_r])
        
        # 离散化
        I = np.eye(3)
        A_discrete = I + self.T * A
        B_discrete = self.T * B
        O_discrete = self.T * O
        
        return A_discrete, B_discrete, O_discrete.flatten()
    
    def solve_mpc(self, current_state, ref_traj, ref_controls):
        """求解MPC优化问题"""
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
            problem.solve(solver=cp.OSQP, verbose=False, max_iter=2000)
            
            if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                if controls[0].value is not None:
                    control_value = np.array([float(controls[0].value[0]), float(controls[0].value[1])])
                    return control_value
                else:
                    print("MPC求解成功但控制量为None")
                    return ref_controls[0].copy()
            else:
                print(f"MPC求解失败: {problem.status}")
                return ref_controls[0].copy()
                
        except Exception as e:
            print(f"MPC求解异常: {e}")
            return ref_controls[0].copy()
    
    def run_mpc_control(self, duration=60):
        """运行MPC控制主循环"""
        if not self.connect_serial():
            print("无法连接串口，退出控制")
            return
        
        print("开始MPC实时控制...")
        self.running = True
        
        # 启动状态估计线程
        state_thread = threading.Thread(target=self.state_estimation_thread)
        state_thread.daemon = True
        state_thread.start()
        
        start_time = time.time()
        step = 0
        
        try:
            while time.time() - start_time < duration and self.running:
                cycle_start = time.time()
                
                # 获取当前状态
                with self.state_lock:
                    current_state = self.estimated_state.copy()
                
                # 生成参考轨迹
                ref_traj, ref_controls = self.get_reference_trajectory(step)
                self.reference_trajectory.append(ref_traj[0])

                control = self.solve_mpc(current_state, ref_traj, ref_controls)
                
                # 发送控制命令
                success = self.send_control_command(control[0], control[1])
                
                if success:
                    # 计算跟踪误差
                    pos_error = np.linalg.norm(current_state[:2] - ref_traj[0, :2])
                    angle_error = abs(self.normalize_angle(current_state[2] - ref_traj[0, 2]))
                    
                    if step % 20 == 0:
                        print(f"步数: {step:3d}, 位置: ({current_state[0]:.3f}, {current_state[1]:.3f}), "
                              f"控制: ({control[0]:.3f}, {control[1]:.3f}), "
                              f"位置误差: {pos_error:.3f}, 角度误差: {angle_error:.3f}")
                
                # 控制周期等待
                elapsed = time.time() - cycle_start
                sleep_time = max(0, self.T - elapsed)
                time.sleep(sleep_time)
                
                step += 1
                
        except KeyboardInterrupt:
            print("用户中断控制")
        finally:
            self.running = False
            # 发送停止命令
            self.send_control_command(0, 0)
            self.disconnect_serial()
            print("MPC控制结束")
    
    def plot_results(self):
        """绘制结果 (如果接收到足够的状态数据)"""
        if len(self.actual_trajectory) < 2:
            print("没有足够的状态数据用于绘图")
            return
        
        import matplotlib.pyplot as plt
        
        actual_traj = np.array(self.actual_trajectory)
        ref_traj = np.array(self.reference_trajectory)
        
        plt.figure(figsize=(12, 8))
        
        # 轨迹图
        plt.subplot(2, 2, 1)
        if len(ref_traj) > 0:
            plt.plot(ref_traj[:, 0], ref_traj[:, 1], 'r--', label='参考轨迹', linewidth=2)
        plt.plot(actual_traj[:, 0], actual_traj[:, 1], 'b-', label='实际轨迹', linewidth=2)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('机器人轨迹')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # 控制量
        plt.subplot(2, 2, 2)
        if len(self.control_history) > 0:
            controls = np.array(self.control_history)
            plt.plot(controls[:, 0], 'g-', label='线速度 v', linewidth=2)
            plt.plot(controls[:, 1], 'm-', label='角速度 w', linewidth=2)
            plt.xlabel('时间步')
            plt.ylabel('控制量')
            plt.title('控制输入')
            plt.legend()
            plt.grid(True)
        
        plt.tight_layout()
        plt.show()


# 使用示例
if __name__ == "__main__":
    # 创建控制器实例，请根据实际情况修改串口号
    mpc_controller = STM32MPCController(port='/dev/ttyS1', baudrate=115200)
    
    # 运行MPC控制 (持续60秒)
    mpc_controller.run_mpc_control(duration=10)
    
    # 绘制结果
    mpc_controller.plot_results()