#!/usr/bin/env python3
import sys
import time
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_custom_msgs.msg import RobotStatus

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
import os

current_path = os.path.dirname(os.path.realpath(__file__))
print(current_path)



class WindowConfig:
    def __init__(self, window_id, data_types, joint_indices,fig_size=(10,8),window_name='default_window',
                 xticks=None, yticks=None,x_major_grid_step=None, x_minor_grid_step=None, y_major_grid_step=None, y_minor_grid_step=None):
        """
        :param window_id: 窗口 ID，用于区分不同的窗口
        :param data_types: 要显示的数据类型，例如 ['position', 'velocity']
        :param joint_indices: 要显示的关节索引列表
        """
        self.window_name = window_name
        self.window_id = window_id
        self.data_types = data_types  # ['positions', 'velocities', 'accelerations']
        self.joint_indices = joint_indices
        self.fig_size = fig_size
        self.xticks = xticks  # 可选参数
        self.yticks = yticks  # 可选参数
        self.x_major_grid_step = x_major_grid_step  # 可选参数
        self.x_minor_grid_step = x_minor_grid_step  # 可选参数
        self.y_major_grid_step = y_major_grid_step  # 可选参数
        self.y_minor_grid_step = y_minor_grid_step  # 可选参数


class RobotVisualizer(Node):
    def __init__(self,  num_joints=6,limits=None,windows=None, display_duration=None,show_type='time', show_max_points=1000,window_size=5.0):
        super().__init__('robot_visualizer')
        self.num_joints = num_joints
        self.limits = limits
        self.show_max_points = show_max_points
        self.max_history = 100000  # 限制历史数据的长度
        self.show_type = show_type
        self.init_time_stamp = None

        self.data_path = os.path.join(current_path, '../data')
        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)
        
        self.save_plot_dir = os.path.join(self.data_path, 'saved_plots')
        if not os.path.exists(self.save_plot_dir):
            os.makedirs(self.save_plot_dir)
        self.save_data_dir = os.path.join(self.data_path, "saved_datas")
        if not os.path.exists(self.save_data_dir):
            os.makedirs(self.save_data_dir)



        # 处理要显示的关节索引
        # if joint_indices is None:
        #     self.joint_indices = list(range(self.num_joints))
        # else:
        #     self.joint_indices = joint_indices




        self.display_duration = display_duration  # 显示时长
        self.window_size = window_size  # 时间窗口大小
        self.start_time = None  # 开始时间，将在接收到第一个数据时设置
        self.init_windows(windows)

        self.create_subscription(RobotStatus, '/robot_arm/state', self.joint_state_callback, 10)




        self.joint_history_info = {}
        self.joint_history_info['time_history'] = []
        self.joint_history_info['positions'] = [[] for _ in range(num_joints)]
        self.joint_history_info['velocities'] = [[] for _ in range(num_joints)]
        self.joint_history_info['accelerations'] = [[] for _ in range(num_joints)]
        self.joint_history_info['joint_torques'] = [[] for _ in range(num_joints)]
        self.joint_history_info['forces'] = {'Fx': [], 'Fy': [], 'Fz': []}
        self.joint_history_info['torques'] = {'Mx': [], 'My': [], 'Mz': []}


        # 创建 Matplotlib 绘图
        self.initialize_plots()

        # # 启动动画更新
        # self.ani = animation.FuncAnimation(self.fig, self.update_plots, interval=10)

        # 绑定键盘事件，用于保存绘图
        for fig in self.figures:
            fig.canvas.mpl_connect('key_press_event', self.on_key_press)


        # 启动动画更新
        for i, fig in enumerate(self.figures):
            window = self.windows[i]
            ani = animation.FuncAnimation(fig, self.create_update_plots(window), interval=50)
            self.ani_list.append(ani)
            # plt.draw()


        # 启动线程来运行 ROS 2 接收数据
        self.ros_thread = threading.Thread(target=self.spin_ros)
        self.ros_thread.daemon = True  # 使线程为守护线程
        self.ros_thread.start()

        # plt.ion()  # 打开交互模式
        plt.show(block=True)
        
        
        
    def spin_ros(self):
        """ ROS 2 节点的 spin 方法，用于接收消息 """
        rclpy.spin(self)  # 阻塞式循环，处理订阅的数据



    def init_windows(self,windows):
        if windows is None:
            # 默认情况下，创建一个窗口，显示所有数据类型和所有关节
            self.windows = [WindowConfig(window_id=1, data_types=['position', 'velocity', 'acceleration'],
                                         joint_indices=list(range(self.num_joints)))]
        else:
            self.windows = windows
        # 为每个窗口创建绘图
        self.figures = []
        self.axes_list = []
        self.lines_dict = {}  # 存储每个窗口的绘图线条
        self.ani_list = []
        self.vlines_dict = {}  # 存储每个窗口的竖线
        self.limit_lines_dict = {}  # 存储每个窗口的限界线
        pass


    def initialize_plots(self):

        colors = ['r', 'g', 'b', 'orange', 'purple', 'cyan']

        for window in self.windows:
            num_subplots = len(window.data_types)
            fig, axes = plt.subplots(num_subplots, 1, figsize=window.fig_size)
            fig.suptitle(f"Robot Joint Data Visualization - Window {window.window_id}:{window.window_name}")
            self.figures.append(fig)

            # 将 axes 转换为一维的列表
            axes = np.atleast_1d(axes).flatten().tolist()
            self.axes_list.append(axes)

            lines = {}  # 存储当前窗口的线条
            for idx, data_type in enumerate(window.data_types):
                ax = axes[idx]
                ax.set_title(f"{data_type.capitalize()}")

                lines[data_type] = []
                if window.joint_indices:
                    # 关节相关的数据类型
                    for joint_idx in window.joint_indices:
                        color = colors[joint_idx % len(colors)]
                        line, = ax.plot([], [], color=color, label=f'Joint {joint_idx + 1}')
                        lines[data_type].append(line)
                else:
                    # 全局数据类型，如 forces 和 torques
                    if data_type == 'forces':
                        # 绘制 Fx, Fy, Fz
                        for comp, color in zip(['Fx', 'Fy', 'Fz'], ['r', 'g', 'b']):
                            line, = ax.plot([], [], color=color, label=comp)
                            lines[data_type].append(line)
                    elif data_type == 'torques':
                        # 绘制 Mx, My, Mz
                        for comp, color in zip(['Mx', 'My', 'Mz'], ['c', 'm', 'y']):
                            line, = ax.plot([], [], color=color, label=comp)
                            lines[data_type].append(line)

                ax.legend(loc='upper right', bbox_to_anchor=(1, 1))
                ax.grid(True)

                # 应用刻度配置
                if window.xticks is not None:
                    ax.set_xticks(window.xticks)
                if window.yticks is not None:
                    ax.set_yticks(window.yticks)

                # 应用网格密度配置
                # 应用网格密度配置
                if window.x_major_grid_step is not None:
                    ax.xaxis.set_major_locator(ticker.MultipleLocator(window.x_major_grid_step))
                if window.x_minor_grid_step is not None:
                    ax.xaxis.set_minor_locator(ticker.MultipleLocator(window.x_minor_grid_step))

                if window.y_major_grid_step is not None:
                    ax.yaxis.set_major_locator(ticker.MultipleLocator(window.y_major_grid_step))
                if window.y_minor_grid_step is not None:
                    ax.yaxis.set_minor_locator(ticker.MultipleLocator(window.y_minor_grid_step))

                # 启用主网格和次网格
                ax.grid(which='major', linestyle='-', linewidth=0.75)
                ax.grid(which='minor', linestyle=':', linewidth=0.5)

                if self.show_type == 'time':
                    ax.set_xlim(-self.window_size * 0.8, self.window_size * 0.2)
                    ax.autoscale(axis='y')

                    # 添加竖线
                    vline = ax.axvline(x=0, color='k', linestyle='--')
                    if window.window_id not in self.vlines_dict:
                        self.vlines_dict[window.window_id] = []
                    self.vlines_dict[window.window_id].append(vline)

                    # 添加限界线
                    limit_lines = []
                    if window.joint_indices:
                        for joint_idx in window.joint_indices:
                            # 对每个关节和数据类型添加限界线
                            lines_added = self.add_boundary(ax,data_type, joint_idx,colors)
                            limit_lines.extend(lines_added)
                    # limit_lines = self.add_boundary_new(ax,data_type)

                    if window.window_id not in self.limit_lines_dict:
                        self.limit_lines_dict[window.window_id] = []
                    self.limit_lines_dict[window.window_id].extend(limit_lines)
                else:
                    # 添加限界线
                    limit_lines = []
                    for joint_idx in window.joint_indices:
                        # 对每个关节和数据类型添加限界线
                        lines_added = self.add_boundary(ax, data_type, joint_idx, colors)
                        limit_lines.extend(lines_added)
                    # limit_lines = self.add_boundary_new(ax,data_type)

                    if window.window_id not in self.limit_lines_dict:
                        self.limit_lines_dict[window.window_id] = []
                    self.limit_lines_dict[window.window_id].extend(limit_lines)


            self.lines_dict[window.window_id] = lines

            # # 为每个窗口创建对应的动画更新函数
            # ani = animation.FuncAnimation(fig, self.create_update_plots(window), interval=50)
            # self.ani_list.append(ani)

    def add_boundary(self,ax,data_type,joint_idx,colors):
        # === 添加上下界限线 ===
        limit_lines = []
        # 使用默认限界值
        default_limits = {
            'positions': {'upper': 360, 'lower': -360},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 200, 'lower': -200}
        }
        # 获取当前关节的数据类型限制
        try:
            if self.limits is None:
                upper_limit = default_limits.get(data_type, {}).get('upper', 360)
                lower_limit = default_limits.get(data_type, {}).get('lower', -360)
                # 添加限界线
                limit_upper = ax.axhline(y=upper_limit, color='black', linestyle='--', linewidth=0.8)
                limit_lower = ax.axhline(y=lower_limit, color='black', linestyle='--', linewidth=0.8)
                limit_lines.extend([limit_upper, limit_lower])
            else:
                limits = self.limits[joint_idx][data_type]
                upper_limit = limits['upper']
                lower_limit = limits['lower']

                # 添加限界线
                limit_upper = ax.axhline(y=upper_limit, color=colors[joint_idx], linestyle='--', linewidth=0.8)
                limit_lower = ax.axhline(y=lower_limit, color=colors[joint_idx], linestyle='--', linewidth=0.8)
                limit_lines.extend([limit_upper, limit_lower])
        except Exception:
            print(
                f"Warning: No limits defined for joint {joint_idx} and data type '{data_type}'. Using default limits.")

            upper_limit = default_limits.get(data_type, {}).get('upper', 360)
            lower_limit = default_limits.get(data_type, {}).get('lower', -360)
            # 添加限界线
            limit_upper = ax.axhline(y=upper_limit, color='black', linestyle='--', linewidth=0.8)
            limit_lower = ax.axhline(y=lower_limit, color='black', linestyle='--', linewidth=0.8)
            limit_lines.extend([limit_upper, limit_lower])



        return limit_lines
        pass

    def on_key_press(self, event):
        """
        键盘按键事件处理函数。
        按下 's' 键时保存当前所有窗口的绘图。
        """
        if event.key == 'a':
            self.save_all_window_plots()

    def save_all_window_plots(self):
        """
        保存所有窗口当前的绘图为图片文件。
        文件名格式为: window_{window_id}_{timestamp}.png
        """
        timestamp = int(time.time())

        self.plot_all_window_data_separately()
        # for window in self.windows:
        #     fig = self.figures[window.window_id - 1]  # 假设 window_id 从1开始
        #     save_name = os.path.join(self.save_plot_dir, f"window_{window.window_id}_{timestamp}.png")
        #     fig.savefig(save_name)
        #     print(f"窗口 {window.window_id} 的数据已保存至 {save_name}")

    def joint_state_callback(self, msg:RobotStatus):
        """ROS 2 回调函数：接收 JointState 消息并更新历史数据"""
        try:
            # 在此处更新关节数据
            time_stamp = msg.joint_state.header.stamp.sec+msg.joint_state.header.stamp.nanosec*1e-9
            self.joint_history_info['time_history'].append(time_stamp)
            print(time_stamp)

            # 遍历所有关节
            for i in range(self.num_joints):
                # 处理 position 数据
                position = msg.joint_state.position[i] if i < len(msg.joint_state.position) else None
                velocity = msg.joint_state.velocity[i] if i < len(msg.joint_state.velocity) else None
                # acceleration = msg.joint_acceleration[i] if i < len(msg.acceleration) else None
                acceleration = None
                effort = msg.joint_state.effort[i] if i < len(msg.joint_state.effort) else None
                # print(f"joint {i} position: {position}, velocity: {velocity}, effort: {effort}")

                # 检查是否缺失数据，如果缺失则填充默认值
                if position is None:
                    position = 0.0  # 设置默认值（例如：0.0）
                if velocity is None:
                    velocity = 0.0  # 设置默认值（例如：0.0）
                
                if effort is None:
                    effort = 0.0  # 设置默认值（例如：0.0）

                # 更新历史数据
                self.joint_history_info['positions'][i].append(position)
                self.joint_history_info['velocities'][i].append(velocity)
                self.joint_history_info['joint_torques'][i].append(effort) 
                if acceleration is None:
                    # 计算加速度
                    self.joint_history_info['accelerations'][i].append(self.joint_history_info['velocities'][i][-1] - self.joint_history_info['velocities'][i][-2] if len(self.joint_history_info['velocities'][i]) > 1 else 0.0)

                else:
                    self.joint_history_info['accelerations'][i].append(acceleration)

            # 限制历史数据长度
            if len(self.joint_history_info['time_history']) > self.max_history:
                self.joint_history_info['time_history'] = self.joint_history_info['time_history'][-self.max_history:]
                for i in range(self.num_joints):
                    self.joint_history_info['positions'][i] = self.joint_history_info['positions'][i][-self.max_history:]
                    self.joint_history_info['velocities'][i] = self.joint_history_info['velocities'][i][-self.max_history:]
                    # self.joint_history_info['accelerations'][i] = self.joint_history_info['accelerations'][i][-self.max_history:]
                    self.joint_history_info['joint_torques'][i] = self.joint_history_info['joint_torques'][i][-self.max_history:]
        except Exception as e:
            self.get_logger().error(f"Error processing JointState message: {e}")


    def create_update_plots(self, window):
        def update(frame):
            try:

                if not self.joint_history_info['time_history']:
                    return

                # 获取当前时间和时间窗口
                t, t_min, t_max, t_window,total_window = self._get_time_window()

                # 获取绘图元素
                lines = self.lines_dict[window.window_id]
                axes = self.axes_list[self.windows.index(window)]

                # 更新每个数据类型的线条
                for idx, data_type in enumerate(window.data_types):
                    ax = axes[idx]
                    data_lines = lines[data_type]
                    if window.joint_indices:
                        # 关节相关的数据类型
                        self._update_data_lines(window, ax, data_type, data_lines, t_window, t_min, t_max)
                    else:
                        # 全局数据类型
                        if data_type == 'forces':
                            self._update_global_data(window, ax, data_type, data_lines, 'forces', t_window)
                        elif data_type == 'torques':
                            self._update_global_data(window, ax, data_type, data_lines, 'torques', t_window)

                    if self.show_type == 'time':
                        # 更新 x 轴范围
                        self._update_vlines(window,ax, t_min, t_max,total_window)
                    else:
                        # 更新 x 轴范围
                        ax.relim()
                        ax.autoscale_view()

            except Exception as e:
                print(f"Error updating plots: {e}")

        return update

    def _get_time_window(self):
        """
        计算当前时间窗口的起始时间和结束时间，并返回相对时间窗口的数据。
        """

        stamp = np.array(self.joint_history_info['time_history'])
        if self.start_time is None:
            # 尚未接收到数据，设置初始的 start_time
            self.start_time = 0.0  # 或者设置为当前时间 time.time()
            current_time = self.start_time
            self.init_time_stamp = stamp[-1]
            t = stamp - self.init_time_stamp
            print(f"init_time_stamp: {self.init_time_stamp}")
        else:
            # 获取当前时间
            t = stamp - self.init_time_stamp
            current_time = t[-1] if len(t) > 0 else self.start_time


        if self.show_type == 'time':
            total_window = self.window_size
            t_max = current_time + total_window * 0.2
            t_min = t_max - total_window


            # 获取时间窗口内的数据索引
            indices = np.where((t >= t_min) & (t <= t_max))[0]
            t_window = t[indices]
        else:
            t_min = 0
            t_max = self.window_size
            t_window = t
            total_window=None
            indices = slice(None)

        return t, t_min, t_max, t_window, total_window

    def _update_data_lines(self, window, ax, data_type, data_lines, t_window, t_min, t_max):
        """
        更新特定数据类型的所有关节数据线条，并调整坐标轴范围。
        """
        for line_idx, line in enumerate(data_lines):
            joint_idx = self.windows[self.windows.index(window)].joint_indices[line_idx]
            if self.joint_history_info[data_type]:
                
                y_data = np.array(self.joint_history_info[data_type][joint_idx])
                y_data_window = y_data[-len(t_window):]  # 确保数据长度匹配
                line.set_data(t_window, np.degrees(y_data_window))
            else:
                y_data = np.zeros_like(t_window)
                # y_data_window = y_data[-len(t_window):]
                line.set_data(t_window, y_data_window)

        # 调整纵坐标范围，仅基于数据线
        y_data = np.concatenate([line.get_ydata() for line in data_lines])
        if y_data.size > 0:
            y_min = np.min(y_data)
            y_max = np.max(y_data)
            y_range = y_max - y_min
            y_range = y_range if y_range != 0 else 1  # 防止除以零
            y_pad = 0.1 * y_range
            ax.set_ylim(y_min - y_pad, y_max + y_pad)

    def _update_vlines(self, window,ax, t_min, t_max,total_window):
        """
        更新竖线的位置，固定在x轴范围的80%处。
        """
        ax.set_xlim(t_min, t_max)

        # 更新竖线的位置，固定在 x 轴范围的 80% 处
        vline_position = t_min + total_window * 0.8  # 竖线位置
        vlines = self.vlines_dict.get(window.window_id, [])
        for vline in vlines:
            vline.set_xdata([vline_position, vline_position])

    def _update_global_data(self,window, ax, data_type, data_lines, data_key, t_window):
        """
        更新全局数据（force 或 torque）的所有分量线条，并调整坐标轴范围。

        :param ax: Matplotlib Axes 对象
        :param data_type: 'forces' 或 'torques'
        :param data_lines: 当前窗口的线条列表
        :param data_key: 'force' 或 'torque'
        :param t_window: 当前时间窗口的数据
        """
        keys = []
        if data_type == 'forces':
            keys = ['Fx', 'Fy', 'Fz']
        elif data_type == 'torques':
            keys = ['Mx', 'My', 'Mz']

        for line_idx, line in enumerate(data_lines):
            comp = keys[line_idx]
            y_data = np.array(self.joint_history_info[data_key][comp])
            y_data_window = y_data[-len(t_window):]  # 确保数据长度匹配
            line.set_data(t_window, y_data_window)  # 假设力和力矩已经是度量单位，无需转换

        # 调整纵坐标范围，仅基于数据线
        y_data = np.concatenate([line.get_ydata() for line in data_lines])
        if y_data.size > 0:
            y_min = np.min(y_data)
            y_max = np.max(y_data)
            y_range = y_max - y_min
            y_range = y_range if y_range != 0 else 1  # 防止除以零
            y_pad = 0.1 * y_range
            ax.set_ylim(y_min - y_pad, y_max + y_pad)



    def plot_all_window_data_separately(self, file_prefix='window'):
        """
        另一个方法，按窗口分别绘制并保存历史数据。
        文件名格式为: {file_prefix}_{window_id}_{timestamp}.png
        """
        timestamp = int(time.time())
        for window in self.windows:
            fig, axes = plt.subplots(len(window.data_types), 1, figsize=(10, 8))
            fig.suptitle(f"Robot Joint Data Visualization - Window {window.window_id}:{window.window_name} - {time.ctime(timestamp)}")

            # 将 axes 转换为一维的列表
            axes = np.atleast_1d(axes).flatten().tolist()

            for idx, data_type in enumerate(window.data_types):
                ax = axes[idx]
                ax.set_title(f"{data_type.capitalize()}")

                if data_type in ['positions', 'velocities', 'accelerations']:
                    for joint_idx in window.joint_indices:
                        y_data = np.array(self.joint_history_info[data_type][joint_idx])
                        x_data = np.array(self.joint_history_info['time_history'])
                        ax.plot(x_data, np.degrees(y_data), label=f'Joint {joint_idx + 1}')
                elif data_type == 'forces' or data_type=='torques':
                    # 力分量
                    force_components = ['Fx', 'Fy', 'Fz']
                    force_colors = ['r', 'g', 'b']
                    # 力矩分量
                    torque_components = ['Mx', 'My', 'Mz']
                    torque_colors = ['c', 'm', 'y']
                    for comp,c in zip(force_components,force_colors):
                        y_data = np.array(self.joint_history_info['forces'][comp])
                        x_data = np.array(self.joint_history_info['time_history'])
                        ax.plot(x_data, y_data, color=c, label=comp)
                    for comp,c in zip(torque_components,torque_colors):
                        y_data = np.array(self.joint_history_info['torques'][comp])
                        x_data = np.array(self.joint_history_info['time_history'])
                        ax.plot(x_data, y_data, color=c, label=comp)

                ax.legend(loc='upper right', bbox_to_anchor=(1, 1))
                ax.grid(True)

            fig.tight_layout()
            fig.subplots_adjust(top=0.95)
            date = time.strftime('%Y年%m月%d日%Hh%Mm%Ss', time.localtime())
            save_data_name = os.path.join(self.save_data_dir, f"{file_prefix}_{window.window_id}_{date}")
            np.save(save_data_name,np.array(self.joint_history_info))
            save_plot_name = os.path.join(self.save_plot_dir, f"{file_prefix}_{window.window_id}_{date}.png")
            fig.savefig(save_plot_name)
            plt.close(fig)  # 关闭图形以释放内存
            print(f"窗口 {window.window_id} 的历史数据已保存至 {save_plot_name}")

    def close(self):
        print('close')
        self.running = False
        
joint_limits = {
        0: {'positions': {'upper': 360, 'lower': -360},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 400, 'lower': -400}},
        1: {'positions': {'upper': 125, 'lower': -125},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 400, 'lower': -400}},
        2: {'positions': {'upper': 130, 'lower': -130},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 400, 'lower': -400}},
        3: {'positions': {'upper': 360, 'lower': -360},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 400, 'lower': -400}},
        4: {'positions': {'upper': 120, 'lower': -120},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 400, 'lower': -400}},
        5: {'positions': {'upper': 360, 'lower': -360},
            'velocities': {'upper': 180, 'lower': -180},
            'accelerations': {'upper': 400, 'lower': -400}},
        # 其他关节...
    }
def main():
    rclpy.init()  # 初始化 rclpy

    # visualizer1 = RobotVisualizer(joint_indices=[5],display_duration=None)
    window1 = WindowConfig(window_id=1, data_types=['positions', 'velocities','accelerations'], joint_indices=[0, 3],fig_size=(5,6),x_minor_grid_step=0.1)
    # window2 = WindowConfig(window_id=2, data_types=['accelerations'], joint_indices=[2])
    window3 = WindowConfig(window_id=3, data_types=['forces', ], joint_indices=None,fig_size=(5,4),xticks=[0,1,2,3],x_minor_grid_step=0.1)
    visualizer = RobotVisualizer(num_joints=6,limits=joint_limits, windows=[window1,], display_duration=None,)

    # 之后你的程序逻辑
    # rclpy.spin(visualizer)  # 如果这是 ROS2 节点，需要调用 spin

    rclpy.shutdown()  # 最后关闭 rclpy

if __name__ == '__main__':
    main()
