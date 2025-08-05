import tkinter as tk
from tkinter import *
from tkinter import messagebox
from tkinter import scrolledtext
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import threading
import time
import subprocess
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion,Twist
import sys



class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        self.torque_pub = self.create_publisher(Float64, 'motor_torque', 10)
        self.angle_sub = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.set_state = self.create_client(SetEntityState,'/plug/set_entity_state')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = None
        self.start_time = 0
        self.position = 0.0
        self.prev_position = 0.0
        self.torque_max = 1.5
        self.const_torque = 0.0
        self.kp = 1.2
        self.ki = 0.0
        self.ki_max = 0.6
        self.kd = 1.0
        self.sum_dTheta = 0.0
        self.theta_set = 0.256
        self.theta_init = 0.256 
        self.sim_time = 0.0
        self.prev_sim_time = 0.0
        self.init_time = 0.0
        self.delta_time = 0.0
        self.startFlag = False
        self.Angle = []
        self.Time = []
        self.all_trajectories = []
        self.current_trajectory = {'time':[],'angle':[]}

        self.x=0.0
        self.y=0.0
        self.z=0.0
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0

        self.table_amplitude=0.0
        self.table_frequence=0.0

        self.set_self_state = EntityState()
        self.set_self_state.name = 'spherorobot_cpp'
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        self.window = Tk()
        self.window.geometry('900x600')

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)



        self.box_kp = Entry(self.window,width=10)
        self.box_kp.place(x=50,y=20)
        self.box_kp.insert(0,'1.0')
        self.txt_kp = Label(self.window,text = 'kp - ')
        self.txt_kp.place(x=20,y=20)

        self.box_ki = Entry(self.window,width=10)
        self.box_ki.place(x=50,y=50)
        self.box_ki.insert(0,'0.05')
        self.txt_ki = Label(self.window,text = 'ki - ')
        self.txt_ki.place(x=20,y=50)

        self.box_kd = Entry(self.window,width=10)
        self.box_kd.place(x=50,y=80)
        self.box_kd.insert(0,'0.02')
        self.txt_kd = Label(self.window,text = 'kd - ')
        self.txt_kd.place(x=20,y=80)

        self.constTorqueFlag = tk.IntVar()
        self.check_constTorque = tk.Checkbutton(self.window,text='постоянный момент',variable=self.constTorqueFlag)
        self.check_constTorque.place(x=10,y=110)

        self.box_constTorque = Entry(self.window,width=9)
        self.box_constTorque.place(x=60,y=140)
        self.box_constTorque.insert(0,'0.2')
        self.txt_constTorque = Label(self.window,text = 'Mн - ')
        self.txt_constTorque.place(x=20,y=140)

        self.lqrFlag = tk.IntVar()
        self.check_lqr = tk.Checkbutton(self.window,text='включить стабилизацию',variable=self.lqrFlag)
        self.check_lqr.place(x=10,y=170)

        self.box_angle_init = Entry(self.window,width=10)
        self.box_angle_init.place(x=50,y=200)
        self.box_angle_init.insert(0,'0.0')
        self.txt_angle_init = Label(self.window,text='\u03B8\u2080 -')
        self.txt_angle_init.place(x=20,y=200)

        self.box_angle_set = Entry(self.window,width=10)
        self.box_angle_set.place(x=50,y=230)
        self.box_angle_set.insert(0,'0.1')
        self.txt_angle_set = Label(self.window,text='\u03B8\u209B -')
        self.txt_angle_set.place(x=20,y=230)

        self.start_button = Button(self.window,text='Start',width=10,command=self.Start_button)
        self.start_button.place(x=20,y=260)

        self.controlTorqueFlag = tk.IntVar()
        self.check_controlTorque = tk.Checkbutton(self.window,text='En',variable=self.controlTorqueFlag)
        self.check_controlTorque.place(x=170,y=260)

        self.stop_button = Button(self.window,text='Stop',width=10,command=self.Stop_button)
        self.stop_button.place(x=20,y=290)
        self.clear_button = Button(self.window,text='clear',width=10,command=self.Clear_button)
        self.clear_button.place(x=20,y=320)

        self.box_x = Entry(self.window,width=5)
        self.box_x.place(x=40,y=360)
        self.box_x.insert(0,'0.0')
        self.txt_x = Label(self.window,text = 'x-')
        self.txt_x.place(x=20,y=360)

        self.box_y = Entry(self.window,width=5)
        self.box_y.place(x=110,y=360)
        self.box_y.insert(0,'0.0')
        self.txt_y = Label(self.window,text = 'y-')
        self.txt_y.place(x=90,y=360)

        self.box_z = Entry(self.window,width=5)
        self.box_z.place(x=180,y=360)
        self.box_z.insert(0,'0.05')
        self.txt_z = Label(self.window,text = 'z-')
        self.txt_z.place(x=160,y=360)

        self.box_roll = Entry(self.window,width=5)
        self.box_roll.place(x=40,y=390)
        self.box_roll.insert(0,'0.0')
        self.txt_roll = Label(self.window,text = 'R-')
        self.txt_roll.place(x=20,y=390)

        self.box_pitch = Entry(self.window,width=5)
        self.box_pitch.place(x=110,y=390)
        self.box_pitch.insert(0,'0.0')
        self.txt_pitch = Label(self.window,text = 'P-')
        self.txt_pitch.place(x=90,y=390)

        self.box_yaw = Entry(self.window,width=5)
        self.box_yaw.place(x=180,y=390)
        self.box_yaw.insert(0,'0.0')
        self.txt_yaw = Label(self.window,text = 'Y-')
        self.txt_yaw.place(x=160,y=390)

        self.txt_table = Label(self.window,text = 'Настройки стола')
        self.txt_table.place(x=20,y=420)

        self.box_ampl = Entry(self.window,width=5)
        self.box_ampl.place(x=40,y=450)
        self.box_ampl.insert(0,'1.0')
        self.txt_ampl = Label(self.window,text = 'A-')
        self.txt_ampl.place(x=20,y=450)

        self.box_freq = Entry(self.window,width=5)
        self.box_freq.place(x=110,y=450)
        self.box_freq.insert(0,'10.0')
        self.txt_freq = Label(self.window,text = 'F-')
        self.txt_freq.place(x=90,y=450)

        self.start_button_table = Button(self.window,text='Start',width=6,command=self.start_vibration)
        self.start_button_table.place(x=20,y=480)
        self.stop_button_table = Button(self.window,text='Stop',width=6,command=self.stop_vibration)
        self.stop_button_table.place(x=100,y=480)

        self.txt_discret = Label(self.window,text = 'Период дискретизации системы')
        self.txt_discret.place(x=20,y=510)

        self.box_discret = Entry(self.window,width=5)
        self.box_discret.place(x=40,y=530)
        self.box_discret.insert(0,'0.05')
        self.txt_discret = Label(self.window,text = 'T-')
        self.txt_discret.place(x=20,y=530)

        self.graph_frame=tk.Frame(self.window,bg="lightgray")
        self.graph_frame.place(x=250,y=20)
        self.fig=Figure(figsize=(6,4),dpi=100)
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        
        self.controlArr = 0.1*np.cos(np.pi*np.arange(0, 2, 0.05))
        self.numControlArr = 1


        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
    
    def on_closing(self):
        if tk.messagebox.askokcancel("Выход", "Закрыть приложение?"):
            self.window.destroy()
            sys.exit(0)

    def joint_states_callback(self,msg):
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.position = msg.velocity[0]
        # self.get_logger().info(f"Pendulum_angle: {self.position}")

    def robot_teleport(self,x=0.0,y=0.0,z=0.0,roll=0.0,pitch=0.0,yaw=0.0):
        object_state = self.set_self_state
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = Quaternion()
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        q.w = cy * cp * cr + sy * sp * sr

        object_state.pose.position = Point(x=x, y=y, z=z)
        object_state.pose.orientation = q
        state = SetEntityState.Request()
        state._state = object_state

        try:
            self.set_state.call_async(state)
        except rclpy.ServiceException as e:
            print("plug/set_entity_state service call failed")
    
    def Start_button(self):
        # subprocess.run('ros2 service call /reset_world std_srvs/Empty' ,shell=True)
        self.delta_time = float(self.box_discret.get())
        self.x=float(self.box_x.get())
        self.y=float(self.box_y.get())
        self.z=float(self.box_z.get())
        self.roll=float(self.box_roll.get())
        self.pitch=float(self.box_pitch.get())
        self.yaw=float(self.box_yaw.get())
        #self.robot_teleport(x=self.x, y=self.y, z=self.z,roll=self.roll,pitch=self.pitch,yaw=self.yaw)
        #self.get_logger().info(f"Робот перемещен в начальное положение")
        self.kp = float(self.box_kp.get())
        self.ki = float(self.box_ki.get())
        self.kd = float(self.box_kd.get())
        self.const_torque = float(self.box_constTorque.get())
        self.theta_init = float(self.box_angle_init.get())
        self.theta_set = float(self.box_angle_set.get())
        self.init_time = self.sim_time
        self.startFlag = True
        self.sum_dTheta = 0.0
        self.current_trajectory = {'time':[],'angle':[]}

    def Stop_button(self):
        self.startFlag = False
        self.torque = 0.0
        msg = Float64()
        msg.data = float(self.torque)
        self.torque_pub.publish(msg)
        self.get_logger().info(f"Publishing torque: {self.torque}")
        if len(self.current_trajectory['time']) > 0:
                self.all_trajectories.append(self.current_trajectory)
                self.ax.clear()
                for i, traj in enumerate(self.all_trajectories):
                    color = plt.cm.viridis(i / len(self.all_trajectories))  # Разные цвета
                    self.ax.plot(traj['time'], traj['angle'], 
                                label=f'Run {i+1}', 
                                color=color,
                                linewidth=2)
                
                # Настройки графика
                self.ax.set_xlabel('Time (s)')
                self.ax.set_ylabel('Angle (rad)')
                self.ax.grid(True)
                self.ax.legend()
                self.canvas.draw()

    def Clear_button(self):
        self.all_trajectories = []
        self.ax.clear()
        self.ax.grid(True)
        self.canvas.draw()

    def start_vibration(self):
        """Запуск вибрации с заданной частотой и амплитудой"""
        self.table_amplitude=float(self.box_ampl.get())
        self.table_frequence=float(self.box_freq.get())
        if self.timer:
            self.timer.cancel()
            
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        period_sec = 1.0 / (2 * self.table_frequence)  # Обновляем 2 раза за период
        
        self.timer = self.create_timer(
            period_sec,
            lambda: self._publish_speed())
    
    def _publish_speed(self):
        self.table_amplitude *= (-1)
        
        msg = Twist()
        msg.linear.x = self.table_amplitude
        self.pub.publish(msg)
        
    def stop_vibration(self):
        """Остановка вибрации"""
        if self.timer:
            self.timer.cancel()
            self.timer = None
        
        stop_msg = Twist()
        self.pub.publish(stop_msg)

    def control_loop(self):
        self.lengthControlArr=len(self.controlArr)
        index = 0
        m = 0.148
        rho = 0.024717
        g = 9.81

        while rclpy.ok():
            if self.startFlag:
                self.torque = 0.0
                if self.sim_time-self.prev_sim_time>=self.delta_time:
                    if self.lqrFlag.get() != 0:
                        if self.controlTorqueFlag.get() !=0:
                            self.theta_set = self.controlArr[index % self.lengthControlArr]
                            index += 1                            
                       
                        self.sum_dTheta += self.position-self.theta_set
                        self.Ki = np.clip(self.ki*self.sum_dTheta,-self.ki_max,self.ki_max)
                        self.feedback = (m*g*rho*np.sin(self.theta_set-self.theta_init))
                        self.torque = self.kp*(self.position-self.theta_set)+self.kd*((self.position-self.prev_position)/(self.sim_time-self.prev_sim_time))+self.Ki+self.feedback
                        self.get_logger().info(f"Ki: {self.Ki}")
                    if self.constTorqueFlag.get() !=0:
                        self.torque = self.const_torque
                    
                    self.torque = np.clip(self.torque,-self.torque_max,self.torque_max)
                    self.get_logger().info(f"dT: {self.sim_time-self.prev_sim_time}")
                    self.current_trajectory['time'].append(self.sim_time-self.init_time)
                    self.current_trajectory['angle'].append(self.position)
                    self.get_logger().info(f"pend_angle: {self.position}")
                    self.get_logger().info(f"time: {self.sim_time-self.init_time}")
                    self.prev_sim_time = self.sim_time
                    self.prev_position = self.position

                    msg = Float64()
                    msg.data = float(self.torque)
                    self.torque_pub.publish(msg)
                    self.get_logger().info(f"Publishing torque: {self.torque}")
                    #time.sleep(0.05)


def main():
    rclpy.init()
    ros_node = LQRController()
    ros_thread = threading.Thread(target=rclpy.spin,args=(ros_node,), daemon=True)
    ros_thread.start()

    ros_node.window.mainloop()
    
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

