#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  
from rclpy.node import Node  
from std_msgs.msg import Float64  # Тип сообщения для чисел с плавающей точкой
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState  # Тип сообщения для состояния сочленений
from sphero_python.sphero_dynamics import SpheroDynamics
import numpy as np
from scipy.integrate import quad
from scipy.interpolate import CubicSpline
import scipy.io as sio
from scipy.interpolate import make_interp_spline
import time

class TransversalController(Node):
    
    def __init__(self):
        super().__init__('transversal_controller')

        # publishers
        self.pendulum_torque_pub = self.create_publisher(Float64, 'pendulum_torque', 10)
        self.rotor_torque_pub = self.create_publisher(Float64, 'rotor_torque', 10)
        self.transversal_coords_pub = self.create_publisher(Float64MultiArray, 'transversal_coords', 10)
        
        # timer for control loop 
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.publish_commands)
        self.get_logger().info(f'Таймер запущен, публикация каждые {timer_period} секунд.')

        # subscriber
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            'sphero_states',
            self.joint_states_callback,
            1  # Размер очереди
        )
        self.get_logger().info("Подписчик на топик 'sphero_states' создан.")

        self.spheroPosX = 0.0
        self.spheroVelX = 0.0
        self.pendulumAng = 0.0
        self.pendulumAngVel = 0.0

        self.dynamics = SpheroDynamics()

        self.T_per, self.x_of_tau_func, self.dx_of_tau_func, self.tht_of_tau_func, self.dtht_of_tau_func, self.tau_of_x_func = self.GetXStarSplines()
        self.k1_func, self.k2_func, self.k3_func = self.GetKSplines()

        start = time.perf_counter()
        self.IntegralNum = self.GetIntegralSpline(self.GetXStarFromTau(0), self.GetdXStarFromTau(0))
        end = time.perf_counter()
        print(f"Время: {end - start:.6f} сек")

        self.numberOfIter = 0

        

    def joint_states_callback(self, msg: JointState):
        self.spheroPosX = msg.position[1]
        self.spheroVelX = msg.velocity[1]
        self.pendulumAng = np.arctan2 (np.sin(msg.position[0]), np.cos(msg.position[0]))
        self.pendulumAngVel = msg.velocity[0]

    def GetXStarSplines(self):
        data_star = sio.loadmat('/home/ros/ros2_ws/src/sphero_python/sphero_python/x_star.mat')
        T = data_star['T']
        x = data_star['S']        
        dx = data_star['V']

        tht = 0.001*np.cos(x)
        dtht = -0.001 * np.sin(x)*dx

        T_per = T[-1]

        x_of_tau = make_interp_spline(T[:,0], x[:,0], k=3) # bc_type='periodic'
        dx_of_tau = make_interp_spline(T[:,0], dx[:,0], k=3)
        
        tht_of_tau = make_interp_spline(T[:,0], tht[:,0], k=3) # bc_type='periodic'
        dtht_of_tau = make_interp_spline(T[:,0], dtht[:,0], k=3)

        tau_of_x = make_interp_spline(x[:,0], T[:,0], k=3)


        return T_per, x_of_tau, dx_of_tau, tht_of_tau, dtht_of_tau, tau_of_x
    
    def GetKSplines(self):
        data = sio.loadmat('/home/ros/ros2_ws/src/sphero_python/sphero_python/data_K.mat')
        K_stab = data['K_stab']

        t_k = np.arange(0.0, self.T_per+0.01, 0.01)
        k1 = make_interp_spline(t_k, K_stab[:,0], k=3,bc_type='periodic')
        k2 = make_interp_spline(t_k, K_stab[:,1], k=3,bc_type='periodic')
        k3 = make_interp_spline(t_k, K_stab[:,2], k=3,bc_type='periodic')

        return k1, k2, k3



    def ModPeriodX(self, x):
        period = 2*np.pi
        return x % period

    def GetThtStarFromTau(self, tau):
        return self.tht_of_tau_func(tau)
    
    def GetdThtStarFromTau(self, tau):
        return self.dtht_of_tau_func(tau)
    
    def GetXStarFromTau(self, tau):
        return self.x_of_tau_func(tau)
        
    def GetdXStarFromTau(self, tau):        
        return self.dx_of_tau_func(tau)
    
    def GetTauFromXStar(self, x):
        return self.tau_of_x_func(x)

    def GetTau(self, x):
        xMod = self.ModPeriodX(x)
        tau = self.GetTauFromXStar(xMod)
        return tau

    def GetK(self, tau):
        k1 = self.k1_func(tau)
        k2 = self.k2_func(tau)
        k3 = self.k3_func(tau)
        return k1, k2, k3
    
    def Psi(self, p0, p1):
        step = 0.01
        k=1.0

        if p0 > p1:
            temp = p0
            p0 = p1
            p1 = temp
            k = -1.0

        x = np.arange(p0, p1 + step, step)  # +step чтобы включить p2
        y = np.zeros_like(x)

        for i in range(len(x)):
            beta = self.dynamics.GetBeta(x[i])
            alpha = self.dynamics.GetAlpha(x[i])
            #print (f"x_i={x[i]}, alpha = {alpha}, beta={beta}, b/a={beta/alpha}")
            y[i] = beta / alpha

        integ = 0.0
        for t in x:
            integ = integ + y[i]*step
        
        # Вычисляем интеграл методом трапеций
        #integral = np.trapz(y, x)

        return np.exp(-2 * k * integ)
    
    def GetIntegralNum(self, x, dx, tht, dtht, xStarZero, dxStarZero):
        step = 0.01
        integ = 0.0

        range_arr = np.arange(xStarZero, x + step, step)

        for t in range_arr:
            integ = integ + (2 * self.Psi(t, xStarZero) * self.dynamics.GetGamma(t) / self.dynamics.GetAlpha(t))*step
        
        ii = dx**2 - self.Psi(xStarZero, x) * (dxStarZero**2 - integ)
        return ii
    
    def GetIntegralSpline(self, xStarZero, dxStarZero):
        lim = 1000
        step_x = 0.005
        xRange = np.arange(-0.1, 2*np.pi + 0.1 + step_x, step_x)
    
        I_1 = np.zeros(len(xRange))
        I_2 = np.zeros(len(xRange))
        I_3 = np.zeros(len(xRange))
                
        def int_psi(z):
            return self.dynamics.GetBeta(z) / self.dynamics.GetAlpha(z)
        
        for i in range(len(xRange)):
            
            I_1[i], _ = quad(int_psi, xStarZero, xRange[i], 
                            epsrel=1e-8, epsabs=1e-12, limit=lim)
            
            I_2[i], _ = quad(int_psi, xRange[i], xStarZero, 
                            epsrel=1e-8, epsabs=1e-12, limit=lim)
        
        I_1_fcn = CubicSpline(xRange, I_1)
        I_2_fcn = CubicSpline(xRange, I_2)
        
        def int_main(z):
            return (np.exp(-2 * I_2_fcn(z)) * 2 * self.dynamics.GetGamma(z) / self.dynamics.GetAlpha(z))
        
        for i in range(len(xRange)):
            
            I_3[i], _ = quad(int_main, xStarZero, xRange[i], 
                            epsrel=1e-8, epsabs=1e-12, limit=lim)
        
        I_3_fcn = CubicSpline(xRange, I_3)
        
        def I_main(q, dq):
            return dq**2 - np.exp(-2 * I_1_fcn(q)) * (dxStarZero**2 - I_3_fcn(q))
        
        return I_main

    def GetTransverseCoords(self, x, dx, tht, dtht):
        x_ = self.ModPeriodX(x)
        
        y = tht - self.dynamics.Servo(x_)[1][0]
        dy = dtht - self.dynamics.dServo(x_)[1][0] * dx
        # tau = self.GetTauFromXStar(x_)
        # y = self.GetThtStarFromTau(tau) - self.dynamics.Servo(x_)[1][0]
        # dy = self.GetdThtStarFromTau(tau) - self.dynamics.dServo(x_)[1][0] * dx

        #i = self.GetIntegralNum(x_, dx, tht, dtht, self.GetXStarFromTau(0), self.GetdXStarFromTau(0))
        i = self.IntegralNum(x_, dx)
        return y, dy, i
    
    def publish_commands(self):
        
        u_max = 0.1
        # Создаем сообщения для моментов
        pendulum_torque_msg = Float64()
        rotor_torque_msg = Float64()

        self.get_logger().info(f"count = {self.numberOfIter}")

        # calculate torques
        tau = self.GetTau(self.spheroPosX)
        k1, k2, k3 = self.GetK(tau)

        y, dy, integral = self.GetTransverseCoords(self.spheroPosX, self.spheroVelX, self.pendulumAng, self.pendulumAngVel)
        self.get_logger().info(f"y = {y}, dy={dy}, i={integral}")
        tr_coords_msg = Float64MultiArray()
        tr_coords_msg.data = [y, dy, integral]
        self.transversal_coords_pub.publish(tr_coords_msg)

        k = 1.0
        v = k*(k1*integral + k2*y + k3*dy)
        self.get_logger().info(f"v = {v}")

        u = -(v - self.dynamics.f(self.spheroPosX, self.spheroVelX, self.pendulumAng, self.pendulumAngVel)) / self.dynamics.g(self.spheroPosX, self.spheroVelX, self.pendulumAng, self.pendulumAngVel)
        # u = u - 0.025
        #u = np.clip(u, -u_max, u_max)
        self.get_logger().info(f"u_clamped = {u}")
                
        # Публикуем сообщения
        pendulum_torque_msg.data = u
        rotor_torque_msg.data = 0.0
        self.pendulum_torque_pub.publish(pendulum_torque_msg)
        self.rotor_torque_pub.publish(rotor_torque_msg)
        self.numberOfIter = self.numberOfIter + 1
        
        # Логируем отправленные данные для отладки
        #self.get_logger().info(f'Sended: pendulum = {u}, rotor = {0:.6f}')

    


def main(args=None):
    """
    Главная функция для запуска узла.
    """
    # Инициализируем библиотеку rclpy
    rclpy.init(args=args)
    
    # Создаем экземпляр нашего узла
    node = TransversalController()
    
    # Запускаем узел и держим его активным.
    # spin() блокирует выполнение программы, пока узел не будет остановлен (Ctrl+C)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Обрабатываем прерывание от пользователя
        node.get_logger().info('Узел остановлен пользователем.')
    finally:
        # Корректно завершаем работу узла
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()