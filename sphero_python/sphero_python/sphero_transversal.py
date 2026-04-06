#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  
from rclpy.node import Node  
from std_msgs.msg import Float64  # Тип сообщения для чисел с плавающей точкой
from sensor_msgs.msg import JointState  # Тип сообщения для состояния сочленений
from .sphero_dynamics import SpheroDynamics as SD
import numpy as np

class TransversalController(Node):
    
    def __init__(self):
        super().__init__('transversal_controller')

        # publishers
        self.pendulum_torque_pub = self.create_publisher(Float64, 'pendulum_torque', 10)
        self.rotor_torque_pub = self.create_publisher(Float64, 'rotor_torque', 10)
        
        # timer for control loop 
        timer_period = 0.1  # seconds
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

    def joint_states_callback(self, msg: JointState):
        self.spheroPosX = msg.position[1]
        self.spheroVelX = msg.velocity[1]
        self.pendulumAng = msg.position[0]
        self.pendulumAngVel = msg.velocity[0]

    def ModPeriod(self, x):
        period = 20.0
        return x % period

    def GetXStarFromTau(self, tau):
        # return from spline for xStar or inverse?
        return 0.0
    
    def GetdXStarFromTau(self, tau):
        # return from spline for dxStar or inverse?
        return 0.0
    
    def GetTauFromXStar(self, x):
        # return tau from spline for xStar or inverse?
        return 0.0

    def GetTau(self, x):
        xMod = self.ModPeriod(x)
        tau = self.GetTauFromXStar(xMod)
        return tau

    def GetK(self, tau):
        # get elements of matrix K from its splines
        return 0.0, 1.0, 2.0
    
    def Psi(self, p0, p1):
        step = 0.001

        x = np.arange(p0, p1 + step, step)  # +step чтобы включить p2

        # Вычисляем значения частного функций в каждой точке
        y = SD.GetBeta(x) / SD.GetAlpha(x)

        # Вычисляем интеграл методом трапеций
        integral = np.trapz(y, x)

        return np.exp(-2 * integral)
    
    def GetIntegral(self, x, dx, tht, dtht, xStarZero, dxStarZero):
        step = 0.001
        integ = 0.0
        for t in range(xStarZero, x, step):
            integ = integ + 2 * self.Psi(t, xStarZero) * SD.GetGamma(t) / SD.GetAlpha(t)
        
        ii = dx*dx - self.Psi(xStarZero, x) * (xStarZero^2 - integ)
        return ii

    def GetTransverseCoords(self, x, dx, tht, dtht):
        y = tht - SD.Servo(x)[1]
        dy = dtht - SD.dServo(x)[1] * dx
        i = self.GetIntegral(x, dx, tht, dtht, self.GetXStarFromTau(0), self.GetdXStarFromTau(0))
        return y, dy, i
    
    def publish_commands(self):
        
        # Создаем сообщения для моментов
        pendulum_torque_msg = Float64()
        rotor_torque_msg = Float64()

        # calculate torques
        tau = self.GetTau(self.spheroPosX)
        k1, k2, k3 = self.GetK(tau)

        y, dy, integral = self.GetTransverseCoords(self.spheroPosX, self.spheroVelX, self.pendulumAng, self.pendulumAngVel)
        v = k1*integral + k2*y + k3*dy

        u = (v - SD.f(self.spheroPosX, self.spheroVelX, self.pendulumAng, self.pendulumAngVel)) / SD.g(self.spheroPosX, self.spheroVelX, self.pendulumAng, self.pendulumAngVel)

                
        # Публикуем сообщения
        pendulum_torque_msg.data = u
        rotor_torque_msg.data = 0
        self.pendulum_torque_pub.publish(pendulum_torque_msg)
        self.rotor_torque_pub.publish(rotor_torque_msg)
        
        # Логируем отправленные данные для отладки
        self.get_logger().info(f'Sended: pendulum = {pendulum_torque_value:.6f}, rotor = {rotor_torque_value:.6f}')

    


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