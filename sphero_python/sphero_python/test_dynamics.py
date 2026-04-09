from sphero_dynamics import SpheroDynamics as SD
from sphero_transversal import TransversalController as TC
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import os
from scipy.interpolate import make_interp_spline
import rclpy

def ModPeriodX(x):
        period = 2*np.pi
        return x % period


# Создание экземпляра
rclpy.init(args=None)
dynamics = SD()
controller = TC()

# Проверка методов
tht = 0.001
dtht = 0.0
x = 0.0
dx = 0.3

print(f"M={dynamics.M(tht)}")
print(f"C={dynamics.C(tht, dtht)}")
print(f"G={dynamics.G(tht)}")
print(f"B={dynamics.B()}")
print(f"BPerp={dynamics.BPerp()}")
print(f"Servo={dynamics.Servo(x)}")
print(f"Servo1={dynamics.Servo(x)[1][0]}")
print(f"dServo={dynamics.dServo(x)}")
print(f"ddServo={dynamics.ddServo(x)}")

print(f"alpha={dynamics.GetAlpha(x)}")
print(f"beta={dynamics.GetBeta(x)}")
print(f"gamma={dynamics.GetGamma(x)}")

print(f"f={dynamics.f(x, dx, tht, dtht)}")
print(f"g={dynamics.g(x, dx, tht, dtht)}")

# data_star = sio.loadmat('/home/ros/ros2_ws/src/sphero_python/sphero_python/x_star.mat')
# X = data_star['S']
# T = data_star['T']
# V = data_star['V']
# T_per = T[-1]

# x_of_tau_func = make_interp_spline(T[:,0], X[:,0], k=3) # bc_type='periodic'
# tau_of_x_func = make_interp_spline(X[:,0], T[:,0], k=3)

# t_smooth = np.linspace(T.min(), T.max(), 300)
# x_smooth = x_of_tau_func(t_smooth)

# t_smooth1 = np.linspace(X.min(), X.max(), 300)
# tau_smooth = tau_of_x_func(t_smooth1)
# tau_of_x_func.extrapolate = 'periodic'

# data = sio.loadmat('/home/ros/ros2_ws/src/sphero_python/sphero_python/data_K.mat')
# K_stab = data['K_stab']

# t_k = np.arange(0.0, T_per+0.01, 0.01)
# k1_func = make_interp_spline(t_k, K_stab[:,0], k=3,bc_type='periodic')
# k2_func = make_interp_spline(t_k, K_stab[:,1], k=3,bc_type='periodic')
# k3_func = make_interp_spline(t_k, K_stab[:,2], k=3,bc_type='periodic')

# k1_points = k1_func(t_k)
# k2_points = k2_func(t_k)
# k3_points = k3_func(t_k)

psi = controller.Psi(0.1, 1.0)
print(f"psi={psi}")

y, dy, i = controller.GetTransverseCoords(0.0, 0.3, 0.001, 0.0)
print(f"y={y}, dy={dy}, i={i}")

# plt.figure(figsize=(8, 5))
# plt.plot(t_k, k1_points)
# #plt.plot(t_smooth, x_smooth, 'r-', label='B-Spline (интерполяция)')
# plt.xlabel('t')
# plt.ylabel('k1')
# plt.grid(True)

# plt.figure(figsize=(8, 5))
# plt.plot(t_k, k2_points)
# plt.xlabel('t')
# plt.ylabel('k2')
# plt.grid(True)

# plt.figure(figsize=(8, 5))
# plt.plot(t_k, k3_points)
# plt.xlabel('t')
# plt.ylabel('k3')
# plt.grid(True)

# plt.figure(figsize=(8, 5))
# plt.plot(X, T)
# plt.plot(t_smooth1, tau_smooth, 'r-', label='B-Spline1')
# plt.xlabel('X')
# plt.ylabel('tau')
# plt.grid(True)
#plt.show()

