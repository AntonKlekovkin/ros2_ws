from sphero_dynamics import SpheroDynamics as SD
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import os
from scipy.interpolate import make_interp_spline

def ModPeriodX(x):
        period = 2*np.pi
        return x % period


# Создание экземпляра
dynamics = SD()

# Проверка методов
tht = 0.5
dtht = 0.1
x = 0.2
dx = 0.02

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

print(f"fl_vect={dynamics.GetFLVect(x) @ np.linalg.inv(dynamics.M(tht)) @ dynamics.B() }")

print(f"f={dynamics.f(x, dx, tht, dtht)}")
print(f"g={dynamics.g(x, dx, tht, dtht)}")

data_star = sio.loadmat('/home/ros/ros2_ws/src/sphero_python/sphero_python/x_star.mat')
X = data_star['S']
T = data_star['T']
T_per = T[-1]

print(X[0])
print(T_per[0])

x_of_tau_func = make_interp_spline(T[:,0], X[:,0], k=3) # bc_type='periodic'
tau_of_x_func = make_interp_spline(X[:,0], T[:,0], k=3)

t_smooth = np.linspace(T.min(), T.max(), 300)
x_smooth = x_of_tau_func(t_smooth)

t_smooth1 = np.linspace(X.min(), X.max(), 300)
tau_smooth = tau_of_x_func(t_smooth1)
tau_of_x_func.extrapolate = 'periodic'

print(ModPeriodX(10*np.pi))

print(tau_of_x_func(np.pi))

data = sio.loadmat('/home/ros/ros2_ws/src/sphero_python/sphero_python/data_K.mat')
K_stab = data['K_stab']

print(K_stab[:,0])

t_k = np.arange(0.0, T_per+0.01, 0.01)
k1_func = make_interp_spline(t_k, K_stab[:,0], k=3,bc_type='periodic')
k2_func = make_interp_spline(t_k, K_stab[:,1], k=3,bc_type='periodic')
k3_func = make_interp_spline(t_k, K_stab[:,2], k=3,bc_type='periodic')

k1_points = k1_func(t_k)
k2_points = k2_func(t_k)
k3_points = k3_func(t_k)

plt.figure(figsize=(8, 5))
plt.plot(t_k, k1_points)
#plt.plot(t_smooth, x_smooth, 'r-', label='B-Spline (интерполяция)')
plt.xlabel('t')
plt.ylabel('k1')
plt.grid(True)

plt.figure(figsize=(8, 5))
plt.plot(t_k, k2_points)
plt.xlabel('t')
plt.ylabel('k2')
plt.grid(True)

plt.figure(figsize=(8, 5))
plt.plot(t_k, k3_points)
plt.xlabel('t')
plt.ylabel('k3')
plt.grid(True)

# plt.figure(figsize=(8, 5))
# plt.plot(X, T)
# plt.plot(t_smooth1, tau_smooth, 'r-', label='B-Spline1')
# plt.xlabel('X')
# plt.ylabel('tau')
# plt.grid(True)
plt.show()

