from sphero_dynamics import SpheroDynamics as SD
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import os
from scipy.interpolate import make_interp_spline

print("Текущая директория:", os.getcwd())

# Список файлов в текущей директории
print("Файлы в директории:", os.listdir('.'))


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

x_spl = make_interp_spline(T[:,0], X[:,0], k=3)

t_smooth = np.linspace(T.min(), T.max(), 300)
x_smooth = x_spl(t_smooth)

plt.figure(figsize=(10, 6))
plt.plot(T, X)
plt.plot(t_smooth, x_smooth, 'r-', label='B-Spline (интерполяция)')
plt.xlabel('Индекс')
plt.ylabel('X')
plt.title('График массива X')
plt.grid(True)
plt.show()

