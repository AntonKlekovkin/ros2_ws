from sphero_dynamics import SpheroDynamics as SD
import numpy as np

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