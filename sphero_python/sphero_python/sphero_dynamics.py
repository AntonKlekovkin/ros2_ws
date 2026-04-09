import numpy as np

class SpheroDynamics():

    def __init__(self):
        self.params = {
            "M_m": 0.251, 
            "m": 0.179, 
            "i": 0.792e-4,
            "I": 2.592e-4,
            "R": 0.06,
            "rho": 0.024,
            "g": 9.81,
            "L": 0.001
            }
        
    def M(self, tht):
        M_m = self.params["M_m"]
        m = self.params["m"]
        i = self.params["i"]
        I = self.params["I"]
        R = self.params["R"]
        rho = self.params["rho"]
        g = self.params["g"]

        #tht_ = float(tht)

        a = M_m + I/(R*R) + i/(R*R)
        b = (-i/R + m*rho*np.cos(tht))
        c = m*rho*rho + i

        # print(f"Тип i: {type(i)}, значение: {i}")
        # print(f"Тип R: {type(R)}, значение: {R}")
        # print(f"Тип m: {type(m)}, значение: {m}")
        # print(f"Тип rho: {type(rho)}, значение: {rho}")
        # print(f"Тип tht: {type(tht)}, значение: {tht}")
        # print(f"Тип np.cos(tht): {type(np.cos(tht))}, значение: {np.cos(tht)}")

        # print(f"Тип b: {type(b)}, значение: {b}")

        # print(f"a={a}")
        # print(f"b={b}")
        # print(f"c={c}")
        return np.array([[a,b],[b,c]])
    
    def C(self, tht, dtht):
        M_m = self.params["M_m"]
        m = self.params["m"]
        i = self.params["i"]
        I = self.params["I"]
        R = self.params["R"]
        rho = self.params["rho"]
        g = self.params["g"]

        #tht_ = float(tht)
        #dtht_ = float(dtht)

        return np.array ([[0, -m*rho*np.sin(tht) * dtht],[0,0]])
    
    def G(self, tht):
        M_m = self.params["M_m"]
        m = self.params["m"]
        i = self.params["i"]
        I = self.params["I"]
        R = self.params["R"]
        rho = self.params["rho"]
        g = self.params["g"]

        #tht_ = float(tht)

        return np.array([[0.0, m*g*rho*np.sin(tht)]]).T
    
    def B(self):
        R = self.params["R"]
        return np.array([[-1.0/R],[1.0]])
    
    def BPerp(self):
        R = self.params["R"]
        return np.array([1.0, 1.0/R])
    
    def Servo(self, x):
        L = self.params["L"]
        return np.array([[x],[L*np.cos(x)]])
    
    def dServo(self, x):
        L = self.params["L"]
        return np.array([[1.0],[-L*np.sin(x)]])
    
    def ddServo(self, x):
        L = self.params["L"]
        return np.array([[0.0],[-L*np.cos(x)]])
    
    def GetAlpha(self, x):
        return (self.BPerp() @ self.M(self.Servo(x)[1][0]) @ self.dServo(x))[0]

    def GetBeta(self, x):
        return (self.BPerp() @ self.M(self.Servo(x)[1][0]) @ self.ddServo(x) 
                             + self.BPerp() @ self.C(self.Servo(x)[1][0], self.dServo(x)[1][0]) @ self.dServo(x))[0]
    
    def GetGamma(self, x):
        return (self.BPerp() @ self.G(self.Servo(x)[1][0]))[0]
    
    def f(self, x, dx, tht, dtht):
        flVect = self.GetFLVect(x)
        return (flVect @ np.linalg.inv(self.M(tht)) @ (-self.C(tht, dtht) @ np.array([[dx],[dtht]]) - self.G(tht)))[0] - self.ddServo(x)[1][0] * dx**2

    def g(self, x, dx, tht, dtht): 
        flVect = self.GetFLVect(x)
        return (flVect @ np.linalg.inv(self.M(tht)) @ self.B())[0]
    
    def GetFLVect(self, x):
        return np.array([-self.dServo(x)[1][0],1])