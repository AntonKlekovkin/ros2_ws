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

        a = M_m + I/(R*R) + i/(R*R)
        b = -i/R + m*rho*np.cos(tht)
        c = m*rho*rho + i
        return np.array([[a,b],[b,c]])
    
    def C(self, tht, dtht):
        M_m = self.params["M_m"]
        m = self.params["m"]
        i = self.params["i"]
        I = self.params["I"]
        R = self.params["R"]
        rho = self.params["rho"]
        g = self.params["g"]

        return np.array ([[0, -m*rho*np.sin(tht) * dtht],[0,0]])
    
    def G(self, tht):
        M_m = self.params["M_m"]
        m = self.params["m"]
        i = self.params["i"]
        I = self.params["I"]
        R = self.params["R"]
        rho = self.params["rho"]
        g = self.params["g"]

        return np.array([[0],[m*g*rho*np.sin(tht)]])
    
    def B(self):
        R = self.params["R"]
        return np.array([[-1/R],[1]])
    
    def BPerp(self):
        R = self.params["R"]
        return np.array([1, 1/R])
    
    def Servo(self, x):
        L = self.params["L"]
        return np.array([[x],[L*np.cos(x)]])
    
    def dServo(self, x):
        L = self.params["L"]
        return np.array([[1],[-L*np.sin(x)]])
    
    def ddServo(self, x):
        L = self.params["L"]
        return np.array([[0],[-L*np.cos(x)]])
    
    def GetAlpha(self, x):
        return self.BPerp @ self.M(self.Servo(x)[1]) @ self.dServo(x)

    def GetBeta(self, x):
        return self.BPerp @ (self.M(self.Servo(x)[1]) @ self.ddServo(x) 
                             + self.C(self.Servo(x)[1], self.dServo(x)[1]) @ self.dServo(x))
    
    def GetGamma(self, x):
        return self.BPerp @ self.G(self.Servo(x)[1])
    
    def f(self, x, dx, tht, dtht):
        return -np.invert(self.M(tht)) @ self.C(tht, dtht) @ np.array([[dx],[dtht]]) - np.invert(self.M(tht)) @ self.G(tht)

    def g(self, x, dx, tht, dtht):   
        return -np.invert(self.M(tht)) @ self.B()