
#%%
from math import *
import numpy as np
import matplotlib.pyplot as plt 
class moteurCC : 
    def __init__(self, R = 1, L = 0 , E_t = 0, Um = 0 , i = 0 ,J = 0.01 ,f = 0.1  ,Couple_moteur = 0 , Vitesse_moteur= 0 ,step = 0.1, Kc = 0.01, Ke = 0.01):
        self.R = R
        self.L = L 
        self.i = i 
        self.J = J
        self.f = f
        self.Um =Um 
        self.E_t = E_t 
        self.Couple_moteur = Couple_moteur
        self.Vitesse_moteur = Vitesse_moteur
        self.step = step 
        self.Kc = Kc
        self.Ke = Ke 
        self.temps = [0]
        self.position = 0
        self.vitesse_list = []
        self.position_list = []
    
    
    def __str__(self) : 
        return "MoteurCC (%s,%s,%s,%s,%s,%s, %g, %s, %s)" % (self.R, self.L,self.i, self.J,self.f,self.Um,self.E_t,self.Couple_moteur, self.Vitesse_moteur) 
    
    def __repr__(self) : 
        return str(self)
    
    def setVoltage(self, V) : 
        self.Um = V
        return self.Um 
    
    def getPosition(self) : 
        return self.position
    def getSpeed(self) : 
        return self.Vitesse_moteur
    def getTorque(self):
        self.Couple_moteur = self.Kc * self.i
        return self.Couple_moteur 
    def getIntensity(self): 
        return self.i
    
    
    def simulate(self,step) : 
        if self.L != 0 : 
            self.E_t = self.Ke * self.Vitesse_moteur
            di_dt = (self.Um - self.E_t - self.R * self.i ) / self.L 
            self.i += di_dt * self.step
            
            self.Couple_moteur = self.Kc * self.i
            dOmega_dt = (self.Couple_moteur - self.f * self.Vitesse_moteur) / self.J 
            self.Vitesse_moteur += dOmega_dt * step
            
        elif self.L == 0 : 
            self.i = (self.Um - self.Ke * self.Vitesse_moteur) /self.R
            self.Couple_moteur = self.Kc * self.i
            dOmega_dt = (self.Couple_moteur - self.f * self.Vitesse_moteur) / self.J 
            self.Vitesse_moteur += dOmega_dt * step
        self.vitesse_list.append(self.Vitesse_moteur)
        self.position += self.Vitesse_moteur * self.step
        self.position_list.append(self.position)

             
    def plot_vitesse(self,temps) : 
        plt.plot(temps,self.vitesse_list)
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse")
        plt.title("Évolution de la vitesse du moteur CC")
        plt.grid()
        plt.legend()
        plt.show()  
    def plot_position(self,temps) : 
        plt.plot(temps,self.position_list)
        plt.xlabel("Temps (s)")
        plt.ylabel("position")
        plt.title("Évolution de la position du moteur CC")
        plt.grid()
        plt.legend()
        plt.show() 

class ControlPid_vitesse() :
    def __init__(self,Kp=0,Ki=0,Kd=0,vitesse_cible=1, moteurCC= moteurCC()) : 
        self.Kp = Kp 
        self.Ki = Ki 
        self.Kd = Kd 
        self.vitesse_cible = vitesse_cible
        self.moteurCC = moteurCC
        self.integrale = 0
        self.voltages = []
        self.erreurs = []
        self.erreur_pre = 0
        self.temps = [0]
        self.integrale_list = []
    
    def setTarget(self, vitesse) : 
        self.vitesse_cible = vitesse 
    
    def getVoltage(self): 
        return self.moteurCC.Um
    
    def simulate(self, step):
        vitesse_actuelle = self.moteurCC.getSpeed()
        erreur = self.vitesse_cible - vitesse_actuelle
        self.integrale += erreur * step
        self.integrale = max(min(self.integrale, 100), -100)
        derivee = (erreur - self.erreur_pre) / step
        self.erreur_pre = erreur
        tension = self.Kp * erreur + self.Ki * self.integrale + self.Kd * derivee
        tension = max(min(tension, 12), -12)
        self.integrale_list.append(self.integrale)
        self.moteurCC.setVoltage(tension)
        self.moteurCC.simulate(step)

        self.voltages.append(tension)
        self.erreurs.append(erreur)

    def plot(self):
        import matplotlib.pyplot as plt
        plt.plot(self.temps, self.moteurCC.vitesse_list, label="vitesse")
        plt.plot(self.temps, [self.vitesse_cible]*len(self.temps), '--', label="consigne")
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse")
        plt.grid()
        plt.title("Réponse en boucle fermée (PID)")
        plt.legend()
        plt.show()

        plt.figure()
        plt.plot(self.temps[1:], self.voltages, label="tension (Um)")
        plt.xlabel("Temps (s)")
        plt.ylabel("Tension (V)")
        plt.grid()
        plt.legend()
        plt.title("Tension de commande")
        plt.show()
         
        
class ControlPid_position() :
    def __init__(self,Kp_pos,Ki_pos,Kd_pos,pos_cible, controleur_vitesse=ControlPid_vitesse()) : 
        self.Kp = Kp_pos 
        self.Ki = Ki_pos 
        self.Kd = Kd_pos 
        self.controleur_vitesse = controleur_vitesse
        self.integrale = 0
        self.erreur_pre = 0
        self.pos_cible = pos_cible
        
    def simulate(self,step) : 
        position_actuelle = self.controleur_vitesse.moteurCC.getPosition()
        erreur = self.pos_cible - position_actuelle 
        self.integrale += erreur*step
        self.integrale = max(min(self.integrale, 100), -100)
        derivee = (erreur - self.erreur_pre) / step
        self.erreur_pre = erreur
        
        
        vitesse_cible = self.Kp * erreur + self.Ki * self.integrale + self.Kd * derivee 
        vitesse_cible = max(min(vitesse_cible, 10), -10)
        self.controleur_vitesse.setTarget(vitesse_cible)
        self.controleur_vitesse.simulate(step)
        
    

def test_influence_gains():
    # Liste de configurations (Kp, Ki)
    configurations = [
        (10, 0),    # P seul faible
        (100, 0),   # P seul élevé
        (10, 100),  # PI modéré
        (100, 300)  # PI fort
    ]
    
    step = 0.01
    duration = 5
    t_values = np.arange(0, duration, step)
    
    plt.figure(figsize=(10, 5))
    
    for Kp, Ki in configurations:
        m = moteurCC(L=0, Kc=0.01, Ke=0.01)
        controller = ControlPid_vitesse(Kp=Kp, Ki=Ki, Kd=0, vitesse_cible=1.0, moteurCC=m)
        controller.temps = [0]
        m.vitesse_list = [0]
        
        t = 0
        while t < duration:
            controller.simulate(step)
            controller.temps.append(controller.temps[-1] + step)
            t += step

        # Ensure both lists have the same length for plotting
        min_len = min(len(controller.temps), len(m.vitesse_list))
        plt.plot(controller.temps[:min_len], m.vitesse_list[:min_len], label=f"Kp={Kp}, Ki={Ki}")
    
    plt.axhline(1.0, linestyle="--", color="gray", label="Consigne")
    plt.title("Influence des gains P et I sur la vitesse ω(t)")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse ω(t)")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

def test_robustesse():
    Kp = 1000
    Ki = 1000
    Kd=  1000
    step = 0.01
    duration = 5
    m = moteurCC(L=0, Kc=0.01, Ke=0.01)
    controller = ControlPid_vitesse(Kp=Kp, Ki=Ki, Kd=0, vitesse_cible=1.0, moteurCC=m)
    controller.temps = [0]
    m.vitesse_list = [0]

    t = 0
    while t < duration:
        controller.simulate(step)
        controller.temps.append(controller.temps[-1] + step)
        t += step

    plt.figure(figsize=(8, 4))
    min_len = min(len(controller.temps[:-1]), len(m.vitesse_list))
    plt.plot(controller.temps[:min_len], m.vitesse_list[:min_len], label=f"Gains extrêmes Kp={Kp}, Ki={Ki}, Kd= {Kd}", color='red')
    plt.axhline(1.0, linestyle="--", color="gray", label="Consigne")
    plt.title("Robustesse du simulateur moteur CC avec gains extrêmes")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse ω(t)")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

def test_PID_position_PD():
    configurations = [
        (1, 0),      # P seul
        (1, 5),      # PD modéré
        (2, 10),     # PD fort
        (5, 20),     # P élevé + D élevé
    ]

    duration = 2
    step = 0.001
    pos_cible = 1.0

    plt.figure(figsize=(10, 5))

    for Kp, Kd in configurations:
        m = moteurCC(L=0.5, Kc=0.01, Ke=0.01)
        ctrl_vit = ControlPid_vitesse(Kp=500, Ki=0, Kd=0, moteurCC=m)
        ctrl_pos = ControlPid_position(Kp_pos=Kp, Ki_pos=0, Kd_pos=Kd, pos_cible=pos_cible, controleur_vitesse=ctrl_vit)
        
        t = 0
        ctrl_vit.temps = [0]
        m.position_list = [0]
        m.vitesse_list = [0]

        while t < duration:
            ctrl_pos.simulate(step)
            ctrl_vit.temps.append(ctrl_vit.temps[-1] + step)
            t += step

        # Préparer données pour le plot
        min_len = min(len(ctrl_vit.temps), len(m.position_list))
        plt.plot(ctrl_vit.temps[:min_len], m.position_list[:min_len], label=f"Kp={Kp}, Kd={Kd}")

    plt.axhline(pos_cible, linestyle="--", color="gray", label="Consigne")
    plt.title("Influence des gains P et D sur la position (boucle fermée)")
    plt.xlabel("Temps (s)")
    plt.ylabel("Position (rad)")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()


def main (): 
    #Test de la classe Moteur avec l = 0.5 et l = 0 
    
    Moteur_L0 = moteurCC()
    Moteur_L05 = moteurCC(L=0.5)
    step = Moteur_L0.step

    temps = [0]
    Moteur_L0.setVoltage(1)
    Moteur_L0.vitesse_list = [Moteur_L0.Vitesse_moteur]  # 0.0
    Moteur_L05.setVoltage(1)
    Moteur_L05.vitesse_list = [Moteur_L0.Vitesse_moteur]  # 0.0
    t = 0
    Moteur_L0.position_list= [Moteur_L0.position]
    Moteur_L05.position_list = [Moteur_L05.position]

    while t < 5:
        t += step
        Moteur_L0.simulate(step)
        Moteur_L05.simulate(step)
        temps.append(t)

    plt.figure()
    Moteur_L0.plot_vitesse(temps)
    Moteur_L05.plot_vitesse(temps)
    Moteur_L0.plot_position(temps)
    Moteur_L05.plot_position(temps)
    
    test_influence_gains()
    test_robustesse()
    #Test du controlleur PID vitesse 
    
    m_bo = moteurCC(L=0.5, Kc = 0.01, Ke=0.01)
    m_bf = moteurCC(L=0.5, Kc = 0.01, Ke=0.01)
    K = m_bo.Kc

    P = 100
    I = 0
    D = 0
    control = ControlPid_vitesse(Kp=P, Ki=I, Kd=D, vitesse_cible=1.0, moteurCC=m_bf)
    
    t = 0
    step = 0.01
    temps = [0]
    m_bo.vitesse_list = [0]
    m_bf.vitesse_list = [0]
    control.integrale = 0
    control.erreur_pre = 0 

    while t < 5:
        t += step
        temps.append(t)

        m_bo.setVoltage(1 / K)
        m_bo.simulate(step)

        control.setTarget(1.0)
        control.simulate(step)
        control.temps.append(t)
    plt.plot(temps, m_bf.vitesse_list, label="Boucle fermée (PI)")
    plt.axhline(1.0, linestyle='--', color='gray', label="Consigne 1 rad/s")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (rad/s)")
    plt.title("Comparaison boucle ouverte vs fermée")
    plt.grid()
    plt.legend()
    plt.show()
    
    print(f"integral list : {control.integrale_list}")
    print(f"voltages : {control.voltages}")
    
    
    #Position PID 
    
    m_bo = moteurCC(L=0.5, Kc=0.01, Ke=0.01)
    m_bf = moteurCC(L=0.5, Kc=0.01, Ke=0.01)
    K = m_bo.Kc

    P_pos = 1  # tu peux ajuster
    I_pos = 0
    D_pos = 0

    control_vitesse = ControlPid_vitesse(Kp=500, Ki=0, Kd=0, moteurCC=m_bf)
    control_position = ControlPid_position(Kp_pos=P_pos, Ki_pos=I_pos, Kd_pos=D_pos, pos_cible=1.0, controleur_vitesse=control_vitesse)

    t = 0
    step = 0.001
    temps = [0]
    m_bf.position_list = [0]
    m_bf.vitesse_list = [0]
    control_vitesse.integrale = 0
    control_vitesse.erreur_pre = 0
    control_position.integrale = 0
    control_position.erreur_pre = 0
    control_vitesse.temps = [0]

    while t < 0.2:
        t += step
        control_position.simulate(step)
        control_vitesse.temps.append(t)
        temps.append(t)

    # Plot de la position (comme pour la vitesse)
    plt.plot(temps, m_bf.position_list, label="Boucle fermée (P position)")
    plt.axhline(1.0, linestyle='--', color='gray', label="Consigne 1 rad")
    plt.xlabel("Temps (s)")
    plt.ylabel("Position (rad)")
    plt.title("Réponse en position avec PID")
    plt.grid()
    plt.legend()
    plt.show()
    
    test_PID_position_PD()

    

    
    
if __name__ == "__main__":    
    main()
# %%
