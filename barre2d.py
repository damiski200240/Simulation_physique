from random import random,randint
from torseur import Torseur
from vector3D import Vector3D as V3D
from particule import Particule
import pygame
from pygame.locals import *
from types import MethodType
from univers import *


class Barre2D():
    from torseur import Torseur 
    def __init__(self, masse = 1, longeur = 1, v0=V3D(), a0=V3D(), centre=V3D(), fix=False, color='red', name='barre', point_rotation = None):
        self.masse = masse
        self.longeur = longeur
        self.centre = centre
        self.inertie_centre = (masse * (longeur ** 2)) * 1/12
        self.forces = V3D()
        self.moments = V3D()  # moment scalaire
        self.fix = fix
        self.color = color
        self.name = name
        self.position = [centre]
        self.speed = [v0]
        self.acceleration = [a0]
        self.angle = [0]  # angle en radian
        self.angular_speed = [0]
        self.angular_acceleration = [0]
        self.torseurs = []
        self.point_rotation = None
    def __str__(self):
        msg = 'Barre (' + str(self.masse) + ', ' + str(self.position[-1]) + ', ' + str(self.speed[-1]) + ', ' + str(self.acceleration[-1]) + ', "' + self.name + '", "' + str(self.color) + '" )'
        return msg

    def __repr__(self):
        return str(self)

    def applyforce(self, torseur = Torseur, point_application=0):
        if point_application == 0:
            self.pos = self.getPosition()
        elif 0 < point_application <= 1 : 
            self.pos = self.getPosition() + V3D(self.longeur * abs(point_application)/2,0,0)
        elif -1 <= point_application < 0 : 
            self.pos = self.getPosition() - V3D(self.longeur * abs(point_application)/2,0,0)
        else : 
            print("Erreur: point_application doit être -1 (centre) ou 1 (extrémité)")
        torseur = Torseur(P=self.pos, R=torseur.R, M=torseur.M)
        self.torseurs.append(torseur)
    
    
    
    def applyTorseur(self, torseur):
        self.torseurs.append(torseur)
    
    def apply():
        pass

    def simulate(self, step):
        self.pfd(step)

    def pfd(self, step):
        if not self.fix:
            rotation_point = self.point_rotation if self.point_rotation else self.getPosition()
            # Translation
            R = V3D()
            M = V3D()
            for t in self.torseurs:
                t.changePoint(rotation_point)  # centre
                R += t.R
                M += t.M

            a = R * (1/self.masse)
            v = self.speed[-1] + a * step
            p = self.position[-1] + 0.5 * a * step**2 + self.speed[-1] * step

            # Inertie par rapport au point de rotation
            d = self.getPosition() - rotation_point
            I = self.inertie_centre + self.masse * d.mod() ** 2  # théorème de Huygens

            # Rotation
            alpha = M.z / I
            w = self.angular_speed[-1] + alpha * step
            theta = self.angle[-1] + self.angular_speed[-1] * step + 0.5 * alpha * step**2

        else:
            a = V3D()
            v = V3D()
            p = self.position[-1]
            alpha = 0
            w = 0
            theta = self.angle[-1]

        self.acceleration.append(a)
        self.speed.append(v)
        self.position.append(p)

        self.angular_acceleration.append(alpha)
        self.angular_speed.append(w)
        self.angle.append(theta)

        self.torseurs = []
        self.point_rotation = None
    def getPosition(self):
        return self.position[-1]

    def getSpeed(self):
        return self.speed[-1]

    def plot(self):
        from pylab import plot
        X = []
        Y = []
        for p in self.position:
            X.append(p.x)
            Y.append(p.y)
        return plot(X, Y, color=self.color, label=self.name)

    def gameDraw(self, scale, screen):
        from math import cos, sin

        screen_height = screen.get_height()

        # Centre
        X = int(scale * self.getPosition().x)
        Y = screen_height - int(scale * self.getPosition().y)  

        theta = self.angle[-1]
        dx = (self.longeur / 2) * cos(theta)
        dy = (self.longeur / 2) * sin(theta)

        # Extrémités
        x1 = int(scale * (self.getPosition().x - dx))
        y1 = screen_height - int(scale * (self.getPosition().y - dy))  
        x2 = int(scale * (self.getPosition().x + dx))
        y2 = screen_height - int(scale * (self.getPosition().y + dy))  

        # Couleur
        if type(self.color) is tuple:
            color = (self.color[0] * 255, self.color[1] * 255, self.color[2] * 255)
        else:
            color = self.color

        size = 3
        pygame.draw.line(screen, color, (x1, y1), (x2, y2), size)
        pygame.draw.circle(screen, color, (X, Y), size * 2, size)




if __name__ == '__main__':
    from pylab import figure, show, legend

    # Initialisation Pygame
    pygame.init()
    screen_width, screen_height = 800, 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Simulation Barre2D")
    clock = pygame.time.Clock()

    # Echelle pour l'affichage
    scale = 30  # 1 unité = 30 pixels

    # Création d'une barre lancée en l'air avec vitesse initiale
    B0 = Barre2D(
        masse=1,
        longeur=2,
        v0=V3D(0, 0, 0),
        centre=V3D(10, 10, 0),
        color='blue',
        name='barre_lancee', 
    )

    print(B0)
    
    # Simulation jusqu'à ce que la barre retombe au sol
    dt = 0.1
    B0.point_rotation = B0.centre + V3D(B0.longeur/2,0,0) 
    while B0.getPosition().y >= 0:
        Torseur_effort = Torseur(R=V3D(0, 1, 0))
        B0.applyforce(Torseur_effort, point_application=-1)  # gravité
        B0.simulate(dt)
        screen.fill((255, 255, 255))  # fond blanc
        B0.gameDraw(scale, screen)
        pygame.display.flip()
        clock.tick(60)

