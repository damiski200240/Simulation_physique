from random import random,randint
from vector3D import Vector3D as V3D
from particule import Particule
import pygame
from pygame.locals import *
from types import MethodType
from univers import *
from barre2d import Barre2D


class Pivot : 
    def __init__ ( self, P0=Barre2D(fix = True), P1 = Barre2D(), n = V3D(0,0,1), sens_P0= 1 , sens_P1=-1): 
        self.P0 = P0 
        self.P1 = P1 
        self.n = n.norm() #on tourne suivnat z 
        self.sens_P0 = sens_P0
        self.sens_P1 = sens_P1

    def pivot(self,force) : 
        ext_P0  = self.P0.getPosition() + self.sens_P0 * V3D(self.P0.longeur/2,0,0)
        ext_P1  = self.P1.getPosition() + self.sens_P1 * V3D(self.P1.longeur/2,0,0)
        self.P1.point_rotation = ext_P1
        delta = ext_P0 - ext_P1
        # Corriger la position proprement
        self.P1.position[-1] = self.P1.position[-1] + delta

        # Appliquer la force à l'extrémité
        self.P1.applyforce(force, point_application=-self.sens_P1)

        self.P1.speed[-1] = V3D()

if __name__ == '__main__':
    from pylab import figure, show, legend

    pygame.init()
    screen_width, screen_height = 800, 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Simulation Barre2D")
    clock = pygame.time.Clock()

    scale = 30  # 1 unité = 30 pixels
    # Initialisation des barres
    barre_fixe = Barre2D(masse=10, longeur=2, centre=V3D(10, 10, 0), fix=True, color='black', name='fixe')
    barre_mobile = Barre2D(masse=1, longeur=2, centre=V3D(10, 10, 0), v0=V3D(0, 0, 0), color='blue', name='mobile')
    liaison_pivot = Pivot(barre_fixe, barre_mobile)
    dt= 0.1
    # Boucle de simulation
    while True:
        force_frr = V3D(0, -10, 0)  # force appliquée à l'extrémité de P1

        liaison_pivot.pivot(force_frr)  # appliquer la force et maintenir la liaison
        barre_mobile.simulate(dt)

        screen.fill((255, 255, 255))
        barre_fixe.gameDraw(scale, screen)
        barre_mobile.gameDraw(scale, screen)
        pygame.display.flip()
        clock.tick(60)

