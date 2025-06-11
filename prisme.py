from random import random,randint
from vector3D import Vector3D as V3D
from particule import Particule
import pygame
from pygame.locals import *
from types import MethodType
from univers import *
from barre2d import Barre2D

class Prisme() : 
    def __init__ ( self, P0=Barre2D(fix = True), P1 = Barre2D(), n = V3D()): 
        self.P0 = P0 
        self.P1 = P1 
        self.n = n.norm() #direction normalisé 
    def prismatique(self, force = V3D()) : 
        force_proj = (force ** self.n) * self.n
        self.P1.applyforce(force_proj) 
        position = (self.P1.getPosition() ** self.n) * self.n
        position.y = self.P1.getPosition().y  # on garde la hauteur
        vitesse = (self.P1.getSpeed() ** self.n) * self.n
        #vitesse.y = self.P1.getSpeed().y
        self.P1.position[-1] = position  # forcer la position dans l'axe
        self.P1.speed[-1] = vitesse     # forcer la vitesse dans l'axe


if __name__ == '__main__':
    from pylab import figure, show, legend

    # Initialisation Pygame
    pygame.init()
    screen_width, screen_height = 800, 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Simulation Barre2D")
    clock = pygame.time.Clock()
    scale = 30  # 1 unité = 30 pixels

    barre_fixe = Barre2D(masse=10, longeur=2, centre=V3D(10, 10, 0), fix=True, color='black', name='fixe')
    barre_mobile = Barre2D(masse=1, longeur=2, centre=V3D(10, 10, 0), v0=V3D(0, 0, 0), color='blue', name='mobile')
    direction_x = V3D(10, 0, 0)
    direction_y = V3D(0,10,0)
    force_x = V3D(1,0,0)
    force_y =  V3D(0,1,0)
    Prisme_liaison = Prisme(barre_fixe, barre_mobile, direction_x)


    dt = 0.1
    while True:
        
        
        Prisme_liaison.prismatique(force_x)  # contraindre dans l'axe x
        barre_mobile.simulate(dt)
        

        screen.fill((255, 255, 255))
        barre_fixe.gameDraw(scale, screen)
        barre_mobile.gameDraw(scale, screen)
        pygame.display.flip()
        clock.tick(60)

