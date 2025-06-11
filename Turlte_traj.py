from turtle_1 import *
import pygame
from math import * 
from vector3D import Vector3D as V3D
from moteurCC import *



class myUnivers(Univers): 
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Création des robots
        self.turtle1 = Turtle(P0=V3D(40, 50), R0=0, name='ideal', color='red')
        self.turtle2 = TurtlePID(P0=V3D(40, 50),name='pid', color='black')

        # Ajout dans la population
        self.addUnit(self.turtle1, self.turtle2)
        self.W, self.H = self.gameDimensions 
        self.index = 0

        # Initialize the route as a circular path
        self.route_1 = self.create_linear_path(start=V3D(40, 50), end=V3D(90, 50), num_points=20)
        self.route_2 = self.create_circular_path()
    def gameInteraction(self, events, keys):
        return super().gameInteraction(events, keys)
    
    
    def create_circular_path(self, radius=15, num_points=360):
        self.center = V3D(self.dimensions[0] / 2, self.dimensions[1] / 2)
        return [V3D(
            self.center.x + radius * cos(2 * pi * i / num_points),
            self.center.y + radius * sin(2 * pi * i / num_points)
        ) for i in range(num_points)]
    
    def create_linear_path(self, start, end, num_points=100):
        return [V3D(
            start.x + (end.x - start.x) * i / (num_points - 1),
            start.y + (end.y - start.y) * i / (num_points - 1)
        ) for i in range(num_points)]

    def stepAll(self):
        target = self.route_1[self.index % len(self.route_1)]
         # === DEBUG ===
        # print(f"\n--- Step {self.index} ---")
        # print(f"Target: ({target.x:.2f}, {target.y:.2f})")
        # print(f"Turtle1 Pos: ({self.turtle1.position.x:.2f}, {self.turtle1.position.y:.2f}) | Orientation: {self.turtle1.orientation:.2f} rad")
         
        #turtle ideal 
        v, w = self.turtle1.controlGoTo(target)
        # print(f"[Turtle1] v: {v:.2f}, w: {w:.2f}")
        self.turtle1.setRobotSpeeds(v,w)
        
        #turtle PID
        v2, w2 = self.turtle2.controlGoTo(target)
        left_rot, left_trans, right_rot, right_trans=self.turtle2.setRobotSpeeds(v2,w2)
        self.turtle2.setWheelTargetSpeeds(left_rot,right_rot)
        
        # DEBUG PID
        # print(f"[Turtle2 PID] v: {v2:.2f}, w: {w2:.2f}")
        # print(f"[PID Wheels] Left_rot: {left_rot:.2f}, Right_rot: {right_rot:.2f}")

        
        self.turtle1.move(self.step)
        self.turtle2.move(self.step)
        self.time.append(self.time[-1] + self.step)
        
        self.index += 1 
        
        
    def simulateRealTime(self):
        # Même que Univers mais avec affichage de la cible circulaire
        import pygame

        running = self.game
        successes, failures = pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))
        clock = pygame.time.Clock()

        while running:
            screen.fill((240, 240, 240))

            pygame.event.pump()
            keys = pygame.key.get_pressed()
            events = pygame.event.get()

            if keys[pygame.K_ESCAPE]:
                running = False
            for event in events:
                if event.type == pygame.QUIT:
                    running = False

            self.gameInteraction(events, keys)
            self.moveAll(1 / self.gameFPS)

            for t in self.population:
                t.gameDraw(self.scale, screen)

            # === Afficher la cible circulaire ===
            # Affichage des informations de debug dans la console (commentées pour désactiver)
            # print(f"\n--- Step {self.index} ---")
            # print(f"Target: ({target.x:.2f}, {target.y:.2f})")
            # print(f"Turtle1 Pos: ({self.turtle1.position.x:.2f}, {self.turtle1.position.y:.2f}) | Orientation: {self.turtle1.orientation:.2f} rad")
            # print(f"[Turtle1] v: {v:.2f}, w: {w:.2f}")
            # print(f"[Turtle2 PID] v: {v2:.2f}, w: {w2:.2f}")
            # print(f"[PID Wheels] Left_rot: {left_rot:.2f}, Right_rot: {right_rot:.2f}")
            for point in self.route_1:
                pygame.draw.circle(screen, (0, 255, 0), (int(self.scale * point.x), int(self.scale * point.y)), 2)


            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            pygame.display.flip()
            clock.tick(self.gameFPS)

        pygame.quit()
        
         
    
    
  

def main():
    step = 0.001
    uni = myUnivers(name="Turtles Circle", game=True, step=step)
    uni.simulateRealTime()  
    uni.plot()
    
if __name__ == "__main__":  
    main()