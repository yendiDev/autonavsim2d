import pygame
import math

class Robot:
    
    def __init__(self, startpos, robotImg, width):
        # resolution
        self.m2p = 3779.52  # meters to pixels

        # robot dims
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = math.pi/2
        self.vl = 0.00 * self.m2p
        self.vr = 0.00 * self.m2p
        self.max_speed = 0.02 * self.m2p
        self.min_speed = -0.02 * self.m2p

        # graphics
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
    
    def draw(self, WIN):
        WIN.blit(self.rotated, self.rect)
    
    def move(self, event=None, theta=None, vl=None, vr=None, dt=None):

        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.vl += 0.001 * self.m2p
                
                elif event.key == pygame.K_DOWN:
                    self.vl -= 0.001 * self.m2p

                elif event.key == pygame.K_LEFT:
                    self.vr += 0.001 * self.m2p

                elif event.key == pygame.K_RIGHT:
                    self.vr -= 0.001 * self.m2p

        # updat vl and vr
        if vl is not None and vr is not None:
            self.vr += vr * self.m2p
            self.vl += vl * self.m2p

        self.x += ((self.vl +  self.vr)/2) * math.cos(theta) * dt
        self.y -= ((self.vl +  self.vr)/2) * math.sin(theta) * dt
        self.theta = theta * dt
        
        # set max speed
        self.vl = min(self.vl, self.max_speed)
        self.vr = min(self.vr, self.max_speed)

        # set min speed
        self.vl = max(self.vl, self.min_speed)
        self.vr = max(self.vr, self.min_speed)

        # apply change to robot
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
