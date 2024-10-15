import pygame
import math
import numpy as np
import sys


class SDVModel:
    
    def __init__(self, startpos, car_img, width, length):
        # resolution
        self.m2p = 3779.52  # meters to pixels

        # robot dims
        self.w = width * self.m2p
        self.l = length * self.m2p
        self.x = startpos[0]
        self.y = startpos[1]
        self.x_est = 0
        self.y_est = 0
        self.theta = 0.0
        self.delta = 0.0
        self.delta_prev = 0.0
        self.cross_track = 0
        
        # speed
        self.v = 0
        self.max_speed = 0.02 * self.m2p
        self.min_speed = -0.02 * self.m2p

        # graphics
        self.img = pygame.image.load(car_img)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
    
    def draw(self, WIN, display_font, color):
        WIN.blit(self.rotated, self.rect)

        # vehicle states title
        name_text = display_font.render("VEHICLE STATES", 1, color)
        WIN.blit(name_text, (55, 30))

        # x pos text
        name_text = display_font.render("X: "+str(int(self.x)), 1, color)
        WIN.blit(name_text, (55, 60))

        # y pos text
        time_text = display_font.render('Y: '+str(int(self.y)), 1, color)
        WIN.blit(time_text, (55, 90))

        # theta text
        name_text = display_font.render("θ: "+str(int(math.degrees(self.theta)))+"°", 1, color)
        WIN.blit(name_text, (55, 120))

        # delta text
        time_text = display_font.render('δ: '+str(int(math.degrees(self.delta)))+"°", 1, color)
        WIN.blit(time_text, (55, 150))

        # vp text
        name_text = display_font.render("vp: "+str(round(self.v, 2))+" p/s", 1, color)
        WIN.blit(name_text, (55, 180))

        # vr text
        name_text = display_font.render("vr: "+str(round(self.v/self.m2p, 2))+" m/s", 1, color)
        WIN.blit(name_text, (55, 210))

        # Lateral Controller title
        name_text = display_font.render("CONTROLLER", 1, color)
        WIN.blit(name_text, (255, 30))

        # cross-track text
        name_text = display_font.render("Cross-Track: "+str(round(self.cross_track, 2))+" rads", 1, color)
        WIN.blit(name_text, (255, 60))

    def move(self, event=None, dt=None, pp_controller=None, path=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.v += 0.005*self.m2p  # accelerate forward

                elif event.key == pygame.K_DOWN:
                    self.v -= 0.005*self.m2p  # accelerate backward

        # pure pursuit controller
        if pp_controller is not None:
            car_curr_pos = [self.x, self.y, self.theta]

            # calculate steering angle
            if path is not None:
                str_angle = pp_controller.compute_steering_angle(car_curr_pos, path)
                self.cross_track = str_angle
                
                # Apply correct steering direction
                str_threshold = -0.3 # for applying a more smooth steering response

                if str_angle > str_threshold and str_angle > 0:
                    self.delta = 1.2  # adjust the steering sensitivity as needed

                elif str_angle > str_threshold and str_angle < 0:
                    self.delta = -1.2  # adjust the steering sensitivity

                elif str_angle < str_threshold and str_angle < 0:
                    # do nothing, maintain current heading (theta)
                    self.delta = -1.1

                elif str_angle < str_threshold and str_angle > 0:
                    # do nothing, maintain current heading (theta)
                    self.delta = 1.1

                else:
                   # do nothing, maintain current heading (theta)
                   print('maintaining current heading')

                # Apply steering angle to the vehicle
                self.theta += (self.v / self.l) * math.tan(self.delta) * dt

            else:
                print('path is none')
        
        else:
            print('No lateral controller found')
            sys.exit()


        # update vehicle's pose and orientation
        self.y -= self.v * math.cos(self.theta) * dt
        self.x += self.v * math.sin(self.theta) * dt

        # apply change to car
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))