import math
import random
from time import sleep
import pygame
from math import sin, cos, pi, atan
from constants import SCREEN_WIDTH, SCREEN_HEIGHT, PIX2M, M2PIX


class Simulation(object):
    """
    Represents the simulation.
    """
    def __init__(self, roomba):
        """
        Creates the simulation.

        :param roomba: the roomba robot used in this simulation.
        :type roomba: Roomba
        """
        self.point_list = []
        self.roomba = roomba

    def check_collision(self):
        """
        Checks collision between the robot and the walls.

        :return: the bumper state (if a collision has been detected).
        :rtype: bool
        """
        # Converting screen limits from pixels to meters
        width = SCREEN_WIDTH * PIX2M
        height = SCREEN_HEIGHT * PIX2M
        bumper_state = False
        # Computing the limits of the roomba's bounding box
        left = self.roomba.pose.position.x - self.roomba.radius
        right = self.roomba.pose.position.x + self.roomba.radius
        top = self.roomba.pose.position.y - self.roomba.radius
        bottom = self.roomba.pose.position.y + self.roomba.radius
        # Testing if the bounding box has hit a wall
        if left <= 0.0:
            self.roomba.pose.position.x = self.roomba.radius
            bumper_state = True
        if right >= width:
            self.roomba.pose.position.x = width - self.roomba.radius
            bumper_state = True
        if top <= 0.0:
            self.roomba.pose.position.y = self.roomba.radius
            bumper_state = True
        if bottom >= height:
            self.roomba.pose.position.y = height - self.roomba.radius
            bumper_state = True
        return bumper_state

    def update(self):
        """
        Updates the simulation.
        """
        # Adding roomba's current position to the movement history
        self.point_list.append((round(M2PIX * self.roomba.pose.position.x), round(M2PIX * self.roomba.pose.position.y)))
        if len(self.point_list) > 2000:
            self.point_list.pop(0)
        # Verifying collision
        bumper_state = self.check_collision()
        self.roomba.set_bumper_state(bumper_state)
        # Updating the robot's behavior and movement
        self.roomba.update()

    def draw(self, window):
        """
        Draws the roomba and its movement history.

        :param window: pygame's window where the drawing will occur.
        """
        # If we have less than 2 points, we are unable to plot the movement history
        if len(self.point_list) >= 2:
            pygame.draw.lines(window, (255, 0, 0), False, self.point_list, 4)
        # Computing roomba's relevant points and radius in pixels
        sx = round(M2PIX * self.roomba.pose.position.x)
        sy = round(M2PIX * self.roomba.pose.position.y)
        ex = round(M2PIX * (self.roomba.pose.position.x + self.roomba.radius * cos(self.roomba.pose.rotation)))
        ey = round(M2PIX * (self.roomba.pose.position.y + self.roomba.radius * sin(self.roomba.pose.rotation)))
        r = round(M2PIX * self.roomba.radius)
        # Drawing roomba's inner circle
        pygame.draw.circle(window, (200, 200, 200), (sx, sy), r, 0)
        # Drawing roomba's outer circle
        pygame.draw.circle(window, (100, 100, 100), (sx, sy), r, 4)
        # Drawing roomba's orientation
        pygame.draw.line(window, (50, 50, 50), (sx, sy), (ex, ey), 3)

        if self.roomba.mission == 'Waypoint':
            x_w, y_w = self.roomba.drone_target
            x_w = round(M2PIX * x_w)
            y_w = round(M2PIX * y_w)


            pygame.draw.circle(window, (0, 0, 0), (x_w, y_w), 2, 0)

        if self.roomba.mission == 'StayAt':
            x_w, y_w = self.roomba.drone_target
            x_w = round(M2PIX * x_w)
            y_w = round(M2PIX * y_w)
            radius = round(M2PIX * 1)
            pygame.draw.circle(window, (0, 0, 0), (x_w, y_w), radius, 1)
            pygame.draw.circle(window, (255, 0, 0), (x_w, y_w), 2, 0)


        if self.roomba.mission == 'Scan':
            x_1, y_1, x_2, y_2 = self.roomba.drone_target

            x_1 = round(M2PIX * x_1)
            y_1 = round(M2PIX * y_1)

            x_2 = round(M2PIX * x_2)
            y_2 = round(M2PIX * y_2)


            pygame.draw.circle(window, (255, 0, 0), (x_1, y_1), 2, 0)
            pygame.draw.circle(window, (255, 0, 0), (x_2, y_2), 2, 0)





        if (self.roomba.mission == 'PaparazziEight') | (self.roomba.mission == 'Oval'):
            x_1, y_1, x_2, y_2 = self.roomba.drone_target

            x_1 = round(M2PIX * x_1)
            y_1 = round(M2PIX * y_1)

            x_2 = round(M2PIX * x_2)
            y_2 = round(M2PIX * y_2)



            h = abs(y_2- y_1)
            b = abs(x_1 - x_2)

            if b != 0:
                tan_theta = atan(h/b)

            else:
                tan_theta = pi



            #print(math.degrees(tan_theta))
            #sleep(1)

            radius = round(M2PIX * 0.5)

            pygame.draw.circle(window, (0, 0, 0), (x_1, y_1), 3, 0)
            pygame.draw.circle(window, (0, 0, 0), (x_1, y_1), radius, 1)


            pygame.draw.circle(window, (0, 0, 0), (x_2, y_2), 3, 0)
            pygame.draw.circle(window, (0, 0, 0), (x_2, y_2), radius, 1)



            path_point = []

            if x_1 > x_2:
                tan_theta = pi - tan_theta

            if (b == 0):
                factor = -pi/2

            else:
                factor = 0

            for i in range(3):
                x = x_2 + radius * cos(tan_theta + (i-1)*pi/2 + factor)
                y = y_2 + radius * sin(tan_theta + (i-1)*pi/2 + factor)
                path_point.append([x,y])
                pygame.draw.circle(window, (255, 0, 0), (x, y), 2, 0)
                x = x_1 - radius * cos(tan_theta + (1 - i) *pi / 2 + factor)
                y = y_1 - radius * sin(tan_theta + (1 - i) *pi / 2 + factor)
                path_point.append([x, y])
                pygame.draw.circle(window, (255, 0, 0), (x, y), 2, 0)




def draw(simulation, window):
    """
    Redraws the pygame's window.

    :param simulation: the simulation object.
    :param window: pygame's window where the drawing will occur.
    """
    window.fill((224, 255, 255))
    simulation.draw(window)
    pygame.display.update()
