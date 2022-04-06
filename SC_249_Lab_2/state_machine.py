import random
import math
from time import sleep

from utils import *
from math import cos,sin, pi, atan
from constants import *
import pygame
import simulation
import numpy as np
import numpy.linalg as LA


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name, count = 0):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name
        self.count = count

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

class WayPoint(State):
    def __init__(self):
        super().__init__("MoveForward")
        # Todo: add initialization code
        self.estado_alinhamento = False
        self.estado_way = False
        self.movimentos = 0
        self.deg = 1000
        self.sentido = 1


    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if self.movimentos == 1:
            num = random.uniform(0,1)
            agent.drone_target = None
            if num < 0.1:
                state_machine.change_state(PaparazziEight())
            elif num < 0.25:
                state_machine.change_state(StayAt())
            elif num < 0.45:
                state_machine.change_state(Scan())

            elif num < 0.65:
                state_machine.change_state(Oval())

            else:
                state_machine.change_state(WayPoint())

        pass

    def execute(self, agent, target = None):
        # Todo: add execution logic
        #agent.set_velocity(-FORWARD_SPEED,self.count)

        agent.mission = 'Waypoint'
        if agent.drone_target == None:
            agent.drone_target = (rand_num(2,6),rand_num(2,6))

        x_c = agent.pose.position.x
        y_c = agent.pose.position.y

        x_e = agent.pose.position.x + agent.radius * cos(agent.pose.rotation)
        y_e = agent.pose.position.y + agent.radius * sin(agent.pose.rotation)

        x_w,y_w = agent.drone_target

        vec_target = np.array([x_w - x_c, y_w - y_c])
        vec_mov = np.array([x_e - x_c, y_e - y_c])

        a = vec_target
        b = vec_mov

        inner = np.inner(a, b)
        norms = LA.norm(a) * LA.norm(b)

        cos_t = inner / norms

        rad = np.arccos(np.clip(cos_t, -1.0, 1.0))
        deg = np.rad2deg(rad)


        deg_error = 0.5

        if deg <= deg_error:
            self.estado_alinhamento = True
            agent.set_velocity(3,0)
        else:
            if deg > 10:
                factor = 3
            else:
                factor = 1

            if self.deg < deg:
                self.sentido = self.sentido * -1

            agent.set_velocity(0, self.sentido * factor * ANGULAR_SPEED)
            self.deg = deg



           # if ((x_w < x_c) & (y_w > y_c)):
           #     agent.set_velocity(0, -1*rad*ANGULAR_SPEED)
           # else:
           #     agent.set_velocity(0, rad * ANGULAR_SPEED)
        dist_error = 0.1
        d = dist([x_c,y_c],[x_w,y_w])
        if d <= dist_error:

            #self.estado_way = True
            #agent.set_velocity(0, 0)
            agent.drone_target = (random.uniform(2, 6), random.uniform(2, 6))
            self.movimentos += 1
            #state_machine.change_state(MoveInSpiralState())




        #print(agent.pose.position.y)

        maxX = 6.25
        minX = 0.17

       # print(agent.pose.position.x)
        pass


class PaparazziEight(State):
    def __init__(self):
        super().__init__("PaparazziEight")
        # Todo: add initialization code
        self.etapa = 'Approach First'
        self.proximo = None
        self.sentido = 1
        self.curva_dir = False
        self.movimentos = 0
        self.deg = 1000

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # Cálculo do tempo (em segundos)
        time = self.count * SAMPLE_TIME

        if self.movimentos == 1:
            # Mudanca de estado:  GoBack -> t3 = 0.5s -> Rotate
            agent.drone_target = None
            state_machine.change_state(WayPoint())

        pass

    def execute(self, agent):
        # Todo: add execution logic
        agent.mission = 'PaparazziEight'

        if agent.drone_target == None:
            agent.drone_target = (3,2,   #rand_num(2,6),2,
                                  4,4)    #rand_num(2,6),2)


        p1 = agent.drone_target[:2] # CENTRO MAIS A ESQUERDA
        p2 = agent.drone_target[2:] # CENTRO MAIS A DIREITA
        x_1, y_1, x_2, y_2 = agent.drone_target # AS COORDENADAS DOS PONTOS

        radius = 0.5
        h = abs(y_2 - y_1)
        b = abs(x_1 - x_2)

        if b != 0:
            tan_theta = atan(h / b)
        else:
            tan_theta = pi

        if x_1 > x_2:
            tan_theta = pi - tan_theta

        path_point = []
        if b == 0:
            factor = -pi / 2
        else:
            factor = 0

        for i in range(3):
            x = x_2 + radius * cos(tan_theta + (i - 1) * pi / 2 + factor)
            y = y_2 + radius * sin(tan_theta + (i - 1) * pi / 2 + factor)
            path_point.append([x, y])
            x = x_1 - radius * cos(tan_theta + (1 - i) * pi / 2 + factor)
            y = y_1 - radius * sin(tan_theta + (1 - i) * pi / 2 + factor)
            path_point.append([x, y])


        # Primeiros 2 pares - Pontos extremos de baixo
        # Em seguida os do meio
        # Extremo de cima

        p_first = path_point[-2]

        x_c,y_c = agent.pose.position.x, agent.pose.position.y
        x_e,y_e = agent.pose.position.x + agent.radius * cos(agent.pose.rotation), agent.pose.position.y + agent.radius * sin(agent.pose.rotation)
        vec_mov = np.array([x_e - x_c, y_e - y_c])
        vec_mov = vec_mov / (math.sqrt(vec_mov[0] ** 2 + vec_mov[1] ** 2))

        if self.etapa == 'Approach First':

            vec_target = np.array([p_first[0] - x_c, p_first[1] - y_c])
            vec_target = vec_target/(math.sqrt(vec_target[0]**2 + vec_target[1]**2))
            deg = ang_vector_deg(vec_target, vec_mov)

            if deg <= 1:

                distancia = dist((p_first[0],p_first[1]), (x_c,y_c))

                if distancia >= 0.05:
                    agent.set_velocity(1, 0)
                else:
                    self.etapa = 'First Translado'

            else:
                if deg > 10:
                    factor = 3
                else:
                    factor = 1

                if self.deg < deg:
                    self.sentido = self.sentido *-1

                agent.set_velocity(0,  self.sentido * factor * ANGULAR_SPEED)
                self.deg = deg


        elif self.etapa == 'First Translado':

            focal_point = path_point[1]
            vec_tranlado = np.array([focal_point[0] - x_c, focal_point[1] - y_c])/(math.sqrt((focal_point[0] - x_c) ** 2 + (focal_point[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_tranlado, vec_mov)

            if (deg >= 2):
                if self.deg < deg:
                    self.sentido = self.sentido * -1

                agent.set_velocity(0, self.sentido * 2 * ANGULAR_SPEED)
                self.deg = deg


            else:
                agent.set_velocity(1, 0)

            if dist(focal_point, (x_c, y_c)) <= 0.04:
                self.etapa = 'Left Curve'

        elif self.etapa == 'Left Curve':
            vec_radius = np.array([p1[0] - x_c, p1[1] - y_c])/(math.sqrt((p1[0] - x_c) ** 2 + (p1[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_radius, vec_mov)

            if deg >= 91:
                agent.set_velocity(0, -2 )

                #agent.set_velocity(0, -2 * ANGULAR_SPEED)
                #else:
                #    agent.set_velocity(0, 2 * ANGULAR_SPEED)
            #else:
                #agent.set_velocity(2, 2/ ANGULAR_SPEED)
            else:
                agent.set_velocity(1, -1  / radius)


            if dist(path_point[-1], (x_c, y_c)) <= 0.04:
                #agent.set_velocity(0, 0)
                self.etapa = 'Troca Circ'


        elif self.etapa == 'Troca Circ':
            focal_point = path_point[0]
            vec_tranlado = np.array([focal_point[0] - x_c, focal_point[1] - y_c]) / (math.sqrt((focal_point[0] - x_c) ** 2 + (focal_point[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_tranlado, vec_mov)


            if (deg >= 2):
                if self.deg < deg:
                    self.sentido = self.sentido * -1

                agent.set_velocity(0, self.sentido * 2 * ANGULAR_SPEED)
                self.deg = deg
            else:
                agent.set_velocity(1, 0)

            if dist(focal_point, (x_c, y_c)) <= 0.04:
                self.etapa = 'Teste'


        elif self.etapa == 'Teste':
            vec_radius = np.array([p2[0] - x_c, p2[1] - y_c]) / (math.sqrt((p2[0] - x_c) ** 2 + (p2[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_radius, vec_mov)


            if deg > 90:

                agent.set_velocity(0, 2)

            else:
                agent.set_velocity(2, -2 / radius)

            if dist(path_point[4], (x_c, y_c)) <= 0.04:

               self.etapa = 'Fim'

        elif self.etapa == 'Fim':
            self.etapa = 'First Translado'
            self.movimentos += 1
            if self.movimentos == 1:
                agent.drone_target = (1, 2,  # rand_num(2,6),2,
                                      4, 4)
            elif self.movimentos == 2:
                agent.drone_target = (4, 2,  # rand_num(2,6),2,
                                      4, 4)


        self.count += 1
        pass


class StayAt(State):
    def __init__(self):
        super().__init__("StayAt")
        # Todo: add initialization code
        self.etapa = 'Approach'
        self.tempo = 0
        self.movimento = 0
        self.first_point = None
        self.deg = 1000
        self.sentido = 1

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        if (self.tempo > 25) | (self.movimento == 2):
            # Mudanca de estado:  GoBack -> t3 = 0.5s -> Rotate
            agent.drone_target = None
            state_machine.change_state(WayPoint())

        pass

    def execute(self, agent):
        # Todo: add execution logic
        agent.mission = 'StayAt'

        if agent.drone_target == None:
            agent.drone_target = (4, 2)

        radius = 1

        centro = agent.drone_target

        x_c, y_c = agent.pose.position.x, agent.pose.position.y
        x_e, y_e = agent.pose.position.x + agent.radius * cos(agent.pose.rotation), agent.pose.position.y + agent.radius * sin(agent.pose.rotation)
        vec_mov = np.array([x_e - x_c, y_e - y_c])
        vec_mov = vec_mov / (math.sqrt(vec_mov[0] ** 2 + vec_mov[1] ** 2))

        if self.etapa == 'Approach':
            vec_target = np.array([centro[0] - x_c, centro[1] - y_c])
            vec_target = vec_target / (math.sqrt(vec_target[0] ** 2 + vec_target[1] ** 2))
            deg = ang_vector_deg(vec_target, vec_mov)
            distancia = dist((centro[0], centro[1]), (x_c, y_c))
            #print(deg)
            if (deg < 1):


                if distancia >= radius:

                    agent.set_velocity(1, 0)
                else:
                    self.etapa = 'Alinhamento'
                    self.first_point = [agent.pose.position.x, agent.pose.position.y]
                    agent.set_velocity(0,0)

            else:

                if deg > 10:
                    factor = 1.5
                else:
                    factor = 1

                if self.deg < deg:
                    self.sentido = self.sentido * -1

                agent.set_velocity(0, self.sentido * factor)
                self.deg = deg

        if self.etapa == 'Alinhamento':

            vec_radius = np.array([centro[0] - x_c, centro[1] - y_c]) / (math.sqrt((centro[0] - x_c) ** 2 + (centro[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_radius, vec_mov)



            if (deg < 88) | (deg > 91):
                if deg < 89:
                    agent.set_velocity(0, 1)
                else:
                    agent.set_velocity(0, -1)
            else:
                #print('Entrou')
                self.etapa = 'Circular'
                #agent.set_velocity(2,2/radius)
        if self.etapa == 'Circular':
            agent.set_velocity(2,-1/radius)

            vec_radius = np.array([centro[0] - x_c, centro[1] - y_c]) / (
                math.sqrt((centro[0] - x_c) ** 2 + (centro[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_radius, vec_mov)
            if (deg < 88) | (deg > 91):
                self.etapa = 'Alinhamento'







        self.tempo += SAMPLE_TIME
        pass




class Oval(State):
    def __init__(self):
        super().__init__("Oval")
        # Todo: add initialization code
        self.etapa = 'Approach First'
        self.proximo = None
        self.curva_esq = False
        self.curva_dir = False
        self.movimentos = 0
        self.sentido = 1
        self.deg = 1000

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # Cálculo do tempo (em segundos)
        time = self.count * SAMPLE_TIME

        if self.movimentos == 1:
            # Mudanca de estado:  GoBack -> t3 = 0.5s -> Rotate
            agent.drone_target = None
            state_machine.change_state(WayPoint())

        pass

    def execute(self, agent):
        # Todo: add execution logic
        agent.mission = 'Oval'

        if agent.drone_target == None:
            agent.drone_target = (3,2,   #rand_num(2,6),2,
                                  4,4)    #rand_num(2,6),2)


        p1 = agent.drone_target[:2] # CENTRO MAIS A ESQUERDA
        p2 = agent.drone_target[2:] # CENTRO MAIS A DIREITA
        x_1, y_1, x_2, y_2 = agent.drone_target # AS COORDENADAS DOS PONTOS

        radius = 0.5
        h = abs(y_2 - y_1)
        b = abs(x_1 - x_2)

        if b != 0:
            tan_theta = atan(h / b)
        else:
            tan_theta = pi

        if x_1 > x_2:
            tan_theta = pi - tan_theta

        path_point = []
        if b == 0:
            factor = -pi / 2
        else:
            factor = 0

        for i in range(3):
            x = x_2 + radius * cos(tan_theta + (i - 1) * pi / 2 + factor)
            y = y_2 + radius * sin(tan_theta + (i - 1) * pi / 2 + factor)
            path_point.append([x, y])
            x = x_1 - radius * cos(tan_theta + (1 - i) * pi / 2 + factor)
            y = y_1 - radius * sin(tan_theta + (1 - i) * pi / 2 + factor)
            path_point.append([x, y])


        # Primeiros 2 pares - Pontos extremos de baixo
        # Em seguida os do meio
        # Extremo de cima

        p_first = path_point[-2]

        x_c,y_c = agent.pose.position.x, agent.pose.position.y
        x_e,y_e = agent.pose.position.x + agent.radius * cos(agent.pose.rotation), agent.pose.position.y + agent.radius * sin(agent.pose.rotation)
        vec_mov = np.array([x_e - x_c, y_e - y_c])
        vec_mov = vec_mov / (math.sqrt(vec_mov[0] ** 2 + vec_mov[1] ** 2))

        if self.etapa == 'Approach First':

            vec_target = np.array([p_first[0] - x_c, p_first[1] - y_c])
            vec_target = vec_target/(math.sqrt(vec_target[0]**2 + vec_target[1]**2))
            deg = ang_vector_deg(vec_target, vec_mov)

            if deg < 1:

                distancia = dist((p_first[0],p_first[1]), (x_c,y_c))

                if distancia >= 0.05:
                    agent.set_velocity(1, 0)
                else:
                    self.etapa = 'First Translado'

            else:
                if deg > 10:
                    factor = 3
                else:
                    factor = 1

                if self.deg < deg:
                    self.sentido = self.sentido * -1

                agent.set_velocity(0, self.sentido *factor)
                self.deg = deg


        elif self.etapa == 'First Translado':

            focal_point = path_point[-1]
            vec_tranlado = np.array([focal_point[0] - x_c, focal_point[1] - y_c])/(math.sqrt((focal_point[0] - x_c) ** 2 + (focal_point[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_tranlado, vec_mov)

            if (deg > 1):
                if self.deg < deg:
                    self.sentido = self.sentido * -1

                agent.set_velocity(0, self.sentido *ANGULAR_SPEED)
                self.deg = deg

            else:
                agent.set_velocity(1, 0)

            if dist(focal_point, (x_c, y_c)) <= 0.04:
                self.etapa = 'Left Curve'

        elif self.etapa == 'Left Curve':

            vec_radius = np.array([p1[0] - x_c, p1[1] - y_c])/(math.sqrt((p1[0] - x_c) ** 2 + (p1[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_radius, vec_mov)

            #print(deg)
            if deg > 90:
                agent.set_velocity(0, 2 )


            else:
                agent.set_velocity(1, -1  / radius)

            #print(dist(path_point[1], (x_c, y_c)))
            if dist(path_point[1], (x_c, y_c)) <= 0.04:
            #    #agent.set_velocity(0, 0)
                self.etapa = 'Troca Circ'


        elif self.etapa == 'Troca Circ':
            focal_point = path_point[0]
            vec_tranlado = np.array([focal_point[0] - x_c, focal_point[1] - y_c]) / (math.sqrt((focal_point[0] - x_c) ** 2 + (focal_point[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_tranlado, vec_mov)


            if (deg > 0.5):
                if self.deg < deg:
                    self.sentido = self.sentido * -1

                agent.set_velocity(1, self.sentido * ANGULAR_SPEED)
                self.deg = deg
            else:
                agent.set_velocity(1, 0)

            if dist(focal_point, (x_c, y_c)) <= 0.04:
                self.etapa = 'Teste'


        elif self.etapa == 'Teste':
            vec_radius = np.array([p2[0] - x_c, p2[1] - y_c]) / (math.sqrt((p2[0] - x_c) ** 2 + (p2[1] - y_c) ** 2))
            deg = ang_vector_deg(vec_radius, vec_mov)

            #print(deg)
            if deg > 90:
                agent.set_velocity(0, 2)

                # agent.set_velocity(0, -2 * ANGULAR_SPEED)
                # else:
                #    agent.set_velocity(0, 2 * ANGULAR_SPEED)
            # else:
            # agent.set_velocity(2, 2/ ANGULAR_SPEED)
            else:
                agent.set_velocity(2, -2 / radius)

            if dist(path_point[-2], (x_c, y_c)) <= 0.04:
               #agent.set_velocity(0, 0)
               self.etapa = 'Fim'

        elif self.etapa == 'Fim':
            self.etapa = 'First Translado'
            self.movimentos += 1
            if self.movimentos == 1:
                agent.drone_target = (1, 2,  # rand_num(2,6),2,
                                      4, 4)
            elif self.movimentos == 2:
                agent.drone_target = (4, 2,  # rand_num(2,6),2,
                                      4, 4)


        self.count += 1
        pass

class Scan(State):
    def __init__(self):
        super().__init__("Scan")
        # Todo: add initialization code
        self.etapa = 'Approach First'
        self.sentido = 1
        self.movimentos = 0
        self.deg = 1000
        self.t = 0
        self.i = 0

    def check_transition(self, agent, state_machine):
        # Todo: add logic to check and execute state transition

        # Cálculo do tempo (em segundos)
        time = self.count * SAMPLE_TIME

        if self.movimentos == 1:
            # Mudanca de estado:  GoBack -> t3 = 0.5s -> Rotate
            agent.drone_target = None
            state_machine.change_state(WayPoint())

        pass

    def execute(self, agent):
        # Todo: add execution logic
        agent.mission = 'Scan'

        if agent.drone_target == None:
            agent.drone_target = (2,2,   #rand_num(2,6),2,
                                  6,8)    #rand_num(2,6),2)


        p1 = agent.drone_target[:2] # CENTRO MAIS A ESQUERDA
        p2 = agent.drone_target[2:] # CENTRO MAIS A DIREITA
        x_1, y_1, x_2, y_2 = agent.drone_target # AS COORDENADAS DOS PONTOS
        radius = 0.5

        i = round((y_2 - y_1)/ (2*radius))

        p_first = p1

        x_c,y_c = agent.pose.position.x, agent.pose.position.y
        x_e,y_e = agent.pose.position.x + agent.radius * cos(agent.pose.rotation), agent.pose.position.y + agent.radius * sin(agent.pose.rotation)
        vec_mov = np.array([x_e - x_c, y_e - y_c])
        vec_mov = vec_mov / (math.sqrt(vec_mov[0] ** 2 + vec_mov[1] ** 2))

        if self.etapa == 'Approach First':

            vec_target = np.array([p_first[0] - x_c, p_first[1] - y_c])
            vec_target = vec_target/(math.sqrt(vec_target[0]**2 + vec_target[1]**2))
            deg = ang_vector_deg(vec_target, vec_mov)

            distancia = dist((p_first[0], p_first[1]), (x_c, y_c))
            if (deg < 1) & (deg > 0):

                agent.set_velocity(1, 0)

            else:
                if deg > 10:
                    factor = 3
                else:
                    factor = 1

                if self.deg < deg:
                    self.sentido = self.sentido *-1

                agent.set_velocity(0,  self.sentido * factor *ANGULAR_SPEED)
                self.deg = deg


            if distancia <= 0.01:
                self.etapa = 'Alinhamento'



        elif self.etapa == 'Alinhamento':


            if (vec_mov[0] < 0.9999) & (vec_mov[1] < 0.05):
                agent.set_velocity(0, 2 * ANGULAR_SPEED)
            else:
                agent.set_velocity(2, 0)


            distancia = abs(agent.pose.position.x - x_2)
            if distancia <= 0.01:
                self.etapa = 'Volta Direita'
                self.sentido = 1
                self.t = 0
                self.i += 1

        elif self.etapa == 'Volta Direita':
            agent.set_velocity(1, self.sentido*1/radius)

            if (vec_mov[0] < -0.9999) & (vec_mov[1] < 0.05):
                agent.set_velocity(2, 0)

            distancia = abs(agent.pose.position.x - x_1)
            #print(distancia)
            if distancia <= 0.01:
                self.etapa = 'Volta Esquerda'
                self.i += 1
                self.sentido = -1

        elif self.etapa == 'Volta Esquerda':

            agent.set_velocity(1, self.sentido*1/radius)

            if (vec_mov[0] < 0.9999) & (vec_mov[0] > 0) & (vec_mov[1] < 0.05):
                agent.set_velocity(2, 0)

            distancia = abs(agent.pose.position.x - x_2)
            #print()
            if distancia <= 0.01:
                self.etapa = 'Volta Direita'
                self.sentido = 1
                self.i += 1


            if self.i == i:
                self.etapa = 'Alinhamento final'

        elif self.etapa == 'Alinhamento final':

            vec_target = np.array([x_2 - x_c, y_2 - y_c])
            vec_target = vec_target / (math.sqrt(vec_target[0] ** 2 + vec_target[1] ** 2))
            deg = ang_vector_deg(vec_target, vec_mov)

            #print(deg)

            distancia = dist((x_2,y_2), (x_c, y_c))
            if (deg < 1) & (deg > 0):
                agent.set_velocity(1, 0)

            else:
                if self.deg < deg:
                    self.sentido = self.sentido *-1

                agent.set_velocity(1,  self.sentido / radius)
                self.deg = deg


            if distancia <= 0.01:
                self.etapa = 'Fim'

        elif self.etapa == 'Fim':
            self.movimentos = 1




