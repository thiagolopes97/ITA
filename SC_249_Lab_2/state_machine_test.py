from utils import Pose
from constants import FREQUENCY
from roomba import Roomba
from simulation import *
from state_machine import *

pygame.init()

window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

pygame.display.set_caption("Lab 2 - Drone Finite State Machine")

clock = pygame.time.Clock()

behavior = FiniteStateMachine(PaparazziEight())
# behavior = RoombaBehaviorTree()
pose = Pose(PIX2M * SCREEN_WIDTH / 2.0, PIX2M * SCREEN_HEIGHT / 2.0, 0.0)
roomba = Roomba(pose, 1.0, 2.0, 0.34 / 2.0, behavior)
simulation = Simulation(roomba)

run = True

rechead = False

while run:
    clock.tick(FREQUENCY)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    simulation.update()
    draw(simulation, window)


pygame.quit()
