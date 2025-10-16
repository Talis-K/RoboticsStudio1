import sys
import os

#For the drone movement
from Drone_Movement.dronecontrolling import DroneController as Drone

#For the drone movement
from subpackage_1.test_snake_waypoints import Goals


def main():
    goals = Goals().position()
    print(goals)


if __name__ == '__main__':
    main()
