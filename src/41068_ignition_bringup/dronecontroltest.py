#This script is for understanding and testing the functionality of the Drone Control and how to manipulate that in code.
import rclpy
from dronecontrolling import DroneController

def main():
    rclpy.init()
    controller = DroneController()

    # Run some test moves
    controller.move(speed=0.4, duration=30.0)
    controller.height(speed=1.0, duration=2.0)
    controller.turn(angular_speed=0.6, duration=2.0)
    controller.stop()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()