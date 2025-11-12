""" This script is for understanding and testing the functionality 
    of the Drone Control and how to manipulate that in code. """

import rclpy #For ROS Node Communication
from dronecontrolling import DroneController #For Drone control via DroneController Node

def main():
    #---- Pre-start Node Set Up-------
    rclpy.init() #Initialise ROS communication
    controller = DroneController() #Create DroneController Node instance
    #---------------------------------

    #----Queued Motions---------------
    controller.move_y(speed=1.0, duration=10.0) #Move Left 10 meters
    controller.move_x(speed=0.4, duration=10.0) #Move Forward 4 meters
    controller.height(speed=1.0, duration=2.0) #Move Up 2 meters
    controller.turn(angular_speed=0.6, duration=2.0) #Turn Left 1.2 Radians / 68.75 Degrees
    #controller.stop() #Stops the drone at it's current position
    #---------------------------------

    #----Start Movement of Drone------
    controller.start()  # Begin executing queued motions
    rclpy.spin(controller) # Keep the node alive so timers run
    #---------------------------------

    #----Clean Up after Movement------
    controller.destroy_node() #Clean the Node
    rclpy.shutdown() #Shut down ROS communcations
    #---------------------------------

if __name__ == '__main__':
    main()