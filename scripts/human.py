#!/usr/bin/env python

## @package human
# Implements the basic behaviour of a person controlling the robot via voice commands.
# The person keeps sending either "Play" or "GoTo: Location" commands randomly.

import rospy
import time
import random
from std_msgs.msg import String

# Publisher
comPub = None

# Command
command = String()

##
# Publishes a simple "Play" command on the topic and then waits for some time.
def sendPlayCommand():
    # Create the string to send
    command.data = "Play"

    # Print a feedback message
    print("\nPerson: sending a '%s' command to the robot.\n" %command.data)

    # Send the command
    comPub.publish(command)

    # Wait for some time
    time.sleep(10)

##
# Publishes a "GoTo: Location" command on the topic by choosing randomly the location and then waits for some time.
def sendGoToCommand():
    # Possible locations
    locations = {
        1: "Entrance",
        2: "Closet",
        3: "Living Room",
        4: "Kitchen",
        5: "Bathroom",
        6: "Bedroom"
    }

    # Select a random location
    result = random.randint(1, 6)

    # Create the string to send
    command.data = "GoTo: " + locations.get(result, "Error in human.py.")

    # Print a feedback message
    print("\nPerson: sending a '%s' command to the robot.\n" %command.data)

    # Send the command
    comPub.publish(command)

    # Wait for some time
    time.sleep(10)

##
# Randomly execute one of the two "send command" functions.
def human():
    # Keep sending commands to the robot
    while not rospy.is_shutdown():

        action = random.randint(0, 1)

        if action == 0:
            sendPlayCommand()
        else:
            sendGoToCommand()


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('human')

        # Initialize the command publisher
        comPub = rospy.Publisher("command", String, queue_size=1)

        # Wait for some time before sending commands (to setup the system)
        time.sleep(20)

        human()
        
    except rospy.ROSInterruptException:
        pass