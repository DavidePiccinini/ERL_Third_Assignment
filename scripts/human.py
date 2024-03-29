#!/usr/bin/env python

## @package human
# Implements the basic behaviour of a person controlling the robot via voice commands.
# The person keeps sending either "Play" or "GoTo: Location" commands randomly.

import rospy
import time
import random
import os
from std_msgs.msg import String

## Publisher
comPub = None

## Command
command = String()

## Log file
logfile = None


##
# Publishes a simple "Play" command on the topic.
def sendPlayCommand():
    global logfile

    # Create the string to send
    command.data = "Play"

    # Print a feedback message
    logfile.write("\n[%f] Person: sending a '%s' command to the robot.\n" %(time.time(), command.data))
    logfile.flush()
    os.fsync(logfile)

    # Send the command
    comPub.publish(command)


##
# Publishes a "GoTo: Location" command on the topic by choosing randomly the location.
def sendGoToCommand():
    global logfile 

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
    command.data = "GoTo: " + locations.get(result, "\nError in human.py.\n")

    # Print a feedback message
    logfile.write("\n[%f] Person: sending a '%s' command to the robot.\n" %(time.time(), command.data))
    logfile.flush()
    os.fsync(logfile)

    # Send the command
    comPub.publish(command)


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

        # Wait for some time
        time.sleep(30)


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('human')

        # Open the logfile to write on it
        script_path = os.path.abspath(__file__) 
        path_list = script_path.split(os.sep)
        script_directory = path_list[0:len(path_list)-2]
        file_path = "log/human_logfile.txt"
        path = "/".join(script_directory) + "/" + file_path
        logfile = open(path, 'w')

        # Initialize the command publisher
        comPub = rospy.Publisher("command", String, queue_size=1)

        # Wait for some time before sending commands (to setup the system)
        time.sleep(5)

        human()
        
    except rospy.ROSInterruptException:
        pass