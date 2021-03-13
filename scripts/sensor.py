#!/usr/bin/env python

## @package sensor
# Acts as a robot sensor that perceives the commands told by the user.
# If a command is recognized, it's formatted and sent to the robot.

import rospy
import time
import random
import os
from std_msgs.msg import String
from erl_third_assignment.msg import FormattedCommand

## Publisher
formComPub = None

## Subscriber
comSub = None

## Formatted command
formCommand = FormattedCommand()

## Log file
logfile = None


##
# Understands the command told by the user and format it to send it to the robot.
# @param ros_data The unformatted command told by the user.
def formatCommand(ros_data):
    # Split the command
    temp = ros_data.data.split(": ")

    # List the possible main commands
    possibleCommands = {
        "Play": formatPlay,
        "GoTo": formatGoTo
    }

    # Establish which function to use
    formatting = possibleCommands.get(temp[0], "\nError in sensor.py.\n")

    # Call the appropriate function
    formatting(temp)


##
# Formats a "Play" command.
# @param command The string list containing the command.
def formatPlay(command):
    global logfile 
    global formCommand

    # Create the formatted "Play" command
    formCommand.mainCommand = command[0]
    formCommand.parameter = ""

    # Print a feedback message
    logfile.write("\n[%f] Sensor: sending a 'Play' formatted command.\n" %time.time())
    logfile.flush()
    os.fsync(logfile)

    # Publish it
    formComPub.publish(formCommand)


##
# Formats a "GoTo" command.
# @param command The string list containing the command.
def formatGoTo(command):
    global logfile 
    global formCommand

    # Create the formatted "GoTo" command
    formCommand.mainCommand = command[0]
    formCommand.parameter = command[1]

    # Print a feedback message
    logfile.write("\n[%f] Sensor: sending a 'GoTo: %s' formatted command.\n" %(time.time(), formCommand.parameter))
    logfile.flush()
    os.fsync(logfile)

    # Publish it
    formComPub.publish(formCommand)


if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node('sensor')

        # Open the log file to write on it
        script_path = os.path.abspath(__file__) 
        path_list = script_path.split(os.sep)
        script_directory = path_list[0:len(path_list)-2]
        file_path = "log/sensor_logfile.txt"
        path = "/".join(script_directory) + "/" + file_path
        logfile = open(path, 'w')

        # Initialize the command subscriber
        comSub = rospy.Subscriber("command", String, formatCommand, queue_size=1)

        # Initialize the formatted command publisher
        formComPub = rospy.Publisher("sensor/formatted_command", FormattedCommand, queue_size=1)

        # Wait for some time (to setup the system)
        time.sleep(5)

        while not rospy.is_shutdown():
            # Keep from exiting until the node is stopped
            rospy.spin()
        
    except rospy.ROSInterruptException:
        pass