#!/usr/bin/env python

## @package state_machine
# Defines the different robot behaviours and the transitions between them.
# Available states are NORMAL, SLEEP, PLAY and FIND.

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
import cv2
import imutils
import math
import threading
import os
import numpy as np
from scipy.ndimage import filters
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Float64
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from erl_third_assignment.msg import FormattedCommand

## Action client
movebaseClient = None

## Publishers
velPub = None
initPosPub = None

## Subscriber
imageSub = None
odomSub = None
commandSub = None

## Goal pose
mbGoal = MoveBaseGoal()

## Counter
sleepCounter = 0

## Command variables
receivedPlay = False
receivedGoTo = False
parameter = ""

## Flag to notify that the robot has seen the ball
ballFound = False

## Threading events
finishedMoving = threading.Event()
finishedGettingClose = threading.Event()

## Color masks
blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)
redLower = (0, 50, 50)
redUpper = (5, 255, 255)
greenLower = (50, 50, 50)
greenUpper = (70, 255, 255)
yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)
magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)
blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)

## Detected locations list
detectedLocations = []

## Current robot position
robotPosition_x = None
robotPosition_y = None

## Log file
logfile = None


##
#
def checkContours(image, maskLower, maskUpper):
    mask = cv2.inRange(image, maskLower, maskUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    return cnts


## FIXME
# 
# @param ros_data The compressed image picked up by the camera
def checkForBall(ros_data):
    global ballFound
    global finishedGettingClose
    global detectedLocations
    global movebaseClient
    global velPub
    global blueLower, blueUpper, redLower, redUpper, greenLower, greenUpper, yellowLower, yellowUpper, magentaLower, magentaUpper, blackLower, blackUpper
    
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    cntsList = [checkContours(hsv, blueLower, blueUpper), checkContours(hsv, redLower, redUpper), 
                checkContours(hsv, greenLower, greenUpper), checkContours(hsv, yellowLower, yellowUpper), 
                checkContours(hsv, magentaLower, magentaUpper), checkContours(hsv, blackLower, blackUpper)]

    for i in range(6):
        if len(cntsList[i]) > 0:
            # The robot has seen the ball
            ballFound = True

            # Cancel the current move_base goal
            movebaseClient.cancel_goal()

            # Save the location name if it hasn't been found yet
            saveLocationName(i)              

            c = max(cntsList[i], key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                vel = Twist()
                vel.angular.z = 0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100)

                # Saturation
                if vel.linear.x > 0.6:
                    vel.linear.x = 0.6
                elif vel.linear.x < -0.6:
                    vel.linear.x = -0.6

                if (abs(vel.linear.x) <= 0.1) and (abs(vel.angular.z) <= 0.1):
                    # Save the location's position
                    saveLocationPosition(i)

                    # Turn the robot approximately 180 degrees 
                    # FIXME check if the robot turns appropriately
                    for x in range(3):
                        vel.angular.z = 0.3
                        velPub.publish(vel)
                        time.sleep(1)

                    # Set the flag to false
                    ballFound = False

                    # Set the flag to true
                    finishedGettingClose.set()

                    return

                velPub.publish(vel)

        else:
            # The robot doesn't see the ball
            ballFound = False


##
# Stores the name of a location if it hasn't already been found
# @param locationNumber The integer corresponding to a location
def saveLocationName(locationNumber):
        if locationNumber == 0:
            if not "Entrance" in detectedLocations:
                detectedLocations.append("Entrance")
        elif locationNumber == 1:
            if not "Closet" in detectedLocations:
                detectedLocations.append("Closet")
        elif locationNumber == 2:
            if not "Living Room" in detectedLocations:
                detectedLocations.append("Living Room")
        elif locationNumber == 3:
            if not "Kitchen" in detectedLocations:
                detectedLocations.append("Kitchen")
        elif locationNumber == 4:
            if not "Bathroom" in detectedLocations:
                detectedLocations.append("Bathroom")
        elif locationNumber == 5:
            if not "Bedroom" in detectedLocations:
                detectedLocations.append("Bedroom") 


##
# Stores the position of a location if it hasn't already been found
# @param locationNumber The integer corresponding to a location
def saveLocationPosition(locationNumber):
        if locationNumber == 0:
            if not "Entrance" in detectedLocations:
                rospy.set_param("entrance_x", robotPosition_x)
                rospy.set_param("entrance_y", robotPosition_y)
        elif locationNumber == 1:
            if not "Closet" in detectedLocations:
                rospy.set_param("closet_x", robotPosition_x)
                rospy.set_param("closet_y", robotPosition_y)
        elif locationNumber == 2:
            if not "Living Room" in detectedLocations:
                rospy.set_param("livingroom_x", robotPosition_x)
                rospy.set_param("livingroom_y", robotPosition_y)
        elif locationNumber == 3:
            if not "Kitchen" in detectedLocations:
                rospy.set_param("kitchen_x", robotPosition_x)
                rospy.set_param("kitchen_y", robotPosition_y)
        elif locationNumber == 4:
            if not "Bathroom" in detectedLocations:
                rospy.set_param("bathroom_x", robotPosition_x)
                rospy.set_param("bathroom_y", robotPosition_y)
        elif locationNumber == 5:
            if not "Bedroom" in detectedLocations:
                rospy.set_param("bedroom_x", robotPosition_x)
                rospy.set_param("bedroom_y", robotPosition_y)


##
# Updates the current robot position in order to save the locations' position
# @param ros_data The odometry information
def updateRobotPosition(ros_data):
    global robotPosition_x, robotPosition_y

    # Update the current robot position
    robotPosition_x = ros_data.pose.pose.position.x
    robotPosition_y = ros_data.pose.pose.position.y


##
# Save the received command in global variables
# @param ros_data The received formatted command
def receivedCommand(ros_data):
    global receivedPlay
    global receivedGoTo
    global parameter
    global logfile

    logfile.write("\nState machine: Received a formatted command.\n")
    logfile.flush()
    os.fsync(logfile)

    # Retrieve the command
    if ros_data.mainCommand == "Play":
        receivedPlay = True
        receivedGoTo = False
    elif ros_data.mainCommand == "GoTo":
        receivedGoTo = True
        receivedPlay = False
        parameter = ros_data.parameter
    else:
        logfile.write("\nUnknown command received: error in state_machine.py.\n")
        logfile.flush()
        os.fsync(logfile)


##
# Send a random position goal to move_base for the NORMAL state
def sendGoalNormalState():
    global movebaseClient
    global mbGoal
    global finishedMoving
    global logfile

    # Get a random location on the plane 
    # FIXME (check the dimension of the map)
    # x = random.randint(-6, 6)
    # y = random.randint(-8, 8)

    x = -4
    y = 7

    # Create the goal
    mbGoal.target_pose.pose.position.x = x
    mbGoal.target_pose.pose.position.y = y

    logfile.write("\nNORMAL state: the robot is moving to position [%d, %d].\n" %(x, y))
    logfile.flush()
    os.fsync(logfile)

    # Send the goal
    movebaseClient.send_goal(mbGoal)

    # Wait for the result
    movebaseClient.wait_for_result()

    logfile.write("\nNORMAL state: the action server returned result %s.\n" %str(movebaseClient.get_state()))
    logfile.flush()
    os.fsync(logfile)

    # Set the flag
    finishedMoving.set()



## TODO
# Define Normal state
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sleep','play'])

        # Threshold
        # self.sleepThreshold = random.randint(5, 10)
        self.sleepThreshold = 3

    def execute(self, userdata):
        global sleepCounter
        global ballFound
        global movebaseClient
        global imageSub, commandSub, odomSub
        global mbGoal
        global receivedPlay
        global finishedMoving, finishedGettingClose
        global logfile

        logfile.write('\nState machine: Executing state NORMAL.\n')
        logfile.flush()
        os.fsync(logfile)

        # Subscribe to the image topic to check if the robot sees the ball
        imageSub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, checkForBall, queue_size=1)

        # Subscribe to the command topic to receive them
        commandSub = rospy.Subscriber("sensor/formatted_command", FormattedCommand, receivedCommand, queue_size=1)

        # Subscribe to the odometry topic to update the current robot position
        odomSub = rospy.Subscriber("odom", Odometry, updateRobotPosition)

        while sleepCounter < self.sleepThreshold:
            # If the user told a "Play" command, switch to the PLAY state
            if receivedPlay:
                # Unsubscribe to the image topic, the command topic and the odometry topic
                imageSub.unregister()
                commandSub.unregister()
                odomSub.unregister()

                receivedPlay = False

                logfile.write('\nNORMAL state: The robot wants to play.\n')
                logfile.flush()
                os.fsync(logfile)

                return 'play'

            # Start the thread to move the robot
            thread = threading.Thread(target=sendGoalNormalState)
            thread.start()

            finishedMoving.wait()
            finishedMoving.clear()

            if ballFound:
                finishedGettingClose.wait()
                finishedGettingClose.clear()

            # Increment the counter
            sleepCounter += 1

        # Unsubscribe to the image topic, the command topic and the odometry topic
        imageSub.unregister()
        commandSub.unregister()
        odomSub.unregister()

        # Go into the SLEEP state
        logfile.write('\nNORMAL state: The robot is sleepy.\n')
        logfile.flush()
        os.fsync(logfile)

        return 'sleep'


## TODO
# Define Sleep state
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wakeup'])

    def execute(self, userdata):
        global sleepCounter
        global movebaseClient
        global mbGoal
        global logfile

        logfile.write('\nState machine: Executing state SLEEP.\n')
        logfile.flush()
        os.fsync(logfile)

        # Get the home location on the plane
        x = -5
        y = 8

        # Create the goal
        mbGoal.target_pose.pose.position.x = x
        mbGoal.target_pose.pose.position.y = y

        # Send the goal
        movebaseClient.send_goal(mbGoal)

        # Wait until the robot has reached the destination
        movebaseClient.wait_for_result()

        logfile.write("\nSLEEP state: the action server returned result %s.\n" %str(movebaseClient.get_state()))
        logfile.flush()
        os.fsync(logfile)

        # Sleep for a random amount of seconds
        #time.sleep(random.randint(10, 15))
        time.sleep(10)

        # Go back to the NORMAL state
        sleepCounter = 0
        logfile.write("\nSLEEP state: The robot woke up.\n")
        logfile.flush()
        os.fsync(logfile)

        return 'wakeup'


## TODO
# Define Play state
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopplaying', 'find'])

    def execute(self, userdata):
        global logfile

        logfile.write('\nState machine: Executing state PLAY.\n')
        logfile.flush()
        os.fsync(logfile)

        # global sleepCounter
        # global ballFound
        # global robotStopped
        # global imageSub
        # global headJointPub
        # global velPub

        # # Subscribe to the image topic to track the ball
        # imageSub = rospy.Subscriber("robot/camera1/image_raw/compressed", CompressedImage, trackBall)

        # while True:
        #     # Check if the velocity of the robot is 0: if so, move the head
        #     if robotStopped == True:
        #         # Keep the robot still
        #         vel = Twist()
        #         vel.angular.z = 0
        #         vel.linear.x = 0
        #         velPub.publish(vel)

        #         angle = Float64()
        #         angle.data = 0.0

        #         print("PLAY state: rotating the camera to the left.\n")

        #         # Move the head to the left and stay there for some seconds
        #         while angle.data < (math.pi/4):
        #             angle.data = angle.data + 0.1
        #             headJointPub.publish(angle)
        #             time.sleep(1)
        #         time.sleep(2)

        #         print("PLAY state: rotating the camera to the right.\n")

        #         # Move the head to the right and stay there for some seconds
        #         while angle.data > -(math.pi/4):
        #             angle.data = angle.data - 0.1
        #             headJointPub.publish(angle)
        #             time.sleep(1)
        #         time.sleep(2)

        #         print("PLAY state: rotating the camera to the center.\n")

        #         # Move the head to the center
        #         while angle.data < 0:
        #             angle.data = angle.data + 0.1
        #             headJointPub.publish(angle)
        #             time.sleep(1)

        #         print("PLAY state: finished rotating the camera.\n")

        #         robotStopped = False
        #         sleepCounter += 1

        #     # If the robot doesn't see the ball for a certain period of time then go back to the NORMAL state
        #     if ballFound == False:
        #         startingTime = time.time()

        #     while ballFound == False:
        #         if (time.time() - startingTime) >= 10:
        #             robotStopped = False

        #             # Unsubscribe to the image topic
        #             imageSub.unregister()

        #             # Go back to the NORMAL state
        #             print("PLAY state: The robot hasn't seen the ball for a while, so it stops playing.\n")
        #             return 'stopplaying'
                    
        time.sleep(1)
        

## TODO
# Define Find state
class Find(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopsearching'])

    def execute(self, userdata):
        global logfile

        logfile.write('\nState machine: Executing state FIND.\n')
        logfile.flush()
        os.fsync(logfile)

        return 'stopsearching'


##
# State machine initialization
if __name__ == "__main__":
    rospy.init_node('robot_behaviour', anonymous=True)

    # Open the log file to write on it
    script_path = os.path.abspath(__file__) 
    path_list = script_path.split(os.sep)
    script_directory = path_list[0:len(path_list)-2]
    file_path = "log/logfile.txt"
    path = "/".join(script_directory) + "/" + file_path
    logfile = open(path, 'a')

    logfile.write("\nState machine: Trying to connect to the move_base server.\n")
    logfile.flush()
    os.fsync(logfile)

    # Create the action client and wait for the server
    movebaseClient = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    movebaseClient.wait_for_server()

    logfile.write("\nState machine: Connected to the move base server.\n")
    logfile.flush()
    os.fsync(logfile)

    # Initialize the publishers
    velPub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    initPosPub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

    # Publish the initial position
    initialPos = PoseWithCovarianceStamped()
    initialPos.pose.pose.position.x = -5
    initialPos.pose.pose.position.y = 8
    initialPos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

    initPosPub.publish(initialPos)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),
                                transitions={'sleep': 'SLEEP',
                                             'play': 'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(),
                                transitions={'wakeup': 'NORMAL'})

        smach.StateMachine.add('PLAY', Play(),
                                transitions={'stopplaying': 'NORMAL',
                                             'find': 'FIND'})
        
        smach.StateMachine.add('FIND', Find(),
                                transitions={'stopsearching': 'PLAY'})                                     

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()