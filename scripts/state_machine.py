#!/usr/bin/env python

## @package state_machine
# Defines the different robot behaviours and the transitions between them.
# Available states are NORMAL, SLEEP, PLAY and FIND.

import rospy
import smach
import smach_ros
import time
import random
import actionlib
import cv2
import imutils
import threading
import os
import numpy as np
from scipy.ndimage import filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from erl_third_assignment.msg import FormattedCommand


## Action client
movebaseClient = None

## Publisher
velPub = None

## Subscriber
imageSub = None
## Subscriber
odomSub = None
## Subscriber
commandSub = None

## Goal pose
mbGoal = MoveBaseGoal()

## Counter 
sleepCounter = 0

## Command variable
receivedPlay = False
## Command variable
parameter = None
## Command variable
requestedLocation = None

## Flag to notify that the robot has seen the ball
ballFound = False

## Threading event
finishedMoving = threading.Event()
## Threading event
finishedGettingClose = threading.Event()
## Threading event
stopExploring = threading.Event()
## Threading event
receivedGoTo = threading.Event()

# Color masks
blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)
redLower = (0, 225, 50)
redUpper = (5, 255, 255)
greenLower = (50, 50, 50)
greenUpper = (70, 255, 255)
yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)
magentaLower = (135, 150, 50)
magentaUpper = (150, 255, 255)
blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)

## Detected locations list
detectedLocations = []
## Saved locations list
savedLocations = []

## Current robot's x position
robotPosition_x = None
## Current robot's y position
robotPosition_y = None

## Log file
logfile = None


##
# Computes the contours of colored objects eventually present in the image.
# @param image The preprocessed image in HSV format.
# @param maskLower The lower bound mask of the color of interest.
# @param maskUpper The upper bound mask of the color of interest.
def checkContours(image, maskLower, maskUpper):
    mask = cv2.inRange(image, maskLower, maskUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    return cnts


## 
# Checks if there's a ball and eventually moves the robot closer to it (TRACK sub-state) to save the corresponding location's position.
# @param ros_data The compressed image picked up by the camera.
def checkForBall(ros_data):
    global ballFound
    global finishedGettingClose
    global savedLocations, requestedLocation
    global movebaseClient
    global stopExploring
    global velPub
    global blueLower, blueUpper, redLower, redUpper, greenLower, greenUpper, yellowLower, yellowUpper, magentaLower, magentaUpper, blackLower, blackUpper
    
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    cntsList = [checkContours(hsv, blueLower, blueUpper), checkContours(hsv, redLower, redUpper), 
                checkContours(hsv, greenLower, greenUpper), checkContours(hsv, yellowLower, yellowUpper), 
                checkContours(hsv, magentaLower, magentaUpper), checkContours(hsv, blackLower, blackUpper)]

    # List of the locations to check
    locationsToCheck = list(set(range(6)) - set(savedLocations))

    for i in locationsToCheck:
        if len(cntsList[i]) > 0:
            # The robot has seen a ball, set the flag to true
            ballFound = True

            # Log the location name if it hasn't been found yet
            foundLocation(i)              

            c = max(cntsList[i], key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 20:
                # Cancel the current move_base goal
                movebaseClient.cancel_goal()

                # If the robot is in the FIND state, stop the explore-lite package
                if requestedLocation != None and not stopExploring.is_set():
                    stopExploring.set()

                # Move the robot towards the colored ball
                vel = Twist()
                vel.angular.z = -0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-150)

                # Saturation
                if vel.linear.x > 0.3:
                    vel.linear.x = 0.3
                elif vel.linear.x < -0.3:
                    vel.linear.x = -0.3

                if (abs(vel.linear.x) <= 0.1) and (abs(vel.angular.z) <= 0.1):
                    # Stop the robot
                    vel.angular.z = 0
                    vel.linear.x = 0
                    velPub.publish(vel)

                    # Save the location's position
                    saveLocationPosition(i)

                    # Set the flag to false
                    ballFound = False

                    # Set the flag
                    finishedGettingClose.set()

                    return

                velPub.publish(vel)
            
            return


##
# Writes on the logfile that a previously undetected location has been found.
# @param locationNumber The integer corresponding to a location.
def foundLocation(locationNumber):
    global robotPosition_x, robotPosition_y
    global detectedLocations

    if locationNumber in detectedLocations:
        return

    elif locationNumber == 0:
        detectedLocations.append(0)

        logfile.write("\n[%f] State machine: The robot found the Entrance.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 1:
        detectedLocations.append(1)

        logfile.write("\n[%f] State machine: The robot found the Closet.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 2:
        detectedLocations.append(2)

        logfile.write("\n[%f] State machine: The robot found the Living Room.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 3:
        detectedLocations.append(3)

        logfile.write("\n[%f] State machine: The robot found the Kitchen.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 4:
        detectedLocations.append(4)

        logfile.write("\n[%f] State machine: The robot found the Bathroom.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 5:
        detectedLocations.append(5) 

        logfile.write("\n[%f] State machine: The robot found the Bedroom.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)


##
# Stores the position of a location.
# @param locationNumber The integer corresponding to a location.
def saveLocationPosition(locationNumber):
    global robotPosition_x, robotPosition_y
    global savedLocations

    if locationNumber == 0:
        rospy.set_param("entrance_x", robotPosition_x)
        rospy.set_param("entrance_y", robotPosition_y)

        # The robot must not check this location anymore
        savedLocations.append(0)

        logfile.write("\n[%f] State machine: Stored the Entrance location [%f, %f].\n" %(time.time(), robotPosition_x, robotPosition_y))
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 1:
        rospy.set_param("closet_x", robotPosition_x)
        rospy.set_param("closet_y", robotPosition_y)

        # The robot must not check this location anymore
        savedLocations.append(1)

        logfile.write("\n[%f] State machine: Stored the Closet location [%f, %f].\n" %(time.time(), robotPosition_x, robotPosition_y))
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 2:
        rospy.set_param("livingroom_x", robotPosition_x)
        rospy.set_param("livingroom_y", robotPosition_y)

        # The robot must not check this location anymore
        savedLocations.append(2)

        logfile.write("\n[%f] State machine: Stored the Living Room location [%f, %f].\n" %(time.time(), robotPosition_x, robotPosition_y))
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 3:
        rospy.set_param("kitchen_x", robotPosition_x)
        rospy.set_param("kitchen_y", robotPosition_y)

        # The robot must not check this location anymore
        savedLocations.append(3)

        logfile.write("\n[%f] State machine: Stored the Kitchen location [%f, %f].\n" %(time.time(), robotPosition_x, robotPosition_y))
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 4:
        rospy.set_param("bathroom_x", robotPosition_x)
        rospy.set_param("bathroom_y", robotPosition_y)

        # The robot must not check this location anymore
        savedLocations.append(4)

        logfile.write("\n[%f] State machine: Stored the Bathroom location [%f, %f].\n" %(time.time(), robotPosition_x, robotPosition_y))
        logfile.flush()
        os.fsync(logfile)

    elif locationNumber == 5:
        rospy.set_param("bedroom_x", robotPosition_x)
        rospy.set_param("bedroom_y", robotPosition_y)

        # The robot must not check this location anymore
        savedLocations.append(5)

        logfile.write("\n[%f] State machine: Stored the Bedroom location [%f, %f].\n" %(time.time(), robotPosition_x, robotPosition_y))
        logfile.flush()
        os.fsync(logfile)


##
# Updates the current robot position in the plane.
# @param ros_data The odometry information.
def updateRobotPosition(ros_data):
    global robotPosition_x, robotPosition_y

    # Update the current robot position
    robotPosition_x = ros_data.pose.pose.position.x
    robotPosition_y = ros_data.pose.pose.position.y


##
# Notifies that the state machine received a formatted command by raising the correct flags.
# @param ros_data The received formatted command.
def receivedCommand(ros_data):
    global receivedPlay, receivedGoTo
    global parameter
    global logfile

    logfile.write("\n[%f] State machine: Received a formatted command.\n" %time.time())
    logfile.flush()
    os.fsync(logfile)

    locations = {
        "Entrance": 0,
        "Closet": 1,
        "Living Room": 2,
        "Kitchen": 3,
        "Bathroom": 4,
        "Bedroom": 5
    }

    # Raise the correct flags depending on the command
    if ros_data.mainCommand == "Play":
        receivedPlay = True
        receivedGoTo.clear()
        parameter = None
    elif ros_data.mainCommand == "GoTo":
        receivedGoTo.set()
        receivedPlay = False
        parameter = locations.get(ros_data.parameter)
    else:
        logfile.write("\n[%f] Unknown command received: error in state_machine.py.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)


##
# Sends a random position goal to move_base for the NORMAL state and wait for its result.
def sendGoalNormalState():
    global movebaseClient
    global mbGoal
    global finishedMoving
    global logfile

    # Get a random location on the plane (may be out of the environment bounds)
    x = random.randint(-6, 6)
    y = random.randint(-8, 8)

    # Create the goal
    mbGoal.target_pose.header.frame_id = "map"
    mbGoal.target_pose.header.stamp = rospy.Time.now()
    mbGoal.target_pose.pose.position.x = x
    mbGoal.target_pose.pose.position.y = y
    mbGoal.target_pose.pose.position.z = 0
    mbGoal.target_pose.pose.orientation.z = 0.1
    mbGoal.target_pose.pose.orientation.w = 0.1

    logfile.write("\n[%f] NORMAL state: The robot is moving to position [%f, %f].\n" %(time.time(), x, y))
    logfile.flush()
    os.fsync(logfile)

    # Send the goal
    movebaseClient.send_goal(mbGoal)

    # Wait for the result
    movebaseClient.wait_for_result(rospy.Duration(80))

    logfile.write("\n[%f] NORMAL state: The action server returned with state %s.\n" %(time.time(), str(movebaseClient.get_state())))
    logfile.flush()
    os.fsync(logfile)

    # Set the flag
    finishedMoving.set()


##
# Define Normal state.
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sleep','play'])

        # Threshold
        self.sleepThreshold = 3

    def execute(self, userdata):
        global ballFound
        global sleepCounter
        global movebaseClient
        global imageSub, commandSub, odomSub
        global mbGoal
        global receivedPlay
        global finishedMoving, finishedGettingClose
        global logfile

        logfile.write('\n[%f] State machine: Executing state NORMAL.\n' %time.time())
        logfile.flush()
        os.fsync(logfile)

        # Subscribe to the image topic
        imageSub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, checkForBall)

        # Subscribe to the command topic
        commandSub = rospy.Subscriber("sensor/formatted_command", FormattedCommand, receivedCommand)

        # Subscribe to the odometry topic
        odomSub = rospy.Subscriber("odom", Odometry, updateRobotPosition)

        while sleepCounter < self.sleepThreshold:
            # If the user told a "Play" command, switch to the PLAY state
            if receivedPlay:
                # Unsubscribe to the topics
                imageSub.unregister()
                commandSub.unregister()
                odomSub.unregister()

                receivedPlay = False

                logfile.write('\n[%f] NORMAL state: The robot wants to play.\n' %time.time())
                logfile.flush()
                os.fsync(logfile)

                return 'play'

            # Start the thread to move the robot
            thread = threading.Thread(target=sendGoalNormalState)
            thread.start()

            # Wait until the action server returns a result
            finishedMoving.wait()
            finishedMoving.clear()

            # If a ball was found while moving, wait until the robot has gotten close enough to it
            if ballFound:
                finishedGettingClose.wait()
                finishedGettingClose.clear()

            # Increment the counter
            sleepCounter += 1

        # Unsubscribe to the topics
        imageSub.unregister()
        commandSub.unregister()
        odomSub.unregister()

        # Go into the SLEEP state
        logfile.write('\n[%f] NORMAL state: The robot is sleepy.\n' %time.time())
        logfile.flush()
        os.fsync(logfile)

        return 'sleep'


##
# Define Sleep state.
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wakeup'])

    def execute(self, userdata):
        global sleepCounter
        global movebaseClient
        global mbGoal
        global logfile

        logfile.write('\n[%f] State machine: Executing state SLEEP.\n' %time.time())
        logfile.flush()
        os.fsync(logfile)

        # Get the home location on the plane
        x = rospy.get_param("home_x")
        y = rospy.get_param("home_y")

        # Create the goal
        mbGoal.target_pose.header.frame_id = "map"
        mbGoal.target_pose.header.stamp = rospy.Time.now()
        mbGoal.target_pose.pose.position.x = x
        mbGoal.target_pose.pose.position.y = y
        mbGoal.target_pose.pose.position.z = 0
        mbGoal.target_pose.pose.orientation.z = 0.1
        mbGoal.target_pose.pose.orientation.w = 0.1

        logfile.write("\n[%f] SLEEP state: The robot is going home.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

        # Send the goal
        movebaseClient.send_goal(mbGoal)

        # Wait until the robot has reached the destination
        movebaseClient.wait_for_result(rospy.Duration(180))

        logfile.write("\n[%f] SLEEP state: The action server returned with state %s.\n" %(time.time(), str(movebaseClient.get_state())))
        logfile.flush()
        os.fsync(logfile)

        # Sleep for some time
        #time.sleep(random.randint(10, 15))
        time.sleep(10)

        # Reset the sleep counter back to 0
        sleepCounter = 0

        # Go back to the NORMAL state
        logfile.write("\n[%f] SLEEP state: The robot woke up.\n" %time.time())
        logfile.flush()
        os.fsync(logfile)

        return 'wakeup'


##
# Sends a goal to the move_base for the PLAY state and waits for its result.
# @param x The x-coordinate of the location to go to.
# @param y The y-coordinate of the location to go to.
def sendGoalPlayState(x, y):
    global mbGoal
    global movebaseClient
    global logfile

    # Create the goal
    mbGoal.target_pose.header.frame_id = "map"
    mbGoal.target_pose.header.stamp = rospy.Time.now()
    mbGoal.target_pose.pose.position.x = x
    mbGoal.target_pose.pose.position.y = y
    mbGoal.target_pose.pose.position.z = 0
    mbGoal.target_pose.pose.orientation.z = 0.1
    mbGoal.target_pose.pose.orientation.w = 0.1

    logfile.write("\n[%f] PLAY state: The robot is moving to position [%f, %f].\n" %(time.time(), x, y))
    logfile.flush()
    os.fsync(logfile)

    # Send the goal
    movebaseClient.send_goal(mbGoal)

    # Wait until the robot has reached the destination
    movebaseClient.wait_for_result(rospy.Duration(180))

    logfile.write("\n[%f] PLAY state: The action server returned with state %s.\n" %(time.time(), str(movebaseClient.get_state())))
    logfile.flush()
    os.fsync(logfile)


##
# Define Play state.
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopplaying', 'find'])

    def execute(self, userdata):
        global logfile
        global sleepCounter
        global savedLocations
        global receivedGoTo
        global parameter
        global commandSub
        global requestedLocation

        logfile.write('\n[%f] State machine: Executing state PLAY.\n' %time.time())
        logfile.flush()
        os.fsync(logfile)

        # Subscribe to the command topic
        commandSub = rospy.Subscriber("sensor/formatted_command", FormattedCommand, receivedCommand)

        # Get the starting time of the PLAY state
        startingTime = time.time()

        # Stay in the PLAY state for some time
        while (time.time() - startingTime) <= 180:
            # Get the human location on the plane
            x = rospy.get_param("human_x")
            y = rospy.get_param("human_y")

            logfile.write("\n[%f] PLAY state: The robot is reaching the Person.\n" %time.time())
            logfile.flush()
            os.fsync(logfile)

            sendGoalPlayState(x, y)

            # Wait for a "GoTo: Location" command
            receivedGoTo.wait()
            receivedGoTo.clear()

            if not parameter in savedLocations:
                # If the requested location's position hasn't been saved yet, switch to the FIND state
                commandSub.unregister()
                requestedLocation = parameter

                logfile.write("\n[%f] PLAY state: The requested location's position is currently unknown, switching to the FIND state.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

                return 'find'

            elif parameter == 0:
                # Get the entrance location
                x = float(rospy.get_param("entrance_x"))
                y = float(rospy.get_param("entrance_y"))

                logfile.write("\n[%f] PLAY state: The robot is reaching the Entrance.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

            elif parameter == 1:
                # Get the closet location
                x = float(rospy.get_param("closet_x"))
                y = float(rospy.get_param("closet_y"))

                logfile.write("\n[%f] PLAY state: The robot is reaching the Closet.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

            elif parameter == 2:
                # Get the living room location
                x = float(rospy.get_param("livingroom_x"))
                y = float(rospy.get_param("livingroom_y"))

                logfile.write("\n[%f] PLAY state: The robot is reaching the Living Room.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

            elif parameter == 3:
                # Get the kitchen location
                x = float(rospy.get_param("kitchen_x"))
                y = float(rospy.get_param("kitchen_y"))

                logfile.write("\n[%f] PLAY state: The robot is reaching the Kitchen.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

            elif parameter == 4:
                # Get the bathroom location
                x = float(rospy.get_param("bathroom_x"))
                y = float(rospy.get_param("bathroom_y"))

                logfile.write("\n[%f] PLAY state: The robot is reaching the Bathroom.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

            elif parameter == 5:
                # Get the bedroom location
                x = float(rospy.get_param("bedroom_x"))
                y = float(rospy.get_param("bedroom_y"))

                logfile.write("\n[%f] PLAY state: The robot is reaching the Bedroom.\n" %time.time())
                logfile.flush()
                os.fsync(logfile)

            sendGoalPlayState(x, y)

            # Increment the counter
            sleepCounter += 1

        # After a while, go back to the NORMAL state
        commandSub.unregister()

        return 'stopplaying'
        

##
# Starts the explore-lite package. 
def startExploreLite():
    global logfile

    logfile.write('\n[%f] FIND state: Starting the explore-lite package.\n' %time.time())
    logfile.flush()
    os.fsync(logfile)

    os.system("roslaunch erl_third_assignment explore.launch")


##
# Stops the explore-lite package.
def stopExploreLite():
    global logfile

    logfile.write('\n[%f] FIND state: Stopping the explore-lite package.\n' %time.time())
    logfile.flush()
    os.fsync(logfile)

    os.system("rosnode kill /explore")


##
# Define Find state.
class Find(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopsearching'])

    def execute(self, userdata):
        global logfile
        global imageSub, odomSub
        global movebaseClient
        global finishedGettingClose
        global stopExploring
        global savedLocations, requestedLocation

        logfile.write('\n[%f] State machine: Executing state FIND.\n' %time.time())
        logfile.flush()
        os.fsync(logfile)

        # Subscribe to the image topic
        imageSub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, checkForBall)

        # Subscribe to the odometry topic
        odomSub = rospy.Subscriber("odom", Odometry, updateRobotPosition)

        # Get the starting time of the FIND state
        startingTime = time.time()

        # Stay in the FIND state for some time
        while (time.time() - startingTime) <= 180:
            # Launch the thread to start the explore-lite package
            thread = threading.Thread(target=startExploreLite)
            thread.start()

            # Wait for the robot to find a ball
            stopExploring.wait()

            # Launch the thread to stop the explore-lite package
            thread = threading.Thread(target=stopExploreLite)
            thread.start()

            movebaseClient.cancel_all_goals()

            # Wait for the robot to get close to a colored ball
            finishedGettingClose.wait()
            finishedGettingClose.clear()

            stopExploring.clear()

            # If the colored ball that has been found corresponds to the requested location, go back to the PLAY state
            if requestedLocation in savedLocations:
                # Unsubscribe to the topics
                imageSub.unregister()
                odomSub.unregister()

                requestedLocation = None

                logfile.write('\n[%f] FIND state: The robot found the requested location and is going back to the PLAY state.\n' %time.time())
                logfile.flush()
                os.fsync(logfile)
                
                return 'stopsearching'

        # Unsubscribe to the topics
        imageSub.unregister()
        odomSub.unregister()

        requestedLocation = None

        logfile.write('\n[%f] FIND state: The robot still has not found the requested location, so it is going back to the PLAY state.\n' %time.time())
        logfile.flush()
        os.fsync(logfile)

        return 'stopsearching'


# State machine initialization
if __name__ == "__main__":
    rospy.init_node('robot_behaviour', anonymous=True)

    # Open the log file to write on it
    script_path = os.path.abspath(__file__) 
    path_list = script_path.split(os.sep)
    script_directory = path_list[0:len(path_list)-2]
    file_path = "log/state_machine_logfile.txt"
    path = "/".join(script_directory) + "/" + file_path
    logfile = open(path, 'w')

    logfile.write("[%f] State machine: Trying to connect to the move_base server.\n" %time.time())
    logfile.flush()
    os.fsync(logfile)

    # Create the action client and wait for the server
    movebaseClient = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    movebaseClient.wait_for_server()

    logfile.write("\n[%f] State machine: Connected to the move base server.\n" %time.time())
    logfile.flush()
    os.fsync(logfile)

    # Initialize the publishers
    velPub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

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