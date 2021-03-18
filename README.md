# Experimental Robotics Laboratory Third Assignment
The third and last assignment for this course covers all the topics that we faced during the lessons. We need to develop a ROS architecture to make a robot navigate an indoor environment, find colored balls that correspond to specific locations and respond to user commands; the system will be simulated in Gazebo and visualized in Rviz. The robot implements 4 possible states/behaviours, i.e. NORMAL, SLEEP, PLAY and FIND.

---

## System Architecture

### Component Diagram

<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/Component_Diagram.png">
</p>

The software architecture is based on several components and packages:

- **Human component**

    This component simulates a user that gives commands to the robot: it implements a simple publisher on the `/command` topic, where "Play" or "GoTo: Location" commands are sent. 
&nbsp;

- **Sensor component**

    This component represents a robot's sensor which understands the commands told by the user, translates them in a specific format and sends them on the `/sensor/formatted_command` topic, where they are picked up by the state machine.
&nbsp;

- **Finite state machine component**

    The finite state machine is very complex because it defines the logic behind states and their transitions. It retrieves a lot of information, such as formatted commands from the sensor, and communicates with the *"move_base"* package, the *"explore-lite"* package and the simulated robot.
&nbsp;

- **move_base package**

    It's a fundamental component of the ROS navigation stack: it implements an action server, a local and a global planner which are used to move the robot to achieve a goal pose, considering the surrounding environment.
&nbsp;

- **Gmapping package**

    It provides a SLAM approach based on laser scans. This package produces a map which is used by *"explore-lite"*.
&nbsp;

- **explore-lite package**

    It's important for autonomous navigation since it allows to explore unknown environment: it uses frontiers to represent unexplored space and sends goals to the *"move_base"* package to reach them.

### State Diagram

<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/State_Diagram.png">
</p>

This is the state diagram that shows how the finite state machine works. There are a total of 4 states.

In the **NORMAL** behaviour the robot moves randomly inside the house by sending goals to the *"move_base"* package: since the package considers the surrounding environment, the robot will avoid crashing into obstacles. If the camera sees a colored ball (which corresponds to a location) and the robot is close enough to it, the behaviour switches to the "TRACK" sub-state, in which the robot gets near the ball to store the associated location's position. If the user tells a "Play" command, the state transitions to "PLAY". After performing several actions, the state transitions to "SLEEP".  

In the **SLEEP** state the robot reaches the predefined "home" position [-5, -4], stays there for some time and then returns to the "NORMAL" state.

In the **PLAY** state the robot reaches the user position [-5, 8] and waits for a "GoTo: Location" command. Once received, there are two possibilities: if the requested location's position is already saved, reach it and then go back to the user to wait for another "GoTo: Location" command; otherwise switch to the "FIND" state. After some time spent in the "PLAY" behaviour, go back to the "NORMAL" state.

In the **FIND** state the finite state machine starts the *"explore-lite"* package to navigate to the unknown environment: if the camera sees a colored ball, the behaviour switches again to the "TRACK" sub-state. If the detected ball corresponds to the requested location, go back to the "PLAY" state; otherwise keep exploring. If some time has passed and the requested location still hasn't been found, go back to the "PLAY" state. 

### rqt_graph

Here are two graphs representing the system's configuration in two different time instances.

Here the system is in the "PLAY" state.
<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/rosgraph1.png">
</p>

Here the system is in the "FIND" state since we can see that the *"explore-lite"* package is active.
<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/rosgraph2.png">
</p>

---

## ROS Messages and Parameters

The only **custom ROS message** I implemented is:

- **FormattedCommand.msg**

    ```
    string mainCommand
    string parameter
    ```

    It describes the formatted command that will be sent to the finite state machine: `mainCommand` can be either "Play" or "GoTo" and `parameter` represents the requested location of a "GoTo: Location" command.

I introduced several **custom ROS parameters**:

- `home_x` and `home_y`

    The position of the "home" location on the plane, defined in the launchfile.
&nbsp;

- `human_x` and `human_y`

    The position of the user on the plane, defined in the launchfile.
&nbsp;

- `entrance_x`, `entrance_y`, `closet_x`, `closet_y`, `livingroom_x`, `livingroom_y`, `kitchen_x`, `kitchen_y`, `bathroom_x`, `bathroom_y`, `bedroom_x `and `bedroom_y`

    The position of key locations on the plane, defined in the finite state machine.

---

## Packages and File List

Going in alphabetical order:

- **config**

    - `sim.rviz`

        The custom configuration file that sets up the Rviz environment.
&nbsp;

- **diagrams**

    - `Component_Diagram.png`, `State_Diagram.png`, `rosgraph1.png` and `rosgraph2.png`

        The pictures used in this README file.
&nbsp;

- **documentation**

    - `assignment.conf`

        Configuration file used to run "Doxygen". 
&nbsp;

    - `refman.pdf`

        The documentation file obtained with "Doxygen".
&nbsp;

- **launch**

    - `explore.launch`, `gmapping.launch`, `move_base.launch` and `simulation.launch`

        The main launch file is `simulation.launch`, which starts all the nodes and all other launch files.
&nbsp;

- **log**

    - `human_logfile.txt`, `sensor_logfile.txt` and `state_machine.txt`

        Since the terminal is full of Rviz and Gazebo messages, the important events that happen during the system's execution are written in these logfiles with timestamps.
&nbsp;

- **msg**

    - `FormattedCommand.msg`

        The custom ROS message described beforehand.
&nbsp;

- **param**

    - `base_local_planner_params.yaml`, `costmap_common_params.yaml`, `global_costmap_params.yaml`, `local_costmap_params.yaml` and `move_base_params.yaml`

        The yaml files that describe the parameters used by the *"move_base"* package.
&nbsp;

- **scripts**

    - `human.py`, `sensor.py` and `state_machine.py`

        The python programs that run their respective ROS nodes which have been described in the **System Architecture** section.
&nbsp;

- **urdf**

    - `human.urdf`, `robot_laser_camera.gazebo` and `robot_laser_camera.xacro`

        The urdf and xacro files define the human and the robot models respectively, the gazebo file defines the robot plugins and type of control.
&nbsp;

- **worlds**

    - `house2.world`

        The file that defines the environment to be simulated in Gazebo.

---

## Installation and Running Procedure

First of all, clone this repository inside your ROS workspace's *"/src"* folder .

Then, navigate to the *"/scripts"* folder and make the Python scripts executable with:
```
$ chmod +x human.py
$ chmod +x sensor.py
$ chmod +x state_machine.py
```

If you don't have them, install the *"Gmapping"*, *"move_base"* and *"explore-lite"* packages. You can do it by executing:
```
sudo apt-get install ros-<ros_distro>-openslam-gmapping
sudo apt-get install ros-<ros_distro>-navigation
sudo apt-get install ros-<ros_distro>-explore-lite
```

Go back to the root folder of your ROS workspace and execute:
```
$ catkin_make
$ catkin_make install
```

In a separate terminal run:
```
$ roscore
```

Finally, run the launchfile with this command:
```
$ roslaunch erl_third_assignment simulation.launch
```

At this point the system will setup by launching Rviz, Gazebo and all other nodes. At the end of the setup, the robot will start moving: I suggest you to look contemporaneously at Rviz and at the three logfiles to understand what is happening.

---

## System's Features

The architecture features a sensor component, which represents an interface between the user and the robot: new commands can be implemented quite easily by modifying part of the sensor code and part of the state machine code.
The *"move_base"* package allows the robot to move in the environment avoiding obstacles and without crashing into walls.
The human component sends the command randomly, without a specific order: the robot is able to process them with no problems, which denotes the robustness of the system.
The system stores a location's position only when the robot is very close to the ball and is still, so the measurement is accurate.
The Rviz interface allows to easily understand where the robot is going, what path is it taking and what's the known environment: combined with the logfiles, they represent all the information needed to understand the current state of the system.

---

## System's Limitations

When the robot reaches a location, usually the *"move_base"* action server takes some time to send the success result and so the robot stays idle for some seconds.
In the "NORMAL" state the goal may be set to an out of bounds position.
The robot can process "Play" commands in the "NORMAL" state only when it isn't moving: the last command told by the user is stored until a new one is received, so if the user tells a "Play" command while the robot is reaching a location and then tells a "GoTo: Location" one, once the robot is able to process commands it won't go into the "PLAY" state.
Rviz shows the current goal position, the global and local path of the robot, but if the goal is cancelled (i.e. when the robot goes in the "TRACK" sub-state) Rviz doesn't stop showing them, which can cause confusion. The same happens when visualizing *"explore-lite"* 's frontiers.

---

## Possible Technical Improvements

The only two technical improvements I can think of regarding my implementation would be:
- allow the user to choose between a command-reactive robot or the current approach, since they could prefer the robot to switch immediately to the "PLAY" state instead of waiting for it to reach its destination;
&nbsp;

- in the "NORMAL" state, implement a way to send only random goal locations that are not out of bounds.

---

## Authors and Contacts

Davide Piccinini matricola S4404040
Emails: 
- piccio98dp@gmail.com
- 4404040@studenti.unige.it

---

## Doxygen Documentation

[Doxygen pdf documentation](https://github.com/DavidePiccinini/ERL_Third_Assignment/tree/master/documentation/refman.pdf)