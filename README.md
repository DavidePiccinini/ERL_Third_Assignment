# Experimental Robotics Laboratory Third Assignment


---

## System Architecture

### Component Diagram

<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/Component_Diagram.png">
</p>

The software architecture is based on ****:

- **Person module**

    

- **Finite state machine**

    

### State Diagram

<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/State_Diagram.png">
</p>

This is the state diagram that shows how the finite state machine works:


### rqt_graph

<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/rosgraph1.png">
</p>

<p align="center"> 
<img src="https://github.com/DavidePiccinini/ERL_Third_Assignment/blob/master/diagrams/rosgraph2.png">
</p>

---

## ROS Messages and Parameters

Custom ROS **messages** are:

- **.action**

    ```
    ```

    Describes 


---

## Packages and File List

Going in alphabetical order:

- 

---

## Installation and Running Procedure

First of all, clone this repository inside your ROS workspace's *"/src"* folder .

Then, navigate to the *"/scripts"* folder and make the Python scripts executable with:
```
$ chmod +x person.py
$ chmod +x state_machine.py
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


---

## System's Features



---

## System's Limitations



---

## Possible Technical Improvements



---

## Authors and Contacts

Davide Piccinini matricola S4404040
Emails: 
- piccio98dp@gmail.com
- 4404040@studenti.unige.it

---

## Doxygen Documentation

[Doxygen pdf documentation](https://github.com/DavidePiccinini/ERL_Third_Assignment/tree/master/documentation/latex/refman.pdf)