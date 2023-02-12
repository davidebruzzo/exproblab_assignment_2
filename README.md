Second Assignment, FSM - Experimental Robotics
================================
**The second ROS-based assignment for the Experimental Robotics Laboratory course held at the University of Genoa.**

*Code documentation can be found [here](https://davidebruzzo.github.io/exproblab_assignment_2/).*

Author: [*Davide Bruzzo (S4649449)*](mailto:davide.brzo@gmail.com?subject=[GitHub]%20Source%20Han%20Sans)

****************************
## Introduction

This repository contains my solution of the 2nd assignment of Experimental Laboratory class which I made on ROS-based software to simulate a robot's behaviour.

The assignment required to integrate the architecture developed in the first assignment, which I developed and can be found [*here*](https://github.com/davidebruzzo/assignment_FiniteStateMachine), with a robotic simulation.

To start, we have been provided with a package, which contains:

- The definition of a custom message and a custom service
- A simulation environment representing the "house" to be monitored
- A node that implements a service: it requires the id (marker) detected by the robot and it replies with the information about the corresponding room (name of the room, coordinates of the center, connections with other rooms)
- A launch file, which starts Gazebo with the simulation environment, and the service node (assignment.launch).

In particular we were required to:
- Spawn the robot in the initial position x = -6.0, y = 11.0
- Build the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it, by calling the provided service node.
- Start the patrolling algorithm by relying on autonomous navigation strategies (mapping/planning) and on the information collected and stored in the ontology during the previous step.
- When a room is reached, perform a complete scan of the room (by rotating the base or the camera).

Since this assignment is to be integrated with the previous one, the main controller of the robot's behaviour remains the finite state machine implemented previously.
In detail, the robot continues to interact with the ontology to receive the list of reachable locations and urgent rooms. The reasoning algorithm for choosing the next destination remains the same. The only changes that have been made and will be presented later are for integration and better scalability.
In this case, the planning and controlling nodes have been replaced by the actual robot behaviour in the simulation environment and thus in practice implemented directly in simulation.
Additionally, the battery management system needs to be put in place. The robot should navigate to a specific area (the charging room) whenever its battery level is low and, once there, it should imitate the battery recharging procedure for a few seconds.


### Scenario

The scenario involves a robot deployed in a indoor environment for surveillance purposes.

The robot’s objective is to visit the different locations and stay there for some times.

### Environment

<p align="center">
<img src="https://github.com/davidebruzzo/exproblab_assignment_2/blob/main/imgs/ambiente.png" width="500" />
<p>
The simulation environment in this case consists of a .world file in which there is a map that imitates the composition of an environment similar to the one found in the ontology. The robot, as per specifications, must be at the requested coordinates. These place it in a room that is not represented within the ontology because it is the one containing the aruco markers. These markers contain information about the various rooms and it is the robot's job to recognize them in order to build the ontology. It is important to note that I have changed the background color of the boxes on which the markers rest. This was made necessary by the robot's difficulty in recognizing and identifying the markers with a gray background.

<p align="center">
<img src="https://github.com/davidebruzzo/exproblab_assignment_2/blob/main/imgs/inizio stanza.png" width="700" />
<p>

## Software architecture

### Component diagram
<p align="center">
<img src="https://github.com/davidebruzzo/exproblab_assignment_2/blob/main/imgs/component.png" width="800" />
<p>

As you can see, the diagram is made up of various nodes:
- Armor service, node that deals with interacting with the ontology, exchanging information with it and modifying it through the appropriate instructions to keep it updated
- Battery, a node that simulates battery life and its subsequent recharge
- Detection_node, node that moves the robot arm to identify all the markers, through the use of computer vision algorithms. Once obtained the marker ids are sent to the server. The response is taken and sent to the state machine helper as a message.
- Marker server, node that implements a server that provides a contextual response to a request containing the ID of an aruco marker, with information about the room.
- Gazebo, a simulation environment that allows the robot to simulate camera vision, move through Path planning algorithms and obtain information on the positions and speeds of the robot's components.
- Assignment_fsm, node that implements the finite state machine, composed of the same states as the one of the last assignment. It takes care of calling the right functions from the helper interface.

  #### Ros messages and parameters
  
  To link all the nodes and make them communicating each other have been used some services and messages:
  
  - Messages:
  
    - InfoRoom.msg:
        ```cpp
        string room
        float32 x
        float32 y
        RoomConnection[] connections
        ``` 
     
     - RoomConnection:
        ```cpp
        string connected_to
        string trhough_door
        ```
  - Service:
  
    - RoomInformation.srv:
       ```cpp
        int32 id
        ---
        string room
        float32 x
        float32 y
        RoomConnection[] connections
       ```
  ### Sequence diagram
  
  The software architecture's sequence diagram, which emphasizes the timing of communication between nodes, is here below displayed. 
  If no battery low signal is sent, it specifically shows the communication and computation flow beginning at the moment the architecture is launched. 
  The state machine node receives this message and this causes the recharge mechanism to begin.
  The recharge method is quite similar to the one that is described immediately following the formation of the environment in the sequence diagram. 
  The robot just waits for the battery to recharge for a predetermined amount of time once it has arrived in the charging chamber, which is the only significant change.
  
 <p align="center">
<img src="https://github.com/davidebruzzo/exproblab_assignment_2/blob/main/imgs/sequence diagram.png" width="800" />
<p> 
  
  It demonstrates that the finite state machine node, which is expected given that it is the hub that calls the other components, and the battery node, which continuously monitors the battery level, are the nodes that are always active.  In addition, the battery node runs on a separate thread. It publishes a boolean value when the robot has low battery, it returns to FSM to recharge the robot.
 
  As seen in the image, the ```detection_node``` regulates the joints of the arm to enable the camera to detect all of the markers that hold environmental data. By calling the aRMOR server, it then creates the environment using the information that was retrieved.
The finite state machine then determines a target location among the sites that the robot may access.
Following that, a request with the desired location's coordinates is sent to the move base action server.
  The fsm node enters in a loop control exited once after the robot has arrived at the desired place, and it changes the robot's location and time stamp.
Finally, the location's time-stamp is updated after the exploration of the site, which involves manipulating the robot's arm to scan the entire area with the camera.
 
  
  ### States diagram

The software that implements the *FSM* is made of 4 states:
- **WaitForOntology**;
- **Recharging**;
- **Plan_To_Urgent**;
- **Visit_Room**.

 <p align="center">
  <img src="https://github.com/davidebruzzo/assignment_FiniteStateMachine/blob/main/images/statemachine.jpg" width="600" />
  <p>
  
  The yellow arrows represent the recursive iteration, they are not written in order to simplify the graph.
  
  The equivalent graph can be obtained by writing these commands on the bash:
  
  ```bash
$ rosrun smach_viewer smach_viewer.py
```
and the result will be this:

<p align="center">
  <img src="https://github.com/davidebruzzo/assignment_FiniteStateMachine/blob/main/images/smach.png" width="300" />
  <p>
   
   Where you can see aswell the recursive ones.
    
    ## Installation and running procedure
   
 For installing this repository:
  - ```Git clone``` into your ROS workspace.
  - Run ```chmod +x``` for every file inside the [script](https://github.com/davidebruzzo/exproblab_assignment_2/tree/main/scripts) folder.
  - Run ``` catkin_make``` to build the workspace.
    
Then it's also needed the ```xterm``` package, by typing this in shell:
```bash 
  sudo apt-get update
  sudo apt-get -y install xterm
```
But you have also to clone:
- [Slam package](https://github.com/CarmineD8/SLAM_packages)
- [Planning](https://github.com/CarmineD8/planning)
- [Aruco](https://github.com/CarmineD8/aruco_ros):
	- Copy the models folder from the aruco ros package to the ```/root/.gazebo/models``` folder so that the markers may subsequently be loaded correctly in the robotic simulation.

Before launching the .launch file, you need to have a folder (*topological_map*) in your ros workspace to retreive this path variable:

```python 
  path = '/root/ros_ws/src/topological_map/'
```
And the file .owl *topological_map.owl* in this directory, in order to make possible to perform this instruction, which is automathically done by ```LoadOntology()``` function:

```python 
  # Initializing with buffered manipulation and reasoning
  client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
```

Now you can use the following commands to launch the simulation:
  - On a terminal:
    ```bash 
      roscore &
      rosrun armor execute it.emarolab.armor.ARMORMainService
    ```
    
   - On another terminal:
     ```bash 
    	 roslaunch assignment2 final.launch
     ```
	   
## Video demo
	  
METTI VIDEOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
	  
## Working hypothesis and system feature
	  
### System's hypothesis

- Only nearby urgent locations to the one it is in can be detected by the robot.
- The Recharging Room (E), is where the robot starts.
- Robot first determines its current location and then the other ones that are reachable. Then it takes the urgent ones and makes a random selection from a list of both reachable and urgent.
- The robot is also presumptively autonomous for random amount of time, from 400 to 450 seconds before requiring recharging. 
- When robot is in E it randomly waits again in the range of seconds (80, 100) to simulate the recharging.
	  
### System's limitations
The first limitation that I found was due to the dimension of the map, since it has only 2 corridors when the rooms are not urgent the robots could ping-pong between the two corridor.

Another one can be the urgency threshold that is set by 7 seconds, this makes propably robot always visit only the two room on right or left when enter in the first one of these two.
	  
## Improvements
Some of the potential upgrades only entail addressing the system's flaws.
An improvement could be, if the robot had to calculate the course, it would have to do it by examining all potential routes and selecting the best one to prevent wasting time with unnecessary movements and battery power.
By taking into account a precise model of a battery, for instance, both the discharge and recharge operations could be done in a more realistic way. Additionally, the most urgent place from a list of all the urgent locations (including those that are not close to the robot position) could be chosen as the target at the end of each cycle. 
To increase the number of markers that can be detected in terms of marker detection, a more thorough method of scanning the robot's surrounds may be used.

Another one could be the implement of errors and fault management and also making the robot remember a route from each place to the charging room would be an enhancement that would prevent it from having to choose randomly to get to a destination in the event of a low charge and would therefore save time.
