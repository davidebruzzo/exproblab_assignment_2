#! /usr/bin/env python3


## @package exp1_assignment
# \file my_helper.py
# \brief This file contains all the functions needed for the assignment_FSM
# \author Davide Bruzzo
# \version 1.0
# \date 18/11/2022
#
# \details
#
#  Subscribes to: <BR>
#   state/battery_low
#
#  Publishes to: <BR>
#   [None]
#
#  Service: <BR>
#   state/set_pose
#   state/get_pose
#
# This file contains all the functions and their implementation needed for the final state machine
#


import rospy, sys
# Import the armor client class
from armor_client import ArmorClient
import time
import random
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock
# Import ROS-based messages.
from std_msgs.msg import Bool
from actionlib import SimpleActionClient
from arch_skeleton.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse
# Import used messages defined within the ROS architecture.
from arch_skeleton.msg import Gesture, Point, Speech, PlanAction, PlanGoal, ControlAction, ControlGoal
from assignment2.msg import RoomConnection, InfoRoom

# A tag for identifying logs producer.
LOG_TAG = 'MY-HELPER'

global roomL, fullList
roomL = []
fullList = []

global doors
doors = []

global doors_no_duplicate
doors_no_duplicate = []

global individuals_list, locationList, doorList
individuals_list = []
locationList = []
doorList = []

global all_markers_detected
all_markers_detected = False

client = ArmorClient("expro2_client","ontology2")

#publisher_battery_flag = rospy.Publisher('/flag_ok_battery', Bool, queue_size=1, latch=True)

class MyHelper:

    ##
    # \class MyHelper
    # \brief This class defines the helper
    #
    # This class define and implements all the methods which have to called in the assignment_FSM file
    #

    def __init__(self):

        ##
        # \brief This function is the constructor of the MyHelper class
        # \param None
        # \return None
        #
        # This function implements the constructor of the MyHelper and also instantiates the services for getting and setting the pose.
        # It defines also the subscriber to the battery topic, and the clients for plan and action servers.
        #

        self.mutex = Lock()


        # Initialise robot position.
        self._pose = None
        self.reset_battery()
        rospy.Subscriber('state/battery_low', Bool, self._battery_callback)



         # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)

        # # Define the clients for the the plan and control action servers.
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)





    def NewLoadOntology(self, path):

        ##
        # \brief This function is called in LoadOntology state
        # \param client that is the ArmorClient, path that is the path where to save the Ontology
        # \return None
        #
        # This function performs all the clients methods in order to create the ontology and adding
        # all the rooms and corridors
        #
        #self.client = client

        global roomL, doors, doors_no_duplicate, individuals_list, all_markers_detected, client

        # Initializing with buffered manipulation and reasoning
        client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

        client.utils.mount_on_ref()
        client.utils.set_log_to_terminal(True)

        rospy.Subscriber("/info_room_sm", InfoRoom, self.NewLoadOntologyCallback)

        # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
        # client.manipulation.disj_inds_of_class("LOCATION")
        # client.manipulation.disj_inds_of_class("DOOR")

        while(len(roomL) <= 7):
            if(all_markers_detected):

                rospy.loginfo("POSSO FARE COSE")

                client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

                client.utils.mount_on_ref()
                client.utils.set_log_to_terminal(True)
                counter = 0
                once = 1
                hasDoorFlag = 0 # variable to confirm that all properties are added correctly
                while counter < 2:
                    # ADD ALL OUR AXIOMS

                    for item in range(len(locationList)):
                        if(client.manipulation.add_ind_to_class(locationList[item], "LOCATION") and once == 1):
                            print("Added " , locationList[item]," to LOCATION" , end='\r')
                            time.sleep(0.5)
                    for Item in range(len(doorList)):
                        if(client.manipulation.add_ind_to_class(doorList[Item], "DOOR") and once == 1):
                            print("Added " , locationList[Item]," to DOOR" , end='\r')
                            time.sleep(0.5)
                    print("Added DOORS, CORRIDORS AND ROOMS!")

                    # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
                    client.manipulation.disj_inds_of_class("LOCATION")
                    client.manipulation.disj_inds_of_class("DOOR")


                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")):
                        hasDoorFlag += 1
                    if(client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")):
                        hasDoorFlag += 1
                    if(hasDoorFlag == 15 and once == 1):
                        print("Added all hasDoor property!")
                    elif(hasDoorFlag != 15 and once == 1):
                        print("ERROR! Something went wrong in adding hasDoor property!")



                    # INITIALIZE ROBOT POSITION
                    client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")



                    # APPLY CHANGES AND QUERY
                    client.utils.apply_buffered_changes()
                    # client.utils.sync_buffered_reasoner()


                    # SAVE AND EXIT

                    client.utils.save_ref_with_inferences('/root/ros_ws/src/exproblab_assignment_2/assignment2/topological_map/my_new_topological_map.owl')

                    client.call('DISJOINT', 'IND', 'CLASS', individuals_list)

                    if(once == 1):
                        for location in range(len(locationList)):
                            if self.isRoom(locationList[location]):
                                if(client.manipulation.add_dataprop_to_ind("visitedAt", locationList[location], 'Long', str(int(time.time())))):
                                    print("Added timestamp to ", locationList[location] , end='\r')
                        #     #time.sleep(0.5)
                        # if(client.manipulation.add_dataprop_to_ind("visitedAt", "R2", 'Long', str(int(time.time())))):
                        #     print("Added timestamp to R2", end='\r')
                        #     #time.sleep(0.5)
                        # if(client.manipulation.add_dataprop_to_ind("visitedAt", "R3", 'Long', str(int(time.time())))):
                        #     print("Added timestamp to R3", end='\r')
                        #     #time.sleep(0.5)
                        # if(client.manipulation.add_dataprop_to_ind("visitedAt", "R4", 'Long', str(int(time.time())))):
                        #     print("Added timestamp to R4", end='\r')
                        #     #time.sleep(0.5)

                        once = 0


                    client.utils.save_ref_with_inferences('/root/ros_ws/src/exproblab_assignment_2/assignment2/topological_map/my_new_topological_map.owl')

                    counter+=1

                # initTime = str(int(time.time()))
                #
                # # DISJOINT OF THE INDIVIDUALS OF THE CLASSES
                # client.manipulation.disj_inds_of_class("LOCATION")
                # client.manipulation.disj_inds_of_class("DOOR")
                #
                #
                # # for i in range(len(roomL)):
                # #     client.manipulation.add_ind_to_class(roomL[i],"LOCATION")
                # # rospy.loginfo("All rooms added to locations!")
                # #
                # # for j in range(len(doors_no_duplicate)):
                # #     client.manipulation.add_indz_to_class(doors_no_duplicate[j],"DOOR")
                # # rospy.loginfo("All Doors added!")
                #
                #
                #
                #
                # # for room in range(len(roomL)):
                # #     if self.IsRoom(str(room)):
                # #         client.manipulation.add_dataprop_to_ind("visitedAt", room, "Long", initTime)
                #
                # # for room in range(len(roomL)):
                # #     if IsRoom(room):
                # #         client.manipulation.add_dataprop_to_ind("visitedAt", room, "Long", initTime)
                #
                #
                # # INITIALIZE ROBOT POSITION
                # client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")
                #
                # #client.manipulation.add_dataprop_to_ind("now", "Robot1", "Long", initTime)
                #
                # #client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot1", "Long", '7')
                #
                #
                # client.utils.save_ref_with_inferences('/root/ros_ws/src/exproblab_assignment_2/assignment2/topological_map/my_new_topological_map.owl')
                #
                # # APPLY CHANGES AND QUERY
                # client.utils.apply_buffered_changes()
                # #client.utils.sync_buffered_reasoner()
                #
                # client.call('DISJOINT', 'IND', 'CLASS', individuals_list)
                #
                # client.utils.save_ref_with_inferences('/root/ros_ws/src/exproblab_assignment_2/assignment2/topological_map/my_new_topological_map.owl')
                #
                # # APPLY CHANGES AND QUERY
                # client.utils.apply_buffered_changes()
                # client.utils.sync_buffered_reasoner()

                all_markers_detected = False
                break


            #
            # # SAVE AND EXIT
            #
            # client.utils.save_ref_with_inferences('/root/ros_ws/src/exproblab_assignment_2/assignment2/topological_map/my_new_topological_map.owl')
            #
            # client.call('DISJOINT', 'IND', 'CLASS', ["R1","R2","R3","R4", "C1","C2", "D1", "D2", "D3", "D4", "D5", "D5", "D7"])


    def NewLoadOntologyCallback(self, room_msg):

        global roomL, doors, doors_no_duplicate, individuals_list, all_markers_detected, client, locationList, doorList, fullList
        roomL.append(room_msg.room)

        fullList.append(room_msg)

        if len(roomL) == 7:
            print("Room list: ",roomL)


        for connection in room_msg.connections:
            for i in range(len(roomL)):
                if i not in doors:
                    doors.append(connection.through_door)

        print(room_msg.room)

        client.manipulation.add_ind_to_class(room_msg.room, "LOCATION")
        #
        # client.manipulation.add_ind_to_class(str(room_msg.room),"LOCATION")


        if self.IsLocation(room_msg.room):
            locationList.append(room_msg.room)
        else:
            doorList.append(room_msg.room)
            #client.manipulation.add_dataprop_to_ind("visitedAt", room_msg.room, "Long", str(int(time.time())))
            #client.manipulation.add_dataprop_to_ind("visitedAt", str(room_msg.room), "Long", str(int(time.time())))

        # for connection in room_msg.connections:
        #     client.manipulation.add_ind_to_class(connection.through_door,"DOOR")
        #     client.manipulation.add_objectprop_to_ind("hasDoor", room_msg.room, connection.through_door)
        #     client.manipulation.add_objectprop_to_ind("hasDoor", connection.connected_to, connection.through_door)


        #create door list without duplicates
        doors_no_duplicate = [*set(doors)]

        if len(roomL) == 7:
            individuals_list = doors_no_duplicate + roomL
            individuals_list.append("Robot1")
            all_markers_detected = True
            print("Doors list: ", doors_no_duplicate)
            print("location list: ", locationList)



    def Reasoning(self):

        ##
        # \brief This function implements the reasoning
        # \param client that is the ArmorClient
        # \return None
        #
        # This function performs the reasoning on the Ontology by calling the two utils methods of the client
        #
        global client
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    def UpdateNowRobot(self):

        ##
        # \brief This function updates robot's now
        # \param client that is the ArmorClient
        # \return None
        #
        # This function retreives the value of the timestamp dataprop now and substitutes it with the updated one
        # by calling the method on the client
        #
        global client
        nowTimeList = client.query.dataprop_b2_ind("now", "Robot1")
        nowTime = nowTimeList[0]
        start = nowTime.find('"') + 1
        end = nowTime.find('"', start)
        numberNow = nowTime[ start : end ]
        client.manipulation.replace_dataprop_b2_ind("now", "Robot1", 'Long', str(int(time.time())), numberNow)


    def UpdateVisitedRoom(self, room):

        ##
        # \brief This function updates visitedAt
        # \param client that is the ArmorClient
        # \return None
        #
        # This function retreives the value of the timestamp dataprop visitedAt and substitutes it with the updated one
        # by calling the method on the client
        #
        global client
        visitedTimeList = client.query.dataprop_b2_ind("visitedAt", room)
        visitedTime = visitedTimeList[0]
        start = visitedTime.find('"') + 1
        end = visitedTime.find('"', start)
        numberVisited = visitedTime[ start : end ]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", room, 'Long', str(int(time.time())), numberVisited)

    def GetUrgentRooms(self):
        ##
        # \brief This function gets the urgent rooms
        # \param client that is the ArmorClient
        # \return urgentL that is the list of urgent rooms
        #
        # This function takes the urgent room and returns the list of all of them
        #
        global client
        urgent = client.query.ind_b2_class('URGENT')
        urgentL = []
        for z in urgent:
            temp = z
            startS = temp.find('#') + 1
            endS = temp.find('>', startS)
            urgentL.append(temp[startS:endS])
        return(urgentL)


    def GetReachableRooms(self):
        ##
        # \brief This function gets the reachable rooms
        # \param client that is the ArmorClient
        # \return reachL that is the list of reachable rooms
        #
        # This function takes the reachable room and returns the list of all of them
        #
        global client
        reach = client.query.objectprop_b2_ind("canReach", "Robot1")
        reachL = []
        for j in reach:
            dumb = j
            startS = dumb.find('#') + 1
            endS = dumb.find('>', startS)
            reachL.append(dumb[startS:endS])
        return(reachL)



    def DecideUrgentReachable(self, listReach, listUrgent):
        ##
        # \brief This function decides in which urgent move the robot
        # \param client that is the ArmorClient, listReach that is the list of rechable, listUrgent that is the list of urgent
        # \return room that is the urgent room chosen
        #
        # This function takes the reachable and urgent list of room and returns a random one which is both urgent both reachable.
        # If there is no urgent reachable, it returns a corridor
        #
        urgReachable = list(set(listReach).intersection(listUrgent))

        if(len(urgReachable) < 1):
            # I can reach only CORRIDORS
            return random.choice(listReach) # so I return a random corridor
        else:
            print(urgReachable)
            return random.choice(urgReachable)

    def IsRoom(self, location):
        ##
        # \brief This function checks if the location is a room
        # \param location the one to check
        # \return bool that is true if it's a room
        #
        if "R" in location:
            return True
        else:
            return False

    def IsLocation(self, location):
        ##
        # \brief This function checks if the location is a room
        # \param location the one to check
        # \return bool that is true if it's a room
        #
        if "R" or "C" or "E" in location:
            return True
        else:
            return False

    def WhichCorridor(self, location):
        ##
        # \brief This function checks which corridor is reachable
        # \param location the one to check
        # \return string that is the corridor
        #
        if "C1" in location:
            return "C1"
        elif "C2":
            return "C2"


    def MoveRobot(self, newRoom):
        ##
        # \brief This function moves the robot
        # \param client that is the ArmorClient, newRoom which is the room in which robot should be moved
        # \return None
        #
        # This function takes the room in which it is the robot and moves it into the new one
        #
        global client
        posRobotList = client.query.objectprop_b2_ind("isIn", "Robot1")
        posRobot = posRobotList[0]
        startP = posRobot.find('#') + 1
        endP = posRobot.find('>', startP)
        prevRoom = posRobot[ startP : endP ]
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", newRoom , prevRoom)



    def set_pose(self, request):
        ##
        # \brief This function sets the pose
        # \param request  is the current robot pose to be set
        # \return SetPoseResponse
        #
        # The `robot/set_pose` service implementation.
        # The `request` input parameter is the current robot pose to be set,
        # as given by the client. This server returns an empty `response`.
        #
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Log information.
            self._print_info(f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                             f'as ({self._pose.x}, {self._pose.y}).')
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()


    def get_pose(self, request):
        ##
        # \brief This function gets the pose
        # \param request is given by the client as empty
        # \return response
        #
        # The `robot/get_pose` service implementation.
        # The `request` input parameter is given by the client as empty. Thus, it is not used.
        # The `response` returned to the client contains the current robot pose.
        #

        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            self._print_info(log_msg)
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def VisitingRoom(self):
        ##
        # \brief This function simulates the visiting
        # \param client that is the ArmorClient
        # \return bool to see if visiting has been completed
        #
        # This function simulates the robot visiting the room for 5 seconds checking at avery iteration the battery status
        #
        secondsLoop = 0
        while(secondsLoop <= 5):
            try:
                # If the battery is low.
                if self.is_battery_low():
                    print("Battery low arrived ... going to RECHARGING state")
                    return False
                else:
                    secondsLoop += 1
                    print("Room visited at: ", 20*secondsLoop, " %", end='\r')
                    rospy.sleep(1)

            finally:
                if secondsLoop == 5:
                    return True

    def PlanningPath(self):

        ##
        # \brief This function calls the client to send the goal to the planning server
        # \param None
        # \return random_plan if the planning has been done correctly
        #
        environment_size = [10.0, 10.0]
        goal = PlanGoal()
        goal.target = Point(x=random.uniform(0, environment_size[0]),
                            y=random.uniform(0, environment_size[1]))
        # Invoke the planner action server.
        self.planner_client.send_goal(goal)
        rospy.loginfo(anm.tag_log('Planning to go in a new random position...', LOG_TAG))
        # Wait for the action server computation and listen possible incoming stimulus.
        while not rospy.is_shutdown():
                # If the battery is low, then cancel the planning action server and take the `battery_low` transition.
            if self.is_battery_low():  # Higher priority.
                self.planner_client.cancel_goals()
                return [False, None]
                # If the planner finishes its computation, then take the `planned_to_random_pose` transition.
            if self.planner_client.is_done():
                random_plan = self.planner_client.get_results().via_points
                return [True, random_plan]

            rospy.sleep(0.2)


    def MovingAlongPath(self, plan):
            ##
            # \brief This function calls the client to control the path toward the goal on the controlling server
            # \param plan the set of via_poi
            # \return bool if true goal reached
            #

            # Start the action server for moving the robot through the planned via-points.
            goal = ControlGoal(via_points=plan)
            self.controller_client.send_goal(goal)
            rospy.loginfo(anm.tag_log('Following the plan to reach a random position...', LOG_TAG))
            # Wait for the action server computation and listen possible incoming stimulus.
            while not rospy.is_shutdown():

                    # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self.is_battery_low():  # Higher priority
                    self.controller_client.cancel_goals()
                    return False
                    # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self.controller_client.is_done():
                    return True



    def reset_battery(self):
        ##
        # \brief This function reset the battery value
        # \param None
        # \return None
        #
        # When called this function sets the value of the battery low variable to false
        #
        self._battery_low = False


    def _battery_callback(self, msg):
        ##
        # \brief This is the callback of the battery
        # \param msg that is the value of the battery
        # \return None
        #
        # The subscriber to get messages published from the `my_helper` node into the `/state/battery_low/` topic.
        #

        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
        # Uncomment below to log data.
            if self._battery_low:
                log_msg = 'Robot with low battery.'
            else:
                log_msg = 'Robot battery fully charged.'
            rospy.loginfo(log_msg)
        finally:
                # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    def is_battery_low(self):
        ##
        # \brief This function get the state of _battery_low variable
        # \param None
        # \return _battery_low
        #
        # Get the state variable encoded in this class that concerns the battery level.
        # The returning value will be `True` if the battery is low, `False` otherwise.
        # Note that the node using this class might exploit the `reset_battery` function to improve robustness.
        # Also note that this function should be used when the `mutex` has been acquired. This assures the
        # synchronization  with the threads involving the subscribers and action clients.
        #
        return self._battery_low


    def _print_info(self, msg):
        ##
        # \brief This function prints informations
        # \param msg
        # \return None
        #
        # Print logging only when random testing is active.
        # This is done to allow an intuitive usage of the keyboard-based interface.
        #
        rospy.loginfo(anm.tag_log(msg, LOG_TAG))


    @staticmethod
    def init_robot_pose(point):
        ##
        # \brief This function updates the robot position
        # \param point
        # \return None
        #
        # This function updates the current robot pose stored in the `my_helper` node.
        #

        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)  # None that the service `response` is not used.
            log_msg = f'Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        except rospy.ServiceException as e:
            err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
            rospy.logerr(anm.tag_log(err_msg, LOG_TAG))




class ActionClientHelper:
    ##
    # \class ActionClientHelper
    # \brief This class defines the ActionClientHelper
    #
    # A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
    #

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):

        ##
        # \brief Class constructor, i.e., class initializer.
        # \param `service_name`: it is the name of the server that will be invoked by this client
        # \return None
        #

        # Input parameters are:
        #  - `service_name`: it is the name of the server that will be invoked by this client.
        #  - `action_type`: it is the message type that the server will exchange.
        #  - `done_callback`: it is the name of the function called when the action server completed its computation. If
        #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
        #     called when the server completes its computation.
        #  - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
        #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
        #    called when the server sends a feedback message.
        #  - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
        #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
        #    synchronization with other classes.
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()


    def send_goal(self, goal):
        ##
        # \brief This function starts the action server with a new `goal`.
        # \param goal
        # \return None
        #
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        #

        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            warn_msg = 'Warning send a new goal, cancel the current request first!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))


    def cancel_goals(self):
        ##
        # \brief Stop the computation of the action server.
        # \param None
        # \return None
        #

        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(anm.tag_log(warn_msg, LOG_TAG))


    def reset_client_states(self):
        ##
        # \brief Reset the client state variables stored in this class.
        # \param None
        # \return None
        #

        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server send some `feedback` back to the client.
    def _feedback_callback(self, feedback):
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()


    def _done_callback(self, status, results):
        ##
        # \brief This function is called when the action server finish its computation, i.e., it provides a `done` message.
        # \param status, result
        # \return None
        #

        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
            # Uncomment below to log information.
            # log_msg = f'`{self._service_name}` done with state `{self._client.get_state_txt()}` and result: {results}.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            self._mutex.release()


    def is_done(self):
        ##
        # \brief This function checks if computation is done.
        # \param None
        # \return _is_done
        #
        # Get `True` if the action server finished is computation, or `False` otherwise.
        # Note that use this method should do it in a `self._mutex` safe manner.
        #

        return self._is_done


    def is_running(self):
        ##
        # \brief This function checks if computation is is_running.
        # \param None
        # \return _is_done
        #
        # Get `True` if the action server is running, or `False` otherwise.
        # A note that use this method should do it in a `self._mutex` safe manner.
        #

        return self._is_running


    def get_results(self):
        ##
        # \brief This gets the result.
        # \param None
        # \return None
        #
        # Get the results of the action server, if any, or `None`.
        #

        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(anm.tag_log(log_err, LOG_TAG))
            return None
