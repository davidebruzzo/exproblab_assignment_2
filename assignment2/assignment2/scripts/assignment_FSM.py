#!/usr/bin/env python

## @package exp1_assignment
# \file assignment_FSM.py
# \brief This file represent the FSM implementation
# \author Davide Bruzzo
# \version 1.0
# \date 18/11/2022
#
# \details
#
#  Subscribes to: <BR>
#   [None]
#
#  Publishes to: <BR>
#   [None]
#
#  Service: <BR>
#   [None]
#
# This file implements all the nodes of the final state machine that I thought
#
import roslib
import rospy
import smach
import smach_ros
import time
import random

# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import the armor client class
from armor_client import ArmorClient

from my_helper import MyHelper

# Import used messages defined within the ROS architecture.
from arch_skeleton.msg import Gesture, Point, Speech, PlanAction, PlanGoal, ControlAction, ControlGoal





path = '/root/ros_ws/src/topological_map/'


# define state Wait Ontology
class WaitForOntology(smach.State):
    ##
    # \class WaitForOntology
    # \brief This class defines the WaitForOntology state
    #
    # This class implements the WaitForOntology state of the final state machine, it is done once to create the ontology as the requirements of the assignments asked.
    # It calls the LoadOntology function to do that. The LoadOntology has a particular structure with loops that do the same instruction two or three times.
    # This is not an error but vouluntary done due to some bugs of the armor server. This is a solution I found to make things working and overcoming the bugs.
    #

    def __init__(self, my_helper):
        ##
        # \brief This function is the constructor of the WaitForOntology class
        # \param my_helper
        # \return None
        #
        self._helper = my_helper
        smach.State.__init__(self,
                             outcomes=['received','battery_low', 'charged', 'planned_to_urgent', 'visited'])


    def execute(self, userdata):
        ##
        # \brief This function is the excute function of the WaitForOntology state
        # \param userdata
        # \return None
        #
        rospy.loginfo('WAIT FOR INITIALIZATION')
        time.sleep(5)

        rospy.loginfo('Executing state WaitOntology')
        self._helper.NewLoadOntology(path)


        time.sleep(0.5)
        return 'received' #we need to enter in recharging state to see if the robot has  100% battery

class Recharging(smach.State):
    ##
    # \class Recharging
    # \brief This class defines the Recharging state
    #
    # This class implements the Recharging state of the final state machine, it moves the
    # robot to the E location which is the one for recharging.
    # Then checks in every iteration the state of the battery to see if robot has completed recharging.
    #
    def __init__(self, my_helper):
        ##
        # \brief This function is the constructor of the Recharging class
        # \param my_helper
        # \return None
        #
        self._helper = my_helper

        smach.State.__init__(self,
                             outcomes=['received','battery_low', 'charged', 'planned_to_urgent', 'visited'],
                             input_keys=['recharging_counter_in'])


    def execute(self, userdata):
        ##
        # \brief This function is the excute function of the Recharging state
        # \param userdata
        # \return None
        #

        self._helper.MoveRobot("E")
        print("Robot moved to E ")
        rospy.sleep(0.2)
        print("ROBOT IS IN RECHARGING")
        self._helper.mutex.acquire()
        try:
            # If the battery is no low anymore take the `charged` transition.
            if not self._helper.is_battery_low():
                self._helper.reset_battery()  # Reset the state variable related to the stimulus.
                print("BATTERY CHARGED")
                return 'charged'
        finally:
            # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
            self._helper.mutex.release()
        time.sleep(0.5)
        return random.choice(['received', 'battery_low', 'visited', 'planned_to_urgent'])




class Plan_To_Urgent(smach.State):
    ##
    # \class Plan_To_Urgent
    # \brief This class defines the Plan_To_Urgent state
    #
    # This class implements the Plan_To_Urgent state of the final state machine, it gets the
    # room reachable and the urgent ones. Then decides which room robot should reach. It calls the method for creting the path through 'via_points',
    # simulating in this way the behaviour of the real robot, then calls the function to move the robot in the room for the ontology.
    # This is made always checking in every loop the state of the battery to see if robot has to go recharging.
    #
    def __init__(self, my_helper):
        ##
        # \brief This function is the constructor of the Plan_To_Urgent class
        # \param my_helper
        # \return None
        #
        self._helper = my_helper
        smach.State.__init__(self,
                             outcomes=['received','battery_low', 'charged', 'planned_to_urgent', 'visited'],
                             input_keys=['Plan_To_Urgent_counter_in'],
                             output_keys=['Plan_To_Urgent_var', 'resultRoom'])


    def execute(self, userdata):
        ##
        # \brief This function is the excute function of the Plan_To_Urgent state
        # \param userdata
        # \return None
        #

        self._helper.UpdateNowRobot()
        self._helper.Reasoning()
        uRooms = self._helper.GetUrgentRooms()
        rRooms = self._helper.GetReachableRooms()

        resultR = self._helper.DecideUrgentReachable(rRooms, uRooms)
        userdata.resultRoom = resultR
        try:
    # If the battery is low.
            if self._helper.is_battery_low():
                print("Battery low arrived ... going to RECHARGING state")
                if(self._helper.IsRoom(resultR)):
                    print("Robot found in a room, planning for coming back to E location...")
                    rospy.sleep(0.5)
                    checkR = self._helper.GetReachableRooms()
                    corridor = self._helper.WhichCorridor(checkR)
                    self._helper.MoveRobot(corridor)
                    print("Robot moved to CORRIDOR: ",  corridor)
                return 'battery_low'
            else:
                print("URGENT ROOMS: ", uRooms)
                print("REACHABLE ROOMS: " ,rRooms)
                print("RANDOMLY CHOSEN ONE: ", resultR)
                time.sleep(1)
                if(self._helper.IsRoom(resultR)):
                    listPath = self._helper.PlanningPath()
                    if(listPath[0]):
                        print("\nPath planned!\n")
                        userdata.Plan_To_Urgent_var = listPath[1]
                        self._helper.MoveRobot(resultR)
                        print("I'm moving to the urgent room: ", resultR)
                        rospy.sleep(1)
                        self._helper.Reasoning()
                        return 'planned_to_urgent'
                    else:
                        return 'battery_low'
                else:
                    #waiting for planning to in corridor
                    listPathC = self._helper.PlanningPath()
                    if(listPathC[0]):
                        self._helper.MoveRobot(resultR)
                        # I decided to not call server for going to corridor cause i imagined passing from one corridor to another immediately
                        print("Robot is moving to corridor: ", resultR)
                        rospy.sleep(0.5)
                        self._helper.Reasoning()
                        return random.choice(['received', 'charged', 'visited'])

                    else:
                        return 'battery_low'
        finally:
            print()


class Visit_Room(smach.State):
    ##
    # \class Visit_Room
    # \brief This class defines the Visit_Room state
    #
    # This class implements the Visit_Room state of the final state machine, it receives the
    # room chosen and the set of 'via_points' by 'Plan_To_Urgent' state. It calls the method for moving along the path created,
    # simulating in this way the behaviour of the real robot, then calls the function to simulate the robot visiting the room.
    # This is made always checking in every loop the state of the battery to see if robot has to go recharging.
    #

    def __init__(self, my_helper):
        ##
        # \brief This function is the constructor of the Visit_Room class
        # \param my_helper
        # \return None
        #
        self._helper = my_helper
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['received','battery_low', 'charged', 'planned_to_urgent', 'visited'],
                             input_keys=['Plan_To_Urgent_var', 'resultRoom'],
                             output_keys=['Visit_Room_counter_out'])

    def execute(self, userdata):
        ##
        # \brief This function is the excute function of the Visit_Room state
        # \param userdata
        # \return None
        #
        roomChosen = userdata.resultRoom
        plan = userdata.Plan_To_Urgent_var

        if(self._helper.MovingAlongPath(plan)):
            print("Reached room: ", roomChosen)
        else:
            print("Robot found with battery low during its path toward the room \nGoing to recharching state...")
            return 'battery_low'


        if(self._helper.VisitingRoom()):
            print("\nFULLY VISITED ROOM: ", roomChosen, "\n")
            self._helper.UpdateNowRobot()
            self._helper.UpdateVisitedRoom(roomChosen)
            self._helper.Reasoning()
            rospy.sleep(0.5)
            return 'visited'
        else:
            #battery_low arrived, moving robot to E location
            print("BATTERY LOW")
            if(self._helper.IsRoom(roomChosen)):
                self._helper.UpdateNowRobot()
                self._helper.Reasoning()
                print("Robot found in a room, planning for coming back to E location...")
                rospy.sleep(0.5)
                checkR = self._helper.GetReachableRooms()
                corridor = self._helper.WhichCorridor(checkR)
                print("CORRIDOR CHOSEN: ", corridor)
                self._helper.MoveRobot(corridor)
                self._helper.UpdateNowRobot()
                self._helper.Reasoning()
            return 'battery_low'

def main():
    ##
    # \brief This function is the main of assignment_FSM
    # \param None
    # \return None
    #
    rospy.init_node('exercise_robot_state_machine')

    my_helper = MyHelper()

    # Get the initial robot pose from ROS parameters.
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    # Initialise robot position in the `robot_state`, as required by the plan anc control action servers.
    my_helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT ONTOLOGY', WaitForOntology(my_helper),
                               transitions={'received':'RECHARGING',
                                            'battery_low':'WAIT ONTOLOGY',
                                            'charged':'WAIT ONTOLOGY',
                                            'planned_to_urgent':'WAIT ONTOLOGY',
                                            'visited':'WAIT ONTOLOGY'},
                               remapping={'WaitForOntology_counter_in':'sm_counter',
                                          'WaitForOntology_counter_out':'sm_counter'})
        smach.StateMachine.add('RECHARGING', Recharging(my_helper),
                               transitions={'received':'RECHARGING',
                                            'battery_low':'RECHARGING',
                                            'charged':'PLAN TO URGENT',
                                            'planned_to_urgent':'RECHARGING',
                                            'visited':'RECHARGING'},
                               remapping={'recharging_counter_in':'sm_counter',
                                          'recharging_counter_out':'sm_counter'})
        smach.StateMachine.add('PLAN TO URGENT', Plan_To_Urgent(my_helper),
                               transitions={'received':'PLAN TO URGENT',
                                            'battery_low':'RECHARGING',
                                            'charged':'PLAN TO URGENT',
                                            'planned_to_urgent':'VISIT ROOM',
                                            'visited':'PLAN TO URGENT'},
                               remapping={'Plan_To_Urgent_counter_in':'sm_counter',
                                          'Plan_To_Urgent_counter_out':'sm_counter'})

        smach.StateMachine.add('VISIT ROOM', Visit_Room(my_helper),
                               transitions={'received':'VISIT ROOM',
                                            'battery_low':'RECHARGING',
                                            'charged':'VISIT ROOM',
                                            'planned_to_urgent':'VISIT ROOM',
                                            'visited':'PLAN TO URGENT'},
                               remapping={'Visit_Room_counter_in':'sm_counter',
                                          'Visit_Room_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
