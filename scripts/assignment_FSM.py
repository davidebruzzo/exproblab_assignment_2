#!/usr/bin/env python

## @package exp1_assignment
# \file assignment_FSM.py
# \brief This file represent the FSM implementation
# \author Davide Bruzzo
# \version 2.0
# \date 10/02/2023
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
    # It calls the NewLoadOntology function to do that. The NewLoadOntology has a particular structure with loops that do the same instruction two or three times.
    # This is not an error but vouluntary done due to some bugs of the armor server. This is a solution I found to make things working and overcoming the bugs.
    # The robot stays in this state until all the markers are found and the informations that are in them analyzed.
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
        # Wait 5 seconds to let gazebo open
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

        self._helper.NewMoveRobot("E")
        self._helper.MoveRobot("E")
        rospy.loginfo("Robot moved to E location")

        print('\033[5mRobot is going to recharging station!\033[0m', end='\r')
        while not self._helper.moveBaseClient.is_done():
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_battery()  # Reset the state variable related to the stimulus.
                    self._helper.moveBaseClient.cancel_goals()
                    rospy.loginfo("BATTERY CHARGED")
                    return 'charged'
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()

        # reason
        self._helper.Reasoning()
        return random.choice(['received', 'battery_low', 'visited', 'planned_to_urgent'])




class Plan_To_Urgent(smach.State):
    ##
    # \class Plan_To_Urgent
    # \brief This class defines the Plan_To_Urgent state
    #
    # This class implements the Plan_To_Urgent state of the final state machine, it gets the
    # room reachable and the urgent ones. Then decides which room robot should reach. Provides the chosen location to the next state.
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
                    checkR = self._helper.GetReachableRooms()
                    corridor = self._helper.WhichCorridor(checkR)
                return 'battery_low'
            else:
                print("URGENT ROOMS: ", uRooms)
                print("REACHABLE ROOMS: " ,rRooms)
                print("RANDOMLY CHOSEN ONE: ", resultR)
                time.sleep(1)
                return 'planned_to_urgent'
        finally:
            print()


class Visit_Room(smach.State):
    ##
    # \class Visit_Room
    # \brief This class defines the Visit_Room state
    #
    # This class implements the Visit_Room state of the final state machine, it receives the
    # room chosen by 'Plan_To_Urgent' state. It calls the method for sending the goal position to the move_base client, then calls
    # the the function to simulate the robot visiting the location if it's a room, by doing a rotation of 360 degrees. Then returns to 'Plan_To_Urgent'
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
        #send the goal
        self._helper.NewMoveRobot(roomChosen)

        while not self._helper.moveBaseClient.is_done():

            if self._helper.is_battery_low():
                print("Battery low arrived ... canceling goals")
                self._helper.moveBaseClient.cancel_goals()
                print("Going to RECHARGING state")

                return 'battery_low'

            else:
                print("Reaching: ", roomChosen, end='\r')


        rospy.loginfo("ROOM REACHED!")
        #to clear all goals 
        self._helper.moveBaseClient.cancel_goals()
        self._helper.MoveRobot(roomChosen)
        self._helper.Reasoning()

        if self._helper.IsRoom(roomChosen):
            if self._helper.VisitingRoom():
                self._helper.UpdateNowRobot()
                self._helper.UpdateVisitedRoom(roomChosen)
                self._helper.Reasoning()
                return 'visited'
            else:
                print("Battery low")
                return 'battery_low'
        else:
            self._helper.UpdateNowRobot()
            self._helper.Reasoning()
            return 'visited'


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
