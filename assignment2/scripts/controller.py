#! /usr/bin/env python

## @package exp1_assignment
# \file controller.py
# \brief This file simulates the behaviour of the robot while moving to a location
# \author Luca Buoncompagni, Davide Bruzzo
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
#   state/set_pose
#
# An action server to simulate motion controlling.
# Given a plan as a set of via points, it simulate the movements
# to reach each point with a random delay. This server updates
# the current robot position stored in the `my_helper` node.
#

import random
import rospy
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from arch_skeleton.msg import ControlFeedback, ControlResult
from arch_skeleton.srv import SetPose
import arch_skeleton  # This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.

# A tag for identifying logs producer.
LOG_TAG = 'CONTROLLER_NODE'

class ControllingAction(object):

    ##
    # \class ControllingAction
    # \brief This class defines the controller
    #
    # This class implements all the methods related to the controller action service, checking the via points provided by the planner
    #

    def __init__(self):
        ##
        # \brief This function is the constructor of the robot's controller class
        # \param None
        # \return None
        #
        # This function implements the constructor of the controller and also instantiates the ActionServer of it
        #

        # Get random-based parameters used by this server
        self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      arch_skeleton.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'CONTROLLER_NODE Action Server initialised. It will navigate trough the plan with a delay '
                   f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}].')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def execute_callback(self, goal):
        ##
        # \brief This function is the callback of the robot's controller class
        # \param goal is the point to reach provided by the planner through the via_points
        # \return None
        #
        # The callback invoked when a client set a goal to the `controller` server.
        # This function requires a list of via points (i.e., the plan), and it simulate
        # a movement through each point with a delay spanning in
        # ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
        # As soon as each via point is reached, the related robot position is updated
        # in the `robot-state` node.
        #

        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                # Wait before to reach the following via point. This is just for testing purposes.
            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)
            # Publish a feedback to the client to simulate that a via point has been reached.
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            _set_pose_client(point)
            # Log current robot position.
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.



def _set_pose_client(pose):
    ##
    # \brief This function sets the pose of the robot
    # \param pose is the position to set
    # \return None
    #
    # Update the current robot `pose` stored in the `my_helper` node.
    # This method is performed for each point provided in the action's server feedback.
    #

    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Log service call.
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.
    rospy.init_node( 'CONTROLLER_NODE', log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
