#! /usr/bin/env python3

## @package exp1_assignment
# \file battery.py
# \brief This file simulates the behaviour of the battery
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
#   state/battery_low
#
#

import threading
from random import randrange
import rospy
# Import the messages used by services and publishers.
from std_msgs.msg import Bool


class Battery:

    ##
    # \class Battery
    # \brief This class defines all the methods related to the battery
    #
    # This class implements the behaviour of the battery, like discharging and recharging but also the publisher of it to warn other nodes on the battery state
    #

    def __init__(self):

        ##
        # \brief This function is the constructor of the robot's battery class
        # \param None
        # \return None
        #
        # This function implements the constructor of the battery and also starts the thread of it
        #

        rospy.init_node("battery_node", log_level=rospy.INFO)

        self._battery_low = False

        self._random_battery_time = randrange(15, 40)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()
        # Log information.
        log_msg = ('Initialize node battery.')
        rospy.loginfo(log_msg)

    def _is_battery_low(self):

        ##
        # \brief Publish changes of battery levels
        # \param None
        # \return None
        #
        # This function publishes the battery state changes. This method runs on a separate thread.
        #

        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher('state/battery_low', Bool, queue_size=1, latch=True)
        # Publish battery level changes randomly.
        self._random_battery_notifier(publisher)


    def _random_battery_notifier(self, publisher):
        
        ##
        # \brief Publish when the battery change state.
        # \param publisher is the name of the publisher that publish on the topic 'state/battery_low
        # \return None
        #
        # This function publishes when the battery change state (i.e., high/low) based on a random delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        # The message is published through the `publisher` input parameter and is a
        # boolean value, i.e., `True`: battery low, `False`: battery high.
        #

        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                log_msg = f'Robot got low battery after {delay} seconds.'
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
            rospy.loginfo(log_msg)
            # Wait for simulate battery usage.
            delay = randrange(15,40)
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    Battery()
    rospy.spin()
