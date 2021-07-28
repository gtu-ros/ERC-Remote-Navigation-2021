#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from enum import Enum

class Waypoint:
    def __init__(self, id, type):
        self.id = id
        self.type = type
    class Type(Enum):
        GOAL = 0
        SCIENCE = 1

# TODO: consider using smach userdata
# gloabal variables used instead of userdata
# because couldn't figure it out how to pass userdata between hierarchical state machines
next_waypoint_id = 0
current_waypoint_id = -1

# dummy waypoint data
waypoints = [Waypoint(0, Waypoint.Type.GOAL),
             Waypoint(1, Waypoint.Type.SCIENCE),
             Waypoint(2, Waypoint.Type.GOAL),]

# define state NAVIGATE WAYPOINT
class NavigateWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome_navigate_waypoint'])

    def execute(self, userdata):
        global current_waypoint_id
        global next_waypoint_id
        rospy.loginfo('Executing state NAVIGATE WAYPOINT')
        rospy.loginfo('Navigation to waypoint with id: {id}'.format(id=next_waypoint_id))
        current_waypoint_id = next_waypoint_id
        rospy.loginfo('Reached to waypoint with id: {id}'.format(id=current_waypoint_id))
        return 'outcome_navigate_waypoint'


# define state ON WAYPOINT
class OnWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome_on_extra_waypoint', 'outcome_on_waypoint'])

    def execute(self, userdata):
        global current_waypoint_id
        global next_waypoint_id
        rospy.loginfo('Executing state ON WAYPOINT')
        self.current_waypoint = waypoints[current_waypoint_id]
        rospy.loginfo('waypoint:( id: {id}, type: {type} )'.format(id=self.current_waypoint.id,
                                                                   type=self.current_waypoint.type))
        if self.current_waypoint.type is Waypoint.Type.GOAL:
            return 'outcome_on_waypoint'
        if self.current_waypoint.type is Waypoint.Type.SCIENCE:
            return 'outcome_on_extra_waypoint'

# define state SCIENCE
class Science(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_scince'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SCIENCE')
        return 'outcome_scince'

# define state SCIENCE
class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_waiting', 'finished'])

    def execute(self, userdata):
        global current_waypoint_id
        global next_waypoint_id
        rospy.loginfo('Executing state WAITING')
        if (current_waypoint_id == len(waypoints) - 1):
            rospy.loginfo('WAITING: finished')
            return 'finished'
        else:
            # next_waypoint_id = current_waypoint_id + 1
            next_waypoint_id = next_waypoint_id + 1
            rospy.loginfo('WAITING: next_waypoint_id = {id}'.format(id=next_waypoint_id))
            return 'outcome_waiting'

# define state IDLE
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_idle'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')
        return 'outcome_idle'


def main():
    rospy.init_node('rover_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['EXIT'])

    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'outcome_idle':'WORKING'})

        # WORKING SMACH state machine
        # either outcome of usual waypoint or science waypoint
        sm_working = smach.StateMachine(outcomes=['outcome_working_waypoint',
                                                  'outcome_working_scince'])

        # Open the container
        with sm_working:

            # Add states to the container
            smach.StateMachine.add('NAVIGATE WAYPOINT', NavigateWaypoint(),
                                   transitions={'outcome_navigate_waypoint':'ON WAYPOINT'})

            smach.StateMachine.add('ON WAYPOINT', OnWaypoint(),
                                   transitions={'outcome_on_extra_waypoint':'SCIENCE',
                                                'outcome_on_waypoint':'outcome_working_waypoint'})

            smach.StateMachine.add('SCIENCE', Science(),
                                   transitions={'outcome_scince':'outcome_working_scince'})

        smach.StateMachine.add('WORKING', sm_working,
                               transitions={'outcome_working_waypoint':'WAITING',
                                            'outcome_working_scince':'WAITING'})

        smach.StateMachine.add('WAITING', Waiting(),
                               transitions={'outcome_waiting':'WORKING',
                                            'finished':'EXIT'})
    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()
