#!/usr/bin/env python

import roslib
import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import smach
import smach_ros
from enum import Enum
import time
from std_msgs.msg import Empty

def goal_action(x, y, heading = None, canceltime=None):
    # Creates the SimpleActionClient
    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print 'Waiting for server...'
    move_base_client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    if (heading != None) :
      q = tf.transformations.quaternion_from_euler(0, 0, heading)
      pose.orientation = geometry_msgs.msg.Quaternion(*q)
    goal = MoveBaseGoal()
    goal.target_pose.pose = pose
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()


    # Sends the goal to the action server.
    print 'Sending goal to action server: %s' % goal
    move_base_client.send_goal(goal)

    if canceltime != None:
      print 'Letting action server work for 3 seconds but then cancelling...'
      time.sleep(canceltime)
      print 'Cancelling current goal...'
      move_base_client.cancel_goal()
    else:
      # Waits for the server to finish performing the action.
      print 'Waiting for result...'
      move_base_client.wait_for_result()

    print 'Result received. Action state is %s' % move_base_client.get_state()
    print 'Goal status message is %s' % move_base_client.get_goal_status_text()

    return move_base_client.get_result()  



class Waypoint:
    def __init__(self, id, type, x, y):
        self.id = id
        self.type = type
        self.x = x
        self.y = y
    class Type(Enum):
        GOAL = 0
        SCIENCE = 1
        EXTRA = 2

# TODO: consider using smach userdata
# gloabal variables used instead of userdata
# because couldn't figure it out how to pass userdata between hierarchical state machines
next_waypoint_id = 0
current_waypoint_id = -1

# dummy waypoint data
waypoints = [Waypoint(0, Waypoint.Type.EXTRA, 4.02, 2.00),
             Waypoint(1, Waypoint.Type.EXTRA, 9.12, 3.54),
             Waypoint(2, Waypoint.Type.EXTRA, 14.25, 1.29),
             Waypoint(3, Waypoint.Type.EXTRA, 15.59, -0.01),
             Waypoint(4, Waypoint.Type.EXTRA, 18.81, 5.52),
             Waypoint(5, Waypoint.Type.EXTRA, 14.87, 9.20),
             Waypoint(6, Waypoint.Type.GOAL, 12.19, 8.73),
             Waypoint(7, Waypoint.Type.EXTRA, 18.00, 8.00),
             Waypoint(8, Waypoint.Type.GOAL, 25.00, 6.00),
             Waypoint(9, Waypoint.Type.EXTRA, 27.30, 7.75),
             Waypoint(10, Waypoint.Type.EXTRA, 28.72, 0.13),
             Waypoint(11, Waypoint.Type.GOAL, 28.62, -6.17),
             Waypoint(12, Waypoint.Type.EXTRA, 19.70, -6.34),
             Waypoint(13, Waypoint.Type.GOAL, 11.63, -16.85),
             Waypoint(14, Waypoint.Type.EXTRA, 9.30, -15.58),
             Waypoint(15, Waypoint.Type.EXTRA, 8.37, -11.32),
             Waypoint(16, Waypoint.Type.GOAL, 7.64, -5.55)]

# define state NAVIGATE WAYPOINT
class NavigateWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome_navigate_waypoint'])

    def execute(self, userdata):
        global current_waypoint_id
        global next_waypoint_id
        rospy.loginfo('Executing state NAVIGATE WAYPOINT')
        self.nav = waypoints[next_waypoint_id]
        rospy.loginfo('Navigation to waypoint with id: {id} {x} {y}'.format(id=self.nav.id, x=self.nav.x, y=self.nav.y))
        result = goal_action(self.nav.x, self.nav.y, 1)
#        rospy.loginfo('Navigation to waypoint with id: {id}'.format(id=next_waypoint_id))
        current_waypoint_id = next_waypoint_id
        rospy.loginfo('Reached to waypoint with id: {id}'.format(id=current_waypoint_id))
        return 'outcome_navigate_waypoint'


# define state ON WAYPOINT
class OnWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome_on_extra_waypoint', 'outcome_on_waypoint', 'outcome_on_science_waypoint'])

    def execute(self, userdata):
        global current_waypoint_id
        global next_waypoint_id
        rospy.loginfo('Executing state ON WAYPOINT')
        self.current_waypoint = waypoints[current_waypoint_id]
        rospy.loginfo('waypoint:( id: {id}, type: {type} )'.format(id=self.current_waypoint.id,
                                                                   type=self.current_waypoint.type))

        time.sleep(3)
        if self.current_waypoint.type is Waypoint.Type.GOAL:
            pub = rospy.Publisher('/probe_deployment_unit/drop', Empty, queue_size=1)

            # necessary for calling only once
            while pub.get_num_connections() < 1:
                pass

            # deploy probe
            rospy.loginfo('DEPLOY')
            pub.publish(Empty())
            return 'outcome_on_waypoint'

        if self.current_waypoint.type is Waypoint.Type.SCIENCE:
            return 'outcome_on_science_waypoint'

        if self.current_waypoint.type is Waypoint.Type.EXTRA:
            return 'outcome_on_extra_waypoint'   

# define state SCIENCE
class Science(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_scince'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SCIENCE')
        return 'outcome_scince'

# define state WAITING
class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_waiting', 'finished'])

    def execute(self, userdata):
        global current_waypoint_id
        global next_waypoint_id
        rospy.loginfo('Executing state WAITING')
        if (current_waypoint_id == waypoints[-1].id):
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
                                                'outcome_on_waypoint':'outcome_working_waypoint',
                                                'outcome_on_science_waypoint': 'SCIENCE'})

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
