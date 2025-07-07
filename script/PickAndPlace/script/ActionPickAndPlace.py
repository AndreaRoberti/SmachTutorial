#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
import PickAndPlace.msg 

# Dummy pick/place functions
def pick():
    rospy.loginfo("Picking object...")
    rospy.sleep(1)
    return True

def place():
    rospy.loginfo("Placing object...")
    rospy.sleep(1)
    return True

# Dummy pose generator
def generate_pick_pose(id_object):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0.5 + 0.1 * id_object
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

def generate_place_pose(id_object):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = 0.2 * id_object
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

# Pick state
class PickObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded' if pick() else 'failed'

# Place state
class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded' if place() else 'failed'

def main():
    rospy.init_node('smach_pick_place_action')

    object_ids = [1, 2, 3]

    # Top-level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])

    with sm_top:
        # Iterator setup
        iterator = smach.Iterator(
            outcomes=['succeeded', 'aborted'],
            input_keys=[],
            output_keys=[],
            it=lambda: iter(object_ids),
            it_label='id_object',
            exhausted_outcome='succeeded'
        )

        with iterator:
            # Sub-state machine for each object
            sm_obj = smach.StateMachine(outcomes=['succeeded', 'failed'],
                                        input_keys=['id_object'])

            with sm_obj:
                # Move to pick
                def pick_goal_cb(userdata, goal):
                    goal.goal_pose = generate_pick_pose(userdata.id_object)
                    goal.id_object = userdata.id_object
                    return goal

                smach.StateMachine.add('MOVE_TO_PICK',
                    smach_ros.SimpleActionState('/go_to_pose',
                                                GoToPoseAction,
                                                goal_cb=pick_goal_cb,
                                                input_keys=['id_object']),
                    transitions={'succeeded': 'PICK_OBJECT',
                                 'aborted': 'failed'},
                    remapping={'id_object': 'id_object'})

                # Pick action
                smach.StateMachine.add('PICK_OBJECT', PickObject(),
                    transitions={'succeeded': 'MOVE_TO_PLACE',
                                 'failed': 'failed'})

                # Move to place
                def place_goal_cb(userdata, goal):
                    goal.goal_pose = generate_place_pose(userdata.id_object)
                    goal.id_object = userdata.id_object
                    return goal

                smach.StateMachine.add('MOVE_TO_PLACE',
                    smach_ros.SimpleActionState('/go_to_pose',
                                                GoToPoseAction,
                                                goal_cb=place_goal_cb,
                                                input_keys=['id_object']),
                    transitions={'succeeded': 'PLACE_OBJECT',
                                 'aborted': 'failed'},
                    remapping={'id_object': 'id_object'})

                # Place action
                smach.StateMachine.add('PLACE_OBJECT', PlaceObject(),
                    transitions={'succeeded': 'succeeded',
                                 'failed': 'failed'})

            # Add sub-SM to iterator
            smach.Iterator.set_contained_state('DO_PICK_PLACE', sm_obj,
                                               loop_outcomes=['succeeded'])

        # Add iterator to top state machine
        smach.StateMachine.add('ITERATE_OBJECTS', iterator,
                               transitions={'succeeded': 'TASK_COMPLETED',
                                            'aborted': 'TASK_FAILED'})

    # Execute the state machine
    outcome = sm_top.execute()
    rospy.loginfo(f"State machine finished with: {outcome}")

if __name__ == '__main__':
    main()
