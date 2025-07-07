#!/usr/bin/env python

import rospy
import smach
import smach_ros

# Dummy functions for illustration
def move_to(location):
    rospy.loginfo(f"Moving to {location}...")
    rospy.sleep(1)
    return True

def pick():
    rospy.loginfo("Picking object...")
    rospy.sleep(1)
    return True

def place():
    rospy.loginfo("Placing object...")
    rospy.sleep(1)
    return True

# Define state classes
class MoveToPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['object_id'])

    def execute(self, userdata):
        rospy.loginfo(f"Moving to pick location for object {userdata.object_id}")
        return 'succeeded' if move_to("pick location") else 'failed'

class PickObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded' if pick() else 'failed'

class MoveToPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['object_id'])

    def execute(self, userdata):
        rospy.loginfo(f"Moving to place location for object {userdata.object_id}")
        return 'succeeded' if move_to("place location") else 'failed'

class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        return 'succeeded' if place() else 'failed'

# Main function
def main():
    rospy.init_node('pick_and_place_iterator')

    # Create the top-level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])

    # List of object IDs to process
    object_ids = ['object_1', 'object_2', 'object_3']

    with sm_top:
        # Define the iterator container
        iterator = smach.Iterator(
            outcomes=['succeeded', 'aborted'],
            input_keys=[],
            output_keys=[],
            it=lambda: iter(object_ids),
            it_label='object_id',
            exhausted_outcome='succeeded'
        )

        # Create the sub-state machine for pick-and-place
        with iterator:
            pick_place_sm = smach.StateMachine(
                outcomes=['succeeded', 'failed'],
                input_keys=['object_id']
            )

            with pick_place_sm:
                smach.StateMachine.add('MOVE_TO_PICK', MoveToPick(),
                    transitions={'succeeded': 'PICK_OBJECT', 'failed': 'failed'},
                    remapping={'object_id': 'object_id'})

                smach.StateMachine.add('PICK_OBJECT', PickObject(),
                    transitions={'succeeded': 'MOVE_TO_PLACE', 'failed': 'failed'})

                smach.StateMachine.add('MOVE_TO_PLACE', MoveToPlace(),
                    transitions={'succeeded': 'PLACE_OBJECT', 'failed': 'failed'},
                    remapping={'object_id': 'object_id'})

                smach.StateMachine.add('PLACE_OBJECT', PlaceObject(),
                    transitions={'succeeded': 'succeeded', 'failed': 'failed'})

            smach.Iterator.set_contained_state('PICK_PLACE', pick_place_sm,
                                               loop_outcomes=['succeeded'])

        # Add the iterator to the top state machine
        smach.StateMachine.add('ITERATE_OBJECTS', iterator,
                               transitions={'succeeded': 'TASK_COMPLETED',
                                            'aborted': 'TASK_FAILED'})

    # Run the state machine
    outcome = sm_top.execute()
    rospy.loginfo(f"Final outcome: {outcome}")

if __name__ == '__main__':
    main()
