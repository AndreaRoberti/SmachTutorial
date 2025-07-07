#!/usr/bin/env python

import smach

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        print('Executing state FOO')
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])
        
    def execute(self, userdata):
        print('Executing state BAR')
        print('Counter = %f'%userdata.bar_counter_in)        
        return 'outcome1'
        




def main():
    print('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()