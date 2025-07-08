#!/usr/bin/env python

import smach

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in', 'foo_my_name'],
                             output_keys=['foo_counter_out','foo_output_my_name'])

    def execute(self, userdata):
        print('Executing state FOO')
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        else:
            print(userdata.foo_my_name)
            userdata.foo_output_my_name = "FRANCESCO"
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
    sm.userdata.my_name =  "ANDREA"

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter','foo_my_name':'my_name',
                                          'foo_counter_out':'sm_counter','foo_output_my_name':'my_name',})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    outcome = sm.execute()
    print(sm.userdata.my_name)

if __name__ == '__main__':
    main()