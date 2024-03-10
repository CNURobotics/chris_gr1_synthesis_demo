#! /usr/bin/env python

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from synthesis_msgs.action import BehaviorSynthesis
from synthesis_msgs.msg import BehaviorSynthesisRequest, SynthesisOptions


class BehaviorSynthesisClient(Node):
    def __init__(self):
        super().__init__('behavior_synthesis')
        # Create the SimpleActionClient, passing the type of the action to the constructor.
        self._action_client = ActionClient(self, BehaviorSynthesis, 'behavior_synthesis')

    def send_behavior_synthesis_goal(self, system, goals, initial_conditions):
        '''...'''
        # Create a goal to send to the action server.
        goal_msg = BehaviorSynthesis.Goal()

        # Fill out the request part of the message.
        goal_msg.request.system = system
        goal_msg.request.goal = ','.join(goals)
        goal_msg.request.initial_condition = ','.join(initial_conditions)
        goal_msg.request.sm_outcomes = ['finished', 'failed']
        goal_msg.request.name = 'client_request'

        # Fill out any options (all False by default).
        goal_msg.synthesis_options = SynthesisOptions()

        # Wait until the action server has started up.
        self._action_client.wait_for_server()
        print(goal_msg)
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = BehaviorSynthesisClient()
    system = BehaviorSynthesisRequest.SIMPLE
    goals = ['operator']
    initial_conditions = ['log']
    future = action_client.send_behavior_synthesis_goal(system, goals, initial_conditions)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
