#!/usr/bin/env python
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from synthesis_msgs.msg import SynthesisErrorCodes
from synthesis_msgs.action import BehaviorSynthesis

from .ltl_compilation_client import LTLCompilationClient
# from .ltl_synthesis_client import LTLSynthesisClient
from .ltl_synthesis_client import main as ltl_main
# from .sm_generate_client import SMGenerateClient
from .sm_generate_client import main as sm_main


class BehaviorSynthesisActionServer(Node):

    _feedback = BehaviorSynthesis.Feedback()
    _result = BehaviorSynthesis.Result()

    def __init__(self, name):
        super().__init__('behavior_synthesis_server')
        self._action_name = name
        self._action_server = ActionServer(self, BehaviorSynthesis, self._action_name,
                                           execute_callback=self.execute_callback,
                                           cancel_callback=self.cancel_callback)
        self.error_code_value = None
        self.ltl_spec = None
        self.automaton = None
        self.sm = None
        # self._preempt_requested = False

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # TODO Define the main chunks of the server

    def execute_callback(self, goal):
        '''The code to be executed when a BehaviorSynthesisActionGoal is received.'''
        print("The code to be executed when a BehaviorSynthesisActionGoal is received...")
        # r = self.create_rate(1)   # FIXME: What should this be?
        self.success = True      # start optimistically
        self.error_code_value = None  # Reset
        self.ltl_spec = None  # Reset
        self.automaton = None  # Reset
        self.sm = None  # Reset

        # Acknowledge goal reception
        # self.set_and_publish_feedback("Received behavior synthesis request.")

        # Examine the goal message
        synthesis_goal = goal.request
        # print('BehaviorSynthesisServer line 53 ', type(synthesis_goal))
        # print('BehaviorSynthesisServer line 54 ', synthesis_goal)
        # synthesis_options = goal.synthesis_options

        # # TODO: receive a callback when a preempt request is received
        # if self._preempt_requested:  # TODO change to utilize something in actions api
        #     self.get_logger().info('{}: Preempted'.format(self._action_name))
        #     self._as.set_preempted()  # TODO change to utilize something in actions api
        #     self.success = False

        if self.success:
            # Request LTL Specification Compilation from the corresponding server
            # and also update and publish the appropriate feedback
            print("handle spec request given goal ...")
            self.ltl_spec, self.error_code_value = self.handle_ltl_specification_request(
                                                    synthesis_goal)

        if self.success:
            # Request LTL Synthesis from the corresponding server
            # and also update and publish the appropriate feedback
            print('handle synthesis request given spec ... ')
            self.automaton, self.error_code_value = self.handle_ltl_synthesis_request(
                                                    self.ltl_spec, synthesis_goal.request.name)

        if self.success:
            # Request State Machine Generation from the corresponding server
            # and also update and publish the appropriate feedback
            # TODO: how to get the the yaml_config file?
            print("handle SM generation given synthesized automata ...")
            self.sm, self.error_code_value = self.handle_sm_generation_request(
                                                self.automaton, synthesis_goal.request.system)

        if self.success:
            print("Success!", flush=True)
            self._result.error_code = SynthesisErrorCodes(value=SynthesisErrorCodes.SUCCESS)
            self._result.states = self.sm
            self.get_logger().info('\033[92m{}: Succeeded\033[0m'.format(self._action_name))
            goal.succeed()

        else:
            print("Failed to generate SM given goal!", flush=True)
            self._result.error_code = SynthesisErrorCodes(value=self.error_code_value)
            self._result.states = []
            self.get_logger().error('%s: Failed' % self._action_name)
            goal.abort()

        return self._result

    def handle_ltl_specification_request(self, synthesis_goal):
        '''
        Makes a LTL Specification Compilation request
        to the corresponding service and handles the response.
        synthesis_goal: BehaviorSynthesisRequest    A partial specification.
        '''

        # print("  set up the spec ....", flush=True)
        system = synthesis_goal.request.system
        goals = synthesis_goal.request.goal.replace(' ', '').split(',')
        ics = synthesis_goal.request.initial_condition.replace(' ', '').split(',')
        sm_outcomes = synthesis_goal.request.sm_outcomes
        # custom_ltl = synthesis_goal.ltl_specification  # TODO: Handle this field

        # print("system: ", system)
        # print("goals: ", goals)
        # print("IC: ", ics)
        # print("outcomes: ", sm_outcomes)

        # print("request to compile the spec ...", flush=True)
        response = LTLCompilationClient().ltl_compilation_request(system, goals, ics, sm_outcomes)

        # Update success and publish feedback based on response
        if response is not None:
            if response.error_code.value is SynthesisErrorCodes.SUCCESS:
                self.set_and_publish_feedback('Received LTL specification')
                self.success = True
            else:
                self.set_and_publish_feedback('Did not receive LTL specification')
                self.success = False

            return response.ltl_specification, response.error_code.value
        else:
            print(" ltl_compilation request returned NONE!", flush=True)
            self.set_and_publish_feedback('Did not receive LTL specification')
            self.success = False
            return None, SynthesisErrorCodes.LTL_SPEC_COMPILATION_FAILED

    def handle_ltl_synthesis_request(self, ltl_spec, path_name):
        '''
        Makes a LTL Synthesis request
        to the corresponding service and handles the response.
        ltl_spec:   LTLSpecification    A complete LTL specification.
        '''

        # response is not being properly assigned # TODO find missing response location
        # response = LTLSynthesisClient().ltl_synthesis_request(ltl_spec, path_name)
        response = ltl_main(ltl_spec, path_name)
        # print(response)

        # print(response)
        if not response:
            self.set_and_publish_feedback('The LTL Synthesis service failed!')
            self.success = False
            return None, SynthesisErrorCodes.SYNTHESIS_FAILED

        if response.synthesizable:
            self.set_and_publish_feedback('The LTL Specification is synthesizable')
            self.success = True
        else:
            self.set_and_publish_feedback('The LTL Specification is unsynthesizable')
            self.success = False
        # print('behavior_synthesis_server line 160 ', response, flush=True)
        # print('behavior_synthesis_server line 161 ', self.success, flush=True)
        return response.automaton, response.error_code.value

    def handle_sm_generation_request(self, synthesized_automata, system):
        '''
        Generate State Machine definitions for a given
        robotic system based on a synthesized automaton.
        @param synthesized_automata FSAutomaton    The automaton to instantiate as a SM.
        @param system               string                  System name. e.g. "atlas"
        '''
        print('beginning sm_generation', flush=True)
        # response = SMGenerateClient().sm_request(synthesized_automata, system)
        response = sm_main(synthesized_automata, system)
        # Update success and publish feedback based on response
        if response.error_code.value is SynthesisErrorCodes.SUCCESS:
            self.set_and_publish_feedback('Generated State Machine definitions')
            self.success = True
        else:
            self.set_and_publish_feedback('Unable to generate State Machine.')
            self.success = False

        return response.state_definition, response.error_code.value

    def set_and_publish_feedback(self, status):
        '''Helper method for updating and publishing feedback.'''
        print('trying to publish information', flush=True)
        self._feedback.status = status
        self.get_logger().info(status)  # Display the status message
        # print(" @TODO - we want to call action server to publish on feedback topic?", flush=True)
        print('exiting publish function', flush=True)

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    b_s_action_server = BehaviorSynthesisActionServer('behavior_synthesis')
    executor = SingleThreadedExecutor()  # might be able to do without executor
    rclpy.spin(b_s_action_server, executor=executor)
    b_s_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
