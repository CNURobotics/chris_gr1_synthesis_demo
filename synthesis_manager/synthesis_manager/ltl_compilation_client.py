#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from synthesis_msgs.srv import GenerateLTLSpecification


class LTLCompilationClient():
    def __init__(self):
        # super().__init__('ltl_compilation')
        self.node = Node('ltl_compilation')
        # or should it be self.node = rclpy.create_node('ltl_compilation')
        self.ltl_compilation_client = self.node.create_client(
            GenerateLTLSpecification, 'ltl_compilation')
        self.ltl_compilation_client.wait_for_service()
        self.req = GenerateLTLSpecification.Request()

    def ltl_compilation_request(self, system, goals, initial_conditions, sm_outcomes,
                                custom_ltl=None):
        '''Client'''

        try:
            # print("\n\n LTL compilation request ...", flush=True)
            # print('****', system, '****')
            self.req.system = system
            self.req.goals = goals
            self.req.initial_conditions = initial_conditions
            self.req.sm_outcomes = sm_outcomes

            # print("  making async service call ....")
            self.future = self.ltl_compilation_client.call_async(self.req)
            rclpy.spin_until_future_complete(self.node, self.future)
            # print("  future is complete ....", flush=True)

            # DEBUG
            # print response.ltl_specification
            # print("line 37 in compilation server: ", type(self.req), flush=True)
            # print("  request: ", self.req, flush=True)
            # print("Type of result: ", type(self.future.result()), flush=True)
            # print(" result of future: ", self.future.result(), flush=True)
            self.node.get_logger().info('LTL Compilation error code: {}'
                                        .format(self.future.result().error_code.value))
            # print(" returning result of future!", flush=True)
            return self.future.result()

        except Exception as e:
            self.node.get_logger().error('ltl_compilation_request - service call failed {!r}'
                                         .format(e))


def main(system=None, goals=None, ics=None, sm_outcomes=None, args=None):
    isValid = (system is not None and goals is not None and ics is not None and
               sm_outcomes is not None)
    if isValid:
        ltl_client = LTLCompilationClient()
        response = ltl_client.ltl_compilation_request(('simple', ['log'], ['operator']))
        ltl_client.destroy_node()
        return response


if __name__ == '__main__':
    main()
