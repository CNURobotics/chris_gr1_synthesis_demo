#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from synthesis_msgs.srv import GenerateFlexBESM


class SMGenerateClient():
    def __init__(self):
        # super().__init__('sm_generate')
        self.node = Node('sm_generate')
        self.sm_generate_client = self.node.create_client(GenerateFlexBESM, 'sm_generate')
        self.sm_generate_client.wait_for_service()
        self.req = GenerateFlexBESM.Request()

    def sm_request(self, synthesized_automata, system):
        '''A wrapper for a call to the GenerateFlexBESM service.'''
        print('beginning request', flush=True)
        self.req.automaton = synthesized_automata
        self.req.system = system
        try:
            future = self.sm_generate_client.call_async(self.req)
            rclpy.spin_until_future_complete(self.node, future)

            # DEBUG
            for si in future.result().state_definition:
                self.node.get_logger().debug(si)

            return future.result()

        except Exception as e:
            self.node.get_logger().error('Service call failed: {!r}'.format(e))


def main(synthesized_automata=None, system=None):
    if synthesized_automata is not None and system is not None:
        sm_generate_client = SMGenerateClient()
        response = sm_generate_client.sm_request(synthesized_automata, system)
        sm_generate_client.node.destroy_node()
        return response


if __name__ == '__main__':
    main()
