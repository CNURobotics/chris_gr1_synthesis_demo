#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from synthesis_msgs.srv import SynthesizeAutomaton


class LTLSynthesisClient():
    def __init__(self):
        # super().__init__('ltl_synthesis')
        self.node = Node('ltl_synthesis')
        self.ltl_synthesis_client = self.node.create_client(SynthesizeAutomaton, 'ltl_synthesis')
        self.ltl_synthesis_client.wait_for_service('ltl_synthesis')
        self.req = SynthesizeAutomaton.Request()

    def ltl_synthesis_request(self, ltl_spec, name=''):
        '''Client'''

        try:

            self.req.ltl_specification = ltl_spec
            self.req.spec_name = name
            self.future = self.ltl_synthesis_client.call_async(self.req)
            rclpy.spin_until_future_complete(self.node, self.future)

            # DEBUG
            # self.node.get_logger().debug('LTL Synthesis client reporting:')
            # self.node.get_logger().debug(self.future.result().automaton)
            # self.node.get_logger().debug('LTL Synthesis error code: {}'
            #                              .format(self.future.result().error_code))

            return self.future.result()

        except Exception as e:
            self.node.get_logger().error('Service call failed: {!r}'.format(e))


def main(ltl_spec=None, name=''):
    if ltl_spec is not None:
        ltl_synthesis_client = LTLSynthesisClient()
        response = ltl_synthesis_client.ltl_synthesis_request(ltl_spec, name)
        ltl_synthesis_client.node.destroy_node()
        return response


if __name__ == '__main__':
    main()
