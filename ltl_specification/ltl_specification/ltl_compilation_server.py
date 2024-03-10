#!/usr/bin/env python

import importlib
import rclpy
from rclpy.node import Node

from synthesis_msgs.srv import GenerateLTLSpecification
from synthesis_msgs.msg import LTLSpecification, SynthesisErrorCodes


class LTLCompilationServer(Node):

    def __init__(self):
        super().__init__('ltl_specifcation')
        self.srv = self.create_service(GenerateLTLSpecification, 'ltl_compilation',
                                       self.handle_ltl_compilation)

    def handle_ltl_compilation(self, request, placeholder=None):
        '''Responsible for putting together a complete LTL specification.'''
        answer = GenerateLTLSpecification.Response()
        if request.system:
            if (request.system == 'simple' or request.system == 'atlas' or
               request.system == 'system_wide' or request.system == 'turtlebot'):
                ltl_specification, error_code = self.gen_ltl_spec_from_request(request)
                answer.ltl_specification = ltl_specification
                answer.error_code.value = error_code

            else:
                ltl_specification = LTLSpecification()
                answer.error_code.value = SynthesisErrorCodes.LTL_SPEC_COMPILATION_FAILED
                self.get_logger().error(
                    'LTL Specification Compilation does not support {}'.format(request.system))

        return answer

    def gen_ltl_spec_from_request(self, request):
        module_to_import = f'ltl_specification.{request.system}_specification'
        self.get_logger().info(f" Import module {module_to_import}")
        robot_spec_module = getattr(importlib.import_module(name=module_to_import),
                                    'CompleteSpecification')

        try:
            ltl_specification = robot_spec_module(name='spec_from_request',
                                                  initial_conditions=request.initial_conditions,
                                                  goals=request.goals,
                                                  action_outcomes=['completed', 'failed'],
                                                  sm_outcomes=request.sm_outcomes,
                                                  strict_order=True)
        except Exception as exc:
            """
            msg = ('ltl_compilation_server: exception in'
                   ' ltl comp server gen_ltl_spec_from_request:')
            """
            self.get_logger().error('\t***LTL Specification Compilation srv failed:\n{}***'
                                    .format(str(exc)))

            error_code = SynthesisErrorCodes.LTL_SPEC_COMPILATION_FAILED
            return LTLSpecification(), error_code

        ltl_specification_msg = self.gen_msg_from_specification(ltl_specification)

        # Behavior Synthesis error code
        error_code = SynthesisErrorCodes.SUCCESS

        return ltl_specification_msg, error_code

    def gen_msg_from_specification(self, spec):
        '''Creates an LTLSpecification message from a structured slugs formatted specification.'''

        ltl_specification_msg = LTLSpecification()

        # The atomic propositions
        ltl_specification_msg.sys_props = spec.sys_props
        ltl_specification_msg.env_props = spec.env_props

        # The 6 formulas that make up a GR(1) specification
        ltl_specification_msg.sys_init = spec.sys_init
        ltl_specification_msg.env_init = spec.env_init
        ltl_specification_msg.sys_trans = spec.sys_trans
        ltl_specification_msg.env_trans = spec.env_trans
        ltl_specification_msg.sys_liveness = spec.sys_liveness
        ltl_specification_msg.env_liveness = spec.env_liveness

        return ltl_specification_msg


def ltl_compilation_server(args=None):
    '''The LTL Specification Compilation server/node.'''

    rclpy.init(args=args)

    ltl_service = LTLCompilationServer()

    rclpy.spin(ltl_service)

    rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    ltl_compilation_server()
