#!/usr/bin/env python

from respec.spec.gr1_specification import GR1Specification
from respec.spec.robot_specification import RobotConfiguration, ActionSpecification
from respec.spec.ts_specification import TransitionSystemSpecification
from respec.spec.ic_specification import InitialConditionsSpecification
from respec.spec.goal_specification import GoalSpecification
from respec.spec.additional_specs_loader import SpecsLoader
import os
from ament_index_python.packages import get_package_share_directory
from ltl_synthesizer.StructuredSlugsParser import compiler as slugs_compiler
import sys
import subprocess
'''
The module defines the components that make up a LTL specification for ATLAS.

The name of the class below, CompleteSpecification, should be the same for all
robots to facilitate integration with ROS. Only the module's name and the
details of the class's constructor should change between robots.
'''

SM_OUTCOME_SUCCESS = 'finished'
SM_OUTCOME_FAILURE = 'failed'


class CompleteSpecification(GR1Specification):
    '''
    Upon construction, this class generates LTL specifications for individual
    subcomponents of ATLAS (BDI control mode transition system, action
    preconditions) as well as LTL specifications for the objective and the
    initial conditions. It then merges them onto the object itself.
    '''

    def __init__(self, name, initial_conditions, goals, env_props=[], sys_props=[],
                 action_outcomes=['completed', 'failed'],
                 sm_outcomes=[SM_OUTCOME_SUCCESS, SM_OUTCOME_FAILURE],
                 strict_order=True, fpath=""):

        super(CompleteSpecification, self).__init__(spec_name=name,
                                                    env_props=env_props,
                                                    sys_props=[])

        self._check_input_arguments(initial_conditions, goals,
                                    action_outcomes, sm_outcomes)

        # Load control modes and action preconditions from config file
        system_config = RobotConfiguration('system_wide')
        control_mode_ts = system_config.ts
        system_preconditions = system_config.preconditions
        # Generate a LTL specification governing BDI control modes
        # FIX: infer control modes of interest from input arguments and actions
        modes_of_interest = []
        ts_spec = TransitionSystemSpecification(
                                    ts=control_mode_ts,
                                    props_of_interest=modes_of_interest,
                                    outcomes=action_outcomes)

        # Generate LTL specification governing action and preconditions
        action_spec = ActionSpecification(preconditions=system_preconditions)

        for my_action in sys_props:
            action_spec.handle_new_action(action=my_action,
                                          act_out=True,
                                          outcomes=action_outcomes)

        for goal in goals:
            if goal not in ts_spec.ts.keys():  # topology is handled above
                action_spec.handle_new_action(action=goal,
                                              act_out=True,
                                              outcomes=action_outcomes)

        # Generate LTL specification governing the achievement of goals ...
        goal_spec = GoalSpecification()
        # goal_spec.handle_single_liveness(goals=goals,
        #                                  outcomes=sm_outcomes,
        #                                  strict_order=strict_order)
        if SM_OUTCOME_FAILURE in sm_outcomes:
            # Add LTL formula tying all the things that can fail to SM outcome1
            failure_conditions = list(ts_spec.ts.keys()) + action_spec.all_actions
            assert len(failure_conditions) == len(set(failure_conditions))
            goal_spec.handle_any_failure(conditions=failure_conditions,
                                         failure=SM_OUTCOME_FAILURE)

    #     # Merge these specifications. Initial conditions are still missing.
        # self.merge_gr1_specifications([ts_spec, action_spec, goal_spec])
        # self.merge_gr1_specifications([goal_spec, action_spec])
        # self.merge_gr1_specifications([action_spec])
    #     # Now generate LTL formulas encoding all of the initial conditions

        ic_spec = InitialConditionsSpecification()
        ic_spec.set_ics_from_spec(spec=self,
                                  true_props=initial_conditions)
        # Finally, also merge the initial conditions specification
        # self.merge_gr1_specifications([ic_spec])

        if fpath is not None:
            loaded_specs = SpecsLoader(fpath)
            self.merge_gr1_specifications([loaded_specs])

        self.write_structured_slugs_file(os.getcwd())
        self.convert_structured_slugs_to_slugsin(name)
        try:
            synthesizable, automaton_file = self.call_slugs_synthesizer(name)
        except Exception as e:
            print("failed {}".format(e))

    def _check_input_arguments(self, initial_conditions, goals,
                               action_outcomes, sm_outcomes):

        if len(action_outcomes) > len(sm_outcomes):
            raise NotImplementedError('The specification cannot handle more action outcomes {0}\
                                       than State Machine outcomes {1}'
                                      .format(action_outcomes, sm_outcomes))

    def write_structured_slugs_from_msg(self, path, name):
        '''Create the structuredslugs file and write the 8 sections.'''
        # # The directory where specs and automata are saved:
        synthesizer_pkg_dir = get_package_share_directory('ltl_synthesizer')
        # # Could get the share package directory of ltl_synthesizer, but that'll just cause issues
        # # with the commands further below.
        specs_dir_path = os.path.join(synthesizer_pkg_dir, 'synthesis_byproducts')

        # The directory where this spec will be saved:
        this_folder_path = os.path.join(specs_dir_path, path)
        if not os.path.exists(this_folder_path):
            os.makedirs(this_folder_path)

        structured_slugs_file = name + '.structuredslugs'

        full_file_path = os.path.join(this_folder_path, structured_slugs_file)

        with open(full_file_path, 'w') as spec_file:
            # System and environment propositions
            self._write_input(spec_file)
            self._write_output(spec_file)
            # Initial Conditions
            self._write_sys_init(spec_file)
            self._write_env_init(spec_file)
            # Safety Requirements & Assumptions
            self._write_sys_trans(spec_file)
            self._write_env_trans(spec_file)
            # Liveness Requirements & Assumptions
            self._write_sys_liveness(spec_file)
            self._write_env_liveness(spec_file)

        # Color output 92m = Green
        # self.get_logger().info(' \033[92mSuccessfully created specification file:\033[0m\n'
        #                        '{} in {}\n'.format(structured_slugs_file, this_folder_path))

        return structured_slugs_file, this_folder_path

    def _write_input(self, spec_file):
        if self.env_props is not None:
            self.env_props.sort(key=lambda item: (len(item), item))
            spec_file.write('[INPUT]\n')
            for prop in self.env_props:
                spec_file.write(prop + '\n')
            spec_file.write('\n')

    def _write_output(self, spec_file):
        if self.sys_props is not None:
            self.sys_props.sort(key=lambda item: (len(item), item))
            spec_file.write('[OUTPUT]\n')
            for prop in self.sys_props:
                spec_file.write(prop + '\n')
            spec_file.write('\n')

    def _write_sys_init(self, spec_file):
        if self.sys_init is not None:
            self.sys_init.sort(key=lambda item: (len(item), item))
            spec_file.write('[SYS_INIT]\n')
            for formula in self.sys_init:
                spec_file.write(formula + '\n')
            spec_file.write('\n')

    def _write_env_init(self, spec_file):
        if self.env_init is not None:
            self.env_init.sort(key=lambda item: (len(item), item))
            spec_file.write('[ENV_INIT]\n')
            for formula in self.env_init:
                spec_file.write(formula + '\n')
            spec_file.write('\n')

    def _write_sys_trans(self, spec_file):
        if self.sys_trans is not None:
            self.sys_trans.sort(key=lambda item: (len(item), item))
            spec_file.write('[SYS_TRANS]\n')
            for formula in self.sys_trans:
                spec_file.write(formula + '\n')
            spec_file.write('\n')

    def _write_env_trans(self, spec_file):
        if self.env_trans is not None:
            self.env_trans.sort(key=lambda item: (len(item), item))
            spec_file.write('[ENV_TRANS]\n')
            for formula in self.env_trans:
                spec_file.write(formula + '\n')
            spec_file.write('\n')

    def _write_sys_liveness(self, spec_file):
        if self.sys_liveness is not None:
            self.sys_liveness.sort(key=lambda item: (len(item), item))
            spec_file.write('[SYS_LIVENESS]\n')
            for formula in self.sys_liveness:
                spec_file.write(formula + '\n')
            spec_file.write('\n')

    def _write_env_liveness(self, spec_file):
        if self.env_liveness is not None:
            self.env_liveness.sort(key=lambda item: (len(item), item))
            spec_file.write('[ENV_LIVENESS]\n')
            for formula in self.env_liveness:
                spec_file.write(formula + '\n')
            spec_file.write('\n')

    def convert_structured_slugs_to_slugsin(self, name):
        '''Call function from StructuredSlugsParser to get slugsin file.'''
        file_dir = '/home/neec23/chrislab/src/chris_create_behavior_synthesis/wgcf/wgcf'
        slugsin_file = name + '.slugsin'

        # TODO: update performConversion so we don't have to do stdout redirection
        with open(slugsin_file, 'w') as sys.stdout:

            slugs_compiler.performConversion(file_dir + '.structuredslugs',
                                             thoroughly=True)

        sys.stdout = sys.__stdout__

        return slugsin_file

    def call_slugs_synthesizer(self, name):
        '''
        Calls SLUGS in order to synthesize automaton from .slugsin input.
        ltl_compilation_server
        Handles potential failures, such as the specification being unrealizable.
        '''

        options = ['--explicitStrategy',
                   '--jsonOutput',
                   '--sysInitRoboticsSemantics']
        slugs_cmd = ['slugs'] + options + [name + '.slugsin', name + '.json']
        slugs_cmd = ' '.join(slugs_cmd)  # convert to string for use with the commands module
        # self.get_logger().info('[ltl_synthesizer] Calling SLUGS: \n\t{}'.format(slugs_cmd))

        (status, slugs_output) = subprocess.getstatusoutput(slugs_cmd)  # Replaces legacy commands

        if status == 0:
            # The command ran successfully. Analyze output.
            synthesizable = self.determine_synthesizability(slugs_output)
            automaton_file = name + '.json'
        else:
            # The command did not even run.
            # self.get_logger().error('SLUGS command failed with status: {0}'
            #                         '\nHave you installed slugs? Output: {1}'
            #                         .format(status, slugs_output))
            automaton_file = ''
            synthesizable = False

        return synthesizable, automaton_file

# =========================================================
# Entry point
# =========================================================


def main():  # pragma: no cover

    import pprint
    file_path = '/home/neec23/chrislab/install/respec/share/respec/additional_specs/wgcf.yaml'
    specification = CompleteSpecification('wgcf',
                                          ['begin_game'],
                                          ['completed_game'],
                                          ['goat', 'corn', 'wolf', 'farmer'],
                                          ['move_goat', 'move_corn', 'move_wolf', 'move_null'],
                                          fpath=file_path)

    # print('[INPUT]')
    # pprint.pprint(specification.env_props)
    # print('[OUTPUT]')
    # pprint.pprint(specification.sys_props)
    # print('[SYS_INIT]')
    # pprint.pprint(specification.sys_init)
    # print('[ENV_INIT]')
    # pprint.pprint(specification.env_init)
    # print('[SYS_TRANS]')
    # pprint.pprint(specification.sys_trans)
    # print('[ENV_TRANS]')
    # pprint.pprint(specification.env_trans)
    # print('[SYS_LIVENESS]')
    # pprint.pprint(specification.sys_liveness)
    # print('[ENV_LIVENESS]')
    # pprint.pprint(specification.env_liveness)


if __name__ == '__main__':  # pragma: no cover
    main()
