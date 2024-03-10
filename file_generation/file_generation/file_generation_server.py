#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import os
import re
import xml.etree.ElementTree as ET
from std_msgs.msg import String
import yaml
import sys


class SynthesisGenerator(Node):

    '''
    Locates all of the files that are tagged for synthesis and inits variables
    to be used in file generation
    '''
    def __init__(self, keyword):
        super().__init__('file_generation', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.dictionary = {}
        self.package_files = {}
        self.synthesis_folder = ''
        # Not convinced this is currently navigating the correct workspace root
        # Some other options were install and build (and src)
        for root, dirs, list_of_files in os.walk(
                os.path.join(os.getenv('WORKSPACE_ROOT'), 'install')):
            # If we use install above  instead of src, it will cause further problems
            # trying to write yaml files in nonexistent directories.
            if 'behavior_synthesis' in dirs and 'sm_generation' in dirs and 'respec' in dirs:
                self.synthesis_folder = root + '/'
            for files in list_of_files:
                if keyword in files:
                    if root in self.dictionary:
                        self.dictionary[root].append(files)
                    else:
                        self.dictionary[root] = [files]
        self.list_of_source_folders = []
        for k, v in self.dictionary.items():  # TODO fix finding of source folders
            for value in v:
                string = k + '/' + value  # suggest os.path.join here
                tree = ET.parse(string)  # parsing package.xml
                root = tree.getroot()
                for export in root.findall('export'):
                    for node in export:
                        # The node is used in synthesis thus get the files src directory
                        if node.tag == 'flexbe_behaviors' or node.tag == 'flexbe_states':
                            words = k.split('/')
                            filepath = k + '/' + words[-1] + '/'
                            if node.tag == 'flexbe_states':
                                filepath = (filepath.split('share')[0] +
                                            'lib/python3.10/site-packages/' + words[-1] + '/')
                            else:
                                filepath = (filepath.split('share')[0] +
                                            'local/lib/python3.10/dist-packages/' + words[-1] + '/')
                            self.list_of_source_folders.append(filepath)
                            for child in root.findall('name'):
                                if child.text in self.package_files:
                                    self.package_files[child.text] = self.package_files[
                                                                        child.text] + filepath
                                else:
                                    self.package_files[child.text] = filepath
                                    # why not list and append?
                                    # what happens for same file name in multiple packages?

        # walk through the source folders and pull the python files out
        # With those files pull the relevent meta data along with the proper tagging data
        self.state_list_of_source_files = []
        self.state_data_set = {}
        self.behavior_data_set = {}
        self.tab_ = '    '                           # used for yaml indenting
        self.dtab_ = '        '                      # used for yaml indenting
        self.state_parameters = {}                   # given by --
        self.state_user_requirements = {}            # given by >#
        self.state_user_output = {}                  # given by #>
        self.state_outcomes = {}                     # given by <=
        self.state_topic_subscriptions = {}          # given by >~
        self.state_implementations = []
        self.system_discrete_abstractions = {}
        self.system_state_implementations_and_DA = {}
        self.system = {}
        self.system_transitions = {}
        self.behavior_outcomes = {}
        self.behavior_user_requirements = {}
        self.behavior_user_output = {}
        self.behavior_DA_names = {}
        self.transition_outcomes = set()
        self.system_output_remapping = {}
        self.other_transitions = set()
        self.needed_in_preconditions = set()
        self.precondition_da_names_set = set()
        self.root_path_and_state_imp_names = {}
        self.system_package_and_DA = {}

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    '''
    goes through all files and finds the ones that are tagged to be used in flexbe
    It then grabs all comments in the file and parses them for relevent tagging data
    '''
    def parseFiles(self):
        for folders in self.list_of_source_folders:
            for root, dirs, files in os.walk(folders):  # TODO fix the os.walk
                for file_name in files:
                    if ('.py' in file_name and '__init__' not in file_name
                            and '.pyc' not in file_name):
                        self.state_list_of_source_files.append(os.path.join(root, file_name))
        # The list of source files now has all python implementations
        # open and parse the files to find the tags
        for files in self.state_list_of_source_files:
            opened_file = open(files, 'r')  # with open (no close?) # TODO fix
            append = False
            state_data = ''
            behavior_data = ''
            title = ''
            file_type = ''

            file_split = files.split('/')  # creates the file path into a list
            del file_split[-1]  # removes the file extension from the root
            file_root = '/'.join(file_split)  # gets us the file root for this specific file
            for lines in opened_file.readlines():
                # This loop should be in function called "get_sub_class_data")
                if lines.split(' ')[0] == 'class':
                    line = lines.strip()  # Start by getting rid of newline
                    title = line.split(' ')[1].split('(')[0]
                    try:
                        # Sometimes python files in flexbe folders are not subclasses
                        file_type = line.split(' ')[1].split('(')[1].split(')')[0].strip()
                    except Exception:
                        file_type = "None"   # file_type or "sub_class"?
                if lines.strip().startswith('self.name'):
                    behavior_data = ('$$ ' + lines.strip().split('=')[1].strip().strip('\'')
                                     + '\n' + behavior_data)
                if (lines.strip().startswith('_state_machine = OperatableStateMachine')
                        and file_type == 'Behavior'):
                    behavior_data = (behavior_data + '<=' + lines.strip().split(
                                     '[')[1].split(']')[0].strip() + '\n')
                triple_double_quote = '"""'
                triple_single_quote = "'''"
                if triple_double_quote in lines or triple_single_quote in lines:
                    append = not append
                elif append:
                    state_data += lines
                    behavior_data += lines
            if file_type == 'EventState':
                if file_root in self.root_path_and_state_imp_names:
                    # ensures that this is only done if the python file is considered valid
                    self.root_path_and_state_imp_names[file_root].append(title)
                else:
                    self.root_path_and_state_imp_names[file_root] = [title]
                self.state_data_set[title] = state_data
            elif file_type == 'Behavior':
                if file_root in self.root_path_and_state_imp_names:
                    # ensures that this is only done if the python file is considered valid
                    self.root_path_and_state_imp_names[file_root].append(title)
                else:
                    self.root_path_and_state_imp_names[file_root] = [title]
                self.behavior_data_set[title] = behavior_data
            else:
                warning = '{} was not a configured state or behavior located at {}'.format(
                    title, files)
                self.get_logger().warn(warning)
            # print(self.behavior_data_set)
        # With the data in hand parse the data for all state implementations
        for name, data in self.state_data_set.items():
            self.state_implementations.append(name)
            # Parse the data
            parsing_data = data.split('\n')
            for lines in parsing_data:
                if lines.strip().startswith('--'):
                    output = re.split(' {2,}|\t', lines.split('--')[1])
                    output = list(filter(lambda a: a != '', output))
                    if name in self.state_parameters:
                        self.state_parameters[name].append(output)
                    else:
                        self.state_parameters[name] = [output]
                elif lines.strip().startswith('>#'):
                    output = re.split(' {2,}|\t', lines.split('>#')[1])
                    output = list(filter(lambda a: a != '', output))
                    if name in self.state_user_requirements:
                        self.state_user_requirements[name].append(output)
                    else:
                        self.state_user_requirements[name] = [output]
                elif lines.strip().startswith('#>'):
                    output = re.split(' {2,}|\t', lines.split('#>')[1])
                    output = list(filter(lambda a: a != '', output))
                    if name in self.state_user_output:
                        self.state_user_output[name].append(output)
                    else:
                        self.state_user_output[name] = [output]
                # TODO tags below need to be formatted properly in many of the flexbe files
                # not currently being utilized in the file_generation demos
                elif lines.strip().startswith('<='):
                    output = re.split(' {2,}|\t', lines.split('<=')[1])
                    output = list(filter(lambda a: a != '', output))
                    if len(output) > 1:
                        self.transition_outcomes.add(output[1].strip())
                        if name in self.state_outcomes:
                            self.state_outcomes[name].append(output)
                        else:
                            self.state_outcomes[name] = [output]
                elif lines.strip().startswith('>~'):
                    output = re.split(' {2,}|\t', lines.split('>~')[1])[0].strip()
                    if name in self.state_topic_subscriptions:
                        self.state_topic_subscriptions[name].append(output)
                    else:
                        self.state_topic_subscriptions[name] = [output]
        # Do the same things for behaviors
        for name, data in self.behavior_data_set.items():
            parsing_data = data.split('\n')
            for lines in parsing_data:
                if lines.strip().startswith('$$'):
                    names = re.split(' {2,}|\t', lines.split('$$ ')[1])[0].strip()
                    da = names.lower().replace(' ', ' ')
                    self.behavior_DA_names[names] = da
                elif lines.strip().startswith('>#'):
                    output = re.split(' {2,}|\t', lines.split('>#')[1])
                    output = list(filter(lambda a: a != '', output))
                    if names in self.behavior_user_requirements:
                        self.behavior_user_requirements[names].append(output)
                    else:
                        self.behavior_user_requirements[names] = [output]
                elif lines.strip().startswith('#>'):
                    output = re.split(' {2,}|\t', lines.split('#>')[1])
                    output = list(filter(lambda a: a != '', output))
                    if names in self.behavior_user_output:
                        self.behavior_user_output[names].append(output)
                    else:
                        self.behavior_user_output[names] = [output]
                elif lines.strip().startswith('<='):
                    base_output = (lines.split('<=')[1].split(','))
                    output = [x.strip() for x in base_output]
                    # TODO fix behavior outcome processing and uncomment this block
                    # if names in self.behavior_outcomes:
                    #     self.behavior_outcomes[names].append(output)
                    # else:
                    #     self.behavior_outcomes[names] = [output]

    '''
    Generates the LTL Specification server for the system wide specification
    '''
    def generateLTLSpecServer(self):
        target_file = self.synthesis_folder + ("ltl_specification/ltl_specification/"
                                               "system_wide_specification.py")
        pass
        if False:
            target = open(target_file, 'w')
            target.write("#!/usr/bin/env python\n\n")

            # Generated code warning to be placed at the top of file.
            target.write('###########################################################\n')
            target.write('#               WARNING: Generated Code!                  #\n')
            target.write('# Manual changes will get lost if file is generated again.#\n')
            target.write('#           Generated by file_generation_server           #\n')
            target.write('###########################################################\n\n')

            # write necessary imports, with relative paths when needed
            target.write('import pprint\n')
            target.write('from respec.spec.robot_specification import'
                         ' RobotConfiguration, ActionSpecification\n')
            target.write('from respec.spec.ic_specification import'
                         ' InitialConditionsSpecification\n')
            target.write('from respec.spec.goal_specification import GoalSpecification\n')
            target.write('from respec.spec.ts_specification import'
                         ' TransitionSystemSpecification\n')
            target.write('from respec.spec.gr1_specification import GR1Specification\n\n')

            target.write("'''\nThe name of the class below, CompleteSpecification,"
                         " should be the same for all robots\nto facilitate integration with ROS."
                         " Only the module's name and the details\nof the class's constructor"
                         " should change between robots.\n'''\n\n")

            # This might need to be changed based on the current outputs of respec
            target.write('SM_OUTCOME_SUCCESS = \'finished\'\n')

            # This might need to be changed based on the current outputs of respec
            target.write('SM_OUTCOME_FAILURE = \'failed\'\n')
            target.write('ALL_SM_OUTCOMES = [SM_OUTCOME_SUCCESS, SM_OUTCOME_FAILURE]\n\n\n')
            target.write('class CompleteSpecification(GR1Specification):\n')
            target.write(("{}'''\n").format(self.tab_))
            target.write('{}This system is built dynamically based on the current system launched\n'
                         .format(self.tab_))
            target.write("{}'''\n\n".format(self.tab_))

            # This might need to be changed based on the current outputs of respec
            target.write('{}def __init__(self, name, initial_conditions, goals, action_outcomes='
                         '[\'completed\', \'failed\'],\n{}{} sm_outcomes=[SM_OUTCOME_SUCCESS,'
                         ' SM_OUTCOME_FAILURE], strict_order=True):\n\n'
                         .format(self.tab_, self.dtab_, self.dtab_))
            target.write('{}super(CompleteSpecification, self).__init__(spec_name=name, env_props='
                         '[], sys_props=[])\n\n'.format(self.dtab_))
            target.write('{}self._check_input_arguments(initial_conditions, goals, action_outcomes,'
                         ' sm_outcomes)\n\n'.format(self.dtab_))
            target.write('{}# Load control modes and action preconditions from config file\n'
                         .format(self.dtab_))
            target.write('{}system_wide_config = RobotConfiguration(\'system_wide\')\n'
                         .format(self.dtab_))
            target.write('{}control_mode_ts = system_wide_config.ts\n'.format(self.dtab_))
            target.write('{}system_wide_preconditions = system_wide_config.preconditions\n\n'
                         .format(self.dtab_))
            target.write('{}# Generate a LTL specification governing BDI control modes\n'
                         .format(self.dtab_))
            target.write('{}# FIX: infer control modes of interest from'
                         ' input arguments and actions\n'
                         .format(self.dtab_))
            target.write('{}modes_of_interest = []\n'.format(self.dtab_))
            target.write('{}ts_spec = TransitionSystemSpecification(ts=control_mode_ts,\n'
                         '{}props_of_interest=modes_of_interest,\n{}outcomes=action_outcomes)\n\n'
                         .format(self.dtab_, self.dtab_*6, self.dtab_*6))
            target.write('{}# Generate LTL specification governing action and preconditions\n'
                         .format(self.dtab_))
            target.write('{}action_spec = ActionSpecification(preconditions='
                         'system_wide_preconditions)\n'.format(self.dtab_))
            target.write('{}for goal in goals:\n'.format(self.dtab_))
            target.write('{}{}if goal not in ts_spec.ts.keys():  # topology is handled above\n'
                         .format(self.dtab_, self.tab_))
            target.write('{}{}action_spec.handle_new_action(action=goal, act_out=True, outcomes='
                         'action_outcomes)\n\n'.format(self.dtab_, self.dtab_))
            target.write('{}# Generate LTL specification governing the achievement of goals ...\n'
                         .format(self.dtab_))
            target.write('{}goal_spec = GoalSpecification()\n'.format(self.dtab_))
            target.write('{}goal_spec.handle_single_liveness(goals=goals, outcomes=sm_outcomes,'
                         ' strict_order=True)\n\n'.format(self.dtab_))
            target.write('{}# All the things that can fail:\n'.format(self.dtab_))
            target.write('{}failure_conditions = list(ts_spec.ts.keys())'
                         ' + action_spec.all_actions\n'
                         .format(self.dtab_))
            target.write('{}# assert len(failure_conditions) == len(set(failure_conditions))\n\n'
                         .format(self.dtab_))
            target.write('{}if SM_OUTCOME_FAILURE in sm_outcomes:\n'.format(self.dtab_))
            target.write('{}{}# Add LTL formula tying all the things that can fail to SM outcome\n'
                         .format(self.dtab_, self.tab_))
            target.write('{}{}goal_spec.handle_any_failure(conditions=failure_conditions, failure='
                         'SM_OUTCOME_FAILURE)\n'.format(self.dtab_, self.tab_))
            target.write('{}else:\n'.format(self.dtab_))
            target.write('{}{}# If anything fails, retry (re-activate) until it succeeds\n'
                         .format(self.dtab_, self.tab_))
            target.write('{}{}goal_spec.handle_retry_after_failure(failures=failure_conditions)\n\n'
                         .format(self.dtab_, self.tab_))
            target.write('{}# Merge these specifications. Initial conditions are still missing.\n'
                         .format(self.dtab_))
            target.write('{}self.merge_gr1_specifications([ts_spec, action_spec, goal_spec])\n\n'
                         .format(self.dtab_))
            target.write('{}# Now generate LTL formulas encoding all of the initial conditions\n'
                         .format(self.dtab_))
            target.write('{}ic_spec = InitialConditionsSpecification()\n'.format(self.dtab_))
            target.write('{}ic_spec.set_ics_from_spec(spec=self, true_props=initial_conditions)'
                         '\n\n'.format(self.dtab_))
            target.write('{}# Finally, also merge the initial conditions specification\n'
                         .format(self.dtab_))
            target.write('{}self.merge_gr1_specifications([ic_spec])\n\n'.format(self.dtab_))
            target.write('{}def _check_input_arguments(self, initial_conditions, goals,'
                         ' action_outcomes, sm_outcomes):\n\n'.format(self.tab_))
            target.write('{}if any([out not in ALL_SM_OUTCOMES for out in sm_outcomes]):\n'
                         .format(self.dtab_))
            target.write('{}{}raise NotImplementedError(\'Some SM outcomes: {{0}} are unknown.'
                         ' Expected them to be\'\n{}{}  \'subseteq of {{1}}\'.format('
                         'sm_outcomes, ALL_SM_OUTCOMES))\n\n'
                         .format(self.dtab_, self.tab_, self.dtab_*4, self.tab_))

            target.write('# =========================================================\n')
            target.write('# Entry point\n')
            target.write('# =========================================================\n\n\n')
            target.write('def main():  # pragma: no cover\n\n')
            target.write("{}specification = CompleteSpecification(name='system_wide',\n{}  "
                         "initial_conditions=['log_no'],\n{}  goals=['log_yes'])\n\n"
                         .format(self.tab_, self.dtab_*5, self.dtab_*5))
            target.write("{}print('[INPUT]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.env_props)\n'.format(self.tab_))
            target.write("{}print('[OUTPUT]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.sys_props)\n'.format(self.tab_))
            target.write("{}print('[SYS_INIT]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.sys_init)\n'.format(self.tab_))
            target.write("{}print('[ENV_INIT]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.env_init)\n'.format(self.tab_))
            target.write("{}print('[SYS_TRANS]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.sys_trans)\n'.format(self.tab_))
            target.write("{}print('[ENV_TRANS]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.env_trans)\n'.format(self.tab_))
            target.write("{}print('[SYS_LIVENESS]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.sys_liveness)\n'.format(self.tab_))
            target.write("{}print('[ENV_LIVENESS]')\n".format(self.tab_))
            target.write('{}pprint.pprint(specification.env_liveness)\n'.format(self.tab_))
            target.write('\n\nif __name__ == "__main__":\n')
            target.write('{}main()\n'.format(self.tab_))
            target.close()

    def loadCapabilities(self, filePath):
        print(f"file_generation_server:  load capabilities from {filePath}", flush=True)
        with open(filePath, 'r') as file:
            params = yaml.safe_load(file)
            try:
                self.system = params['file_generation']['ros__parameters']['capabilities']
                print(f"    system capabilities :\n{self.system}", flush=True)
                for da in self.system:
                    if self.system[da]['interface'] in self.state_implementations:
                        if self.system[da]['interface'] in self.system_state_implementations_and_DA:
                            self.system_state_implementations_and_DA[self.system[da]
                                                                     ['interface']].append(da)
                        else:
                            self.system_state_implementations_and_DA[self.system[da]
                                                                     ['interface']] = [da]
                    elif self.system[da]['interface'] in self.system_discrete_abstractions:
                        self.system_discrete_abstractions[self.system[da]['interface']].append(da)
                    else:
                        self.system_discrete_abstractions[self.system[da]['interface']] = [da]

                for AC in self.system_discrete_abstractions:
                    for state_imp in self.state_topic_subscriptions:
                        if AC in self.state_topic_subscriptions[state_imp]:
                            for da in self.system_discrete_abstractions[AC]:
                                if state_imp in self.system_state_implementations_and_DA:
                                    self.system_state_implementations_and_DA[state_imp].append(da)
                                else:
                                    self.system_state_implementations_and_DA[state_imp] = [da]
                for da in self.system:
                    if 'request_modes' in self.system[da]:
                        for modes in self.system[da]['request_modes']:
                            if modes in self.system_transitions:
                                self.system_transitions[modes].append(da)
                            else:
                                self.system_transitions[modes] = [da]
                for da in self.system:
                    if 'output_mapping' in self.system[da]:
                        if da in self.system_output_remapping:
                            self.system_output_remapping[da].append(self.system[da]
                                                                    ['output_mapping'])
                        else:
                            self.system_output_remapping[da] = self.system[da]['output_mapping']
                for da in self.system:
                    if 'package' in self.system[da]:
                        if da in self.system_package_and_DA:
                            self.system_package_and_DA[da].append(self.system[da]['package'])
                        else:
                            self.system_package_and_DA[da] = self.system[da]['package']
            except Exception:
                self.get_logger().error('failed to load capabilities from {}'.format(filePath))
                raise ValueError('No parameters loaded to the file_generation Node')

    # '''
    # Checks the param server and updates any configurations based on /synthesis/capabilities
    # '''
    # def updateParams(self):
    #     capabilities = {}
    #     params = self._parameters
    #     print(params)
    #     print(self.get_parameter('file_generation').value)
    #     print(self._parameters['file_generation'])  # Doesn't work
    #     for x in params:
    #         if x.startswith('capabilities.'):
    #             key = self.get_parameter(x).name
    #             value = self.get_parameter(x).value
    #             capabilities[key] = value
    #     if len(capabilities) > 0:
    #         self.system = capabilities
    #         for da in self.system:
    #             if da.endswith('.interface'):
    #                 # print('\tinterface found')
    #                 if da in self.state_implementations:
    #                     if da in self.system_state_implementations_and_DA:
    #                         # print('\t  entered a')
    #                         self.system_state_implementations_and_DA[
    #                                 da].append(da)

    #                     else:
    #                         # print('\t  entered b')
    #                         self.system_state_implementations_and_DA[
    #                             da] = [da]

    #                 elif da in self.system_discrete_abstractions:
    #                     # print('\t  entered c')
    #                     self.system_discrete_abstractions[da].append(da)
    #                 else:
    #                     # print('\t  entered d')
    #                     self.system_discrete_abstractions[da] = [da]

    #         # take the systems discrete abstractions and map them to a state implementation
    #         # print(self.system_discrete_abstractions)
    #         # print(self.state_topic_subscriptions)
    #         # The system_state_implentations_and_DA variable
    #         for AC in self.system_discrete_abstractions:
    #             for state_imp in self.state_topic_subscriptions:
    #                 if AC in self.state_topic_subscriptions[state_imp]:
    #                     # print('\t\tmatching AC')
    #                     for da in self.system_discrete_abstractions[AC]:
    #                         if state_imp in self.system_state_implementations_and_DA:
    #                             # print('\t\tentered a')
    #                             self.system_state_implementations_and_DA[state_imp].append(da)
    #                         else:
    #                             # print('\t\tentered b')
    #                             self.system_state_implementations_and_DA[state_imp] = [da]
    #         # take the request modes and make the basic transition system_wide
    #         for da in self.system:
    #             if da.endswith('request_modes'):
    #                 # print('entering request:')
    #                 # print('\t   ', da)
    #                 for modes in self.system[da]:
    #                     # print('\t\t', self.system_transitions)
    #                     # print('\t\t', modes)
    #                     split_da = da.split('.')
    #                     if modes in self.system_transitions:
    #                         # print('\t   path a')
    #                         self.system_transitions[modes].append(split_da[1])
    #                     else:
    #                         # print('\t   path b')
    #                         self.system_transitions[modes] = [split_da[1]]
    #         for da in self.system:
    #             if 'output_mapping' in da:
    #                 if da in self.system_output_remapping:
    #                     self.system_output_remapping[da].append(self.system[da])
    #                 else:
    #                     self.system_output_remapping[da] = [self.system[da]]
    #         # print(self.system_package_and_DA)  # empty (currently)
    #         for da in self.system:
    #             if da.endswith('package'):
    #                 # print('\t', self.system_package_and_DA)
    #                 # print('\t', da)
    #                 if da in self.system_package_and_DA:
    #                     # print('\t  entered a')
    #                     self.system_package_and_DA[da].append(self.system[da])
    #                 else:
    #                     # print('\t  entered b')
    #                     self.system_package_and_DA[da] = [self.system[da]]

    #         # print(self.state_topic_subscriptions)  # Remains empty
    #         # print('\t\t', self.system_transitions)
    #         # print(self.system_output_remapping)  # Remains empty
    #         # print('\t', self.system_package_and_DA)
    #     else:
    #         self.get_logger().error('No parameters loaded to the file_generation Node')
    #         raise ValueError('No parameters loaded to the file_generation Node')

    '''
    A helper class designed to write outcome mappings with outcomes defined by the user
    '''
    def writeOutcomeMappings(self, target, state_imps, da, outcomes):
        target.write('{}output_mapping:\n'.format(self.tab_))
        for outcome in self.transition_outcomes:  # should only be completed and failure
            target.write('{}{}_{}: ['.format(self.dtab_, da, outcome[0]))
            first = True
            if state_imps in outcomes:
                for results in outcomes.get(state_imps):
                    try:
                        if results[1].strip() == outcome.strip():
                            if first:
                                target.write('{}'.format(results[0].strip()))
                                first = not first
                            else:
                                target.write(', {}'.format(results[0].strip()))
                    except Exception as e:
                        print('error line 518', e)
            target.write(']\n')

    def writeDA(self, target, da, state_imps):
        target.write('{}_a:\n'.format(da))
        target.write('{}class_decl:\n'.format(self.tab_))
        target.write('{}name: {}\n'.format(self.dtab_, state_imps))
        target.write('{}param_names: '.format(self.dtab_))
        if self.system[da].get('parameters') and state_imps in self.state_parameters:
            counter = 0
            for values in self.system[da]['parameters']:

                for params in self.state_parameters[state_imps]:

                    if values.strip() == params[0].strip():
                        target.write('\n')
                        target.write('{}{}- {}'.format(self.dtab_, self.tab_, values))
                        counter += 1
            if counter == 0:
                target.write('[]')

            target.write('\n{}param_values: '.format(self.dtab_))
            # for the parameter values we need to match the values with their names and their types
            # String needs '""'
            # String arrays needs "[]"
            counter = 0
            for values in self.system[da]['parameters']:
                for params in self.state_parameters[state_imps]:
                    if values.strip() == params[0].strip():
                        target.write('\n')
                        if (params[1].strip().lower() == 'string'
                           or params[1].strip().lower() == 'String'):
                            target.write('{}{}- \'"{}"\''.format(self.dtab_, self.tab_,
                                                                 self.system[da]
                                                                            ['parameters']
                                                                            [values]))
                            counter += 1
                        elif (params[1].strip().lower() == 'string[]'
                              or params[1].strip().lower() == 'String[]'):
                            target.write('{}{}- "{}"'.format(self.dtab_, self.tab_,
                                                             self.system[da]
                                                                        ['parameters']
                                                                        [values]))
                            counter += 1
                        else:
                            target.write('{}{}- {}'.format(self.dtab_, self.tab_,
                                                           self.system[da]
                                                                      ['parameters']
                                                                      [values]))
                            counter += 1
            if counter == 0:
                target.write('[]\n')
            else:
                target.write('\n')
        else:
            target.write('[]\n')
            target.write('{}param_values: []\n'.format(self.dtab_))

        self.writeOutcomeMappings(target, state_imps, da, self.state_outcomes)

        target.write('{}autonomy: 0\n'.format(self.tab_))
        if self.state_user_output.get(state_imps):
            target.write('{}userdata_keys: ['.format(self.tab_))
            first = True
            for results in self.state_user_output.get(state_imps):
                if first:
                    target.write('{}'.format(results[0].strip()))
                    first = False
                else:
                    target.write(', {}'.format(results[0].strip()))
            target.write(']\n')
        if self.state_user_output.get(state_imps):
            target.write('{}userdata_remapping: ['.format(self.tab_))
            first = True
            for results in self.state_user_output.get(state_imps):
                stripped_result = results[0].strip()
                if da in self.system_output_remapping:
                    if stripped_result in self.system_output_remapping[da]:
                        stripped_result = self.system_output_remapping[da][results[0].strip()]
                if first:
                    target.write('{}'.format(stripped_result))
                    first = False
                else:
                    target.write(', {}'.format(stripped_result))
            target.write(']\n')

    '''
    Writes the discrete abstraction configuration file for every single discrete
    abstraction and matching state implementaion.
    Calls the helper function writeOutcomeMappings
    '''
    def generateDiscreteAbstractions(self):
        # Generates the entire discrete abstraction
        file_target = self.synthesis_folder + ('sm_generation/share/sm_generation/src/sm_generation'
                                               '/configs/system_wide.yaml')
        target = open(file_target, 'w')
        target.truncate()
        target.write('name: system_wide \n' + 'output:\n' + self.tab_ + 'finished: finished \n'
                     + self.tab_ + 'failed: failed \n')
        print("writing in {}".format(file_target), flush=True, file=sys.stderr)
        for da in self.system_discrete_abstractions.values():
            state_imps = self.get_parameter(da[0]).value
            write = da[0].split('.')[1]

            self.writeDA(target, write, state_imps)
        for state_imps, da_list in self.system_state_implementations_and_DA.items():
            for da in da_list:
                if da in self.system_package_and_DA:
                    package = self.system_package_and_DA[da]
                    root_folder = self.package_files[package]
                    if state_imps in self.root_path_and_state_imp_names[root_folder]:
                        self.writeDA(target, da, state_imps)
                else:  # TODO unsure why top half not executing, temp fix.
                    self.writeDA(target, da, state_imps)
        for name, da_name in self.behavior_DA_names.items():
            target.write('{}_a:\n{}class_decl:\n{}name: :BEHAVIOR\n{}behavior_class: {}\n{}'
                         'param_names: []\n{}param_values: []\n'
                         .format(da_name, self.tab_, self.dtab_,
                                 self.dtab_, name, self.dtab_, self.dtab_))

            self.writeOutcomeMappings(target, name, da_name, self.behavior_outcomes)

            target.write('{}autonomy: 0\n' .format(self.tab_))

            if self.behavior_user_output.get(name):
                target.write('{}userdata_keys: ['.format(self.tab_))
                first = True
                for results in self.behavior_user_output.get(name):
                    if first:
                        target.write('{}'.format(results[0].strip()))
                        first = False
                    else:
                        target.write(', {}'.format(results[0].strip()))
                target.write(']\n')
            if self.behavior_user_output.get(name):
                target.write('{}userdata_remapping: ['.format(self.tab_))
                first = True
                for results in self.behavior_user_output.get(name):
                    stripped_result = results[0].strip()
                    if stripped_result in self.system_output_remapping[da_name]:
                        stripped_result = self.system_output_remapping[da_name][results[0].strip()]
                    if first:
                        target.write('{}'.format(stripped_result))
                        first = False
                    else:
                        target.write(', {}'.format(stripped_result))
                target.write(']\n')
            # target.write('{}_a:\n{}class_decl:\n{}name: :BEHAVIOR\n{}behavior_class: {}\n{}'
            #              'param_names: []\n{}param_values: []\n{}output_mapping:\n{}{}_c:'
            #              ' [finished]\n{}{}_f: [failed]\n{}autonomy: 0\n'
            #              .format(da_name, self.tab_, self.dtab_, self.dtab_, name, self.dtab_,
            #                      self.dtab_, self.tab_, self.dtab_, da_name, self.dtab_, da_name,
            #                      self.tab_))
        target.close()

    '''
    A helper class that helps with action preconditions.
    Checks if state implementation / behavior B has requirements of
    state implementation / behavior A
    '''
    def writePreconditions(self, target, library_a, library_b, outputs, requirements):
        for state_imp_a in library_a:
            for state_imp_b in library_b:
                if state_imp_b in outputs and state_imp_a in requirements:
                    for outcomes_b in outputs[state_imp_b]:
                        for requirement_a in requirements[state_imp_a]:
                            if not (state_imp_a == state_imp_b and
                                    outcomes_b[1].strip() == requirement_a[1].strip()):
                                first = True
                                if type(library_a[state_imp_a]) is str:
                                    if type(library_b[state_imp_b]) is str:
                                        result = outcomes_b[0].strip()
                                        if library_b[state_imp_b] in self.system_output_remapping:
                                            if result in self.system_output_remapping[
                                                         library_b[state_imp_b]]:

                                                result = self.system_output_remapping[
                                                    library_b[state_imp_b]][result]
                                        if result == requirement_a[0].strip():
                                            self.precondition_da_names_set.add(
                                                library_a[state_imp_a])

                                            target.write('{}{}:\n'.format(
                                                self.tab_, library_a[state_imp_a]))
                                            target.write('{}- {}\n'.format(
                                                self.dtab_, library_b[state_imp_b]))
                                    else:
                                        for names_b in library_b[state_imp_b]:
                                            result = outcomes_b[0].strip()
                                            if names_b in self.system_output_remapping:
                                                if result in self.system_output_remapping[names_b]:
                                                    result = self.system_output_remapping[names_b][
                                                                                          result]
                                            if result == requirement_a[0].strip():
                                                if first:
                                                    self.precondition_da_names_set.add(
                                                        library_a[state_imp_a])

                                                    target.write('{}{}:\n'.format(
                                                        self.tab_, library_a[state_imp_a]))

                                                    target.write('{}- {}\n'.format(
                                                        self.dtab_, names_b))

                                                    first = False
                                                else:
                                                    target.write('{}- {}\n'.format(
                                                                 self.dtab_, names_b))
                                else:
                                    for names in library_a[state_imp_a]:
                                        if type(library_b[state_imp_b]) is str:
                                            result = outcomes_b[0].strip()
                                            if library_b[state_imp_b] in (
                                                    self.system_output_remapping):

                                                if result in self.system_output_remapping[
                                                                    library_b[state_imp_b]]:
                                                    result = self.system_output_remapping[
                                                        library_b[state_imp_b]][result]

                                            if result == requirement_a[0].strip():
                                                self.precondition_da_names_set.add(names)
                                                target.write('{}{}:\n'.format(self.tab_, names))
                                                target.write('{}- {}\n'.format(
                                                             self.dtab_, library_b[state_imp_b]))
                                        else:
                                            for names_b in library_b[state_imp_b]:
                                                result = outcomes_b[0].strip()
                                                if names_b in self.system_output_remapping:
                                                    if result in self.system_output_remapping[
                                                                                        names_b]:
                                                        result = self.system_output_remapping[
                                                                                names_b][result]
                                                if result == requirement_a[0].strip():
                                                    if first:
                                                        self.precondition_da_names_set.add(names)
                                                        target.write('{}{}:\n'.format(
                                                                     self.tab_, names))
                                                        target.write('{}- {}\n'.format(
                                                                     self.dtab_, names_b))
                                                        first = False
                                                    else:
                                                        target.write('{}- {}\n'.format(
                                                                     self.dtab_, names_b))

    '''
    Writes the discrete transition relationship file for all transitions and preconditions
    calls the function writePreconditions
    '''
    def generateDiscreteTransitionRelations(self):
        # Generate the transition system
        path = 'respec/lib/python3.10/site-packages/respec/config/system_wide_config.yaml'
        file_target = self.synthesis_folder + path
        target = open(file_target, 'w')
        target.write('transition_system:\n')

        for da in self.system:
            if da in self.system_transitions or da in self.behavior_DA_names:
                target.write('{}{}:\n'.format(self.tab_, da))
                for transitions in self.system_transitions[da]:
                    target.write('{}- {}\n'.format(self.dtab_, transitions))
            else:
                for i, j in self.system_transitions.items():
                    if da in j:
                        target.write('{}{}: [] \n'.format(self.tab_, da))
                        self.other_transitions.add(da)
                if da not in self.other_transitions:
                    self.needed_in_preconditions.add(da)

        # Fixes the issue but concerned about the operation cost of this function
        target.write('action_preconditions:\n')

        # checks all state preconditions
        self.writePreconditions(target, self.system_state_implementations_and_DA,
                                self.system_state_implementations_and_DA, self.state_user_output,
                                self.state_user_requirements)

        # checks behaviors
        self.writePreconditions(target, self.behavior_DA_names, self.behavior_DA_names,
                                self.behavior_user_output, self.behavior_user_requirements)

        # checks behaviors against states
        self.writePreconditions(target, self.behavior_DA_names,
                                self.system_state_implementations_and_DA, self.state_user_output,
                                self.behavior_user_requirements)

        # checks states against behaviors
        self.writePreconditions(target, self.system_state_implementations_and_DA,
                                self.behavior_DA_names, self.behavior_user_output,
                                self.state_user_requirements)

        for da in self.needed_in_preconditions:
            if da not in self.precondition_da_names_set:
                target.write('{}{}: []\n'.format(self.tab_, da))

    '''
    This method handles callback functionality.
    Can update everything or individual files
    '''
    def callback(self, request):
        if request == 'update_all_files':
            self.parseFiles()
            # self.updateParams()
            # @todo - should this file name be part of request?
            self.loadCapabilities(self.get_parameter('config_file').value)
            self.generateDiscreteAbstractions()
            self.generateDiscreteTransitionRelations()
            self.generateLTLSpecServer()
        elif request == 'update_DA':
            self.generateDiscreteAbstractions()
        elif request == 'update_DTR':
            self.generateDiscreteTransitionRelations()
        elif request == 'update_LTL':
            self.generateLTLSpecServer()
        else:
            self.get_logger().warn('The request {} was not recognized'.format(request))


'''
sets up the file generation server and sets up the Subscriber to receive additional requests
'''


def _file_generation_server(args=None):
    rclpy.init(args=args)
    file_generation_server = SynthesisGenerator('package.xml')  # "keyword" as file to look at
    file_generation_server.parseFiles()
    # file_generation_server.updateParams()
    file_generation_server.loadCapabilities(file_generation_server.get_parameter('config_file')
                                            .value)
    file_generation_server.generateDiscreteAbstractions()
    file_generation_server.generateDiscreteTransitionRelations()
    file_generation_server.generateLTLSpecServer()

    file_generation_server.create_subscription(String, 'synthesis',
                                               file_generation_server.callback, 5)
    success_message = 'Initial File Generation Successful, Ready to receive additional requests'
    rclpy.node.get_logger('file_generator').info(success_message)

    rclpy.spin(file_generation_server)
    file_generation_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    _file_generation_server()
