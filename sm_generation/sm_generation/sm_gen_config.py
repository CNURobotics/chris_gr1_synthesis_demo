
import rclpy
from .sm_gen_util import class_decl_to_string, clean_variable
from .sm_gen_error import SMGenError
from synthesis_msgs.msg import SynthesisErrorCodes
import sys


class SMGenConfig():
    '''
    A class that represents the configurations needed to generate state
    machines. This class basically creates a bunch of helper methods for
    navigating the configurations as well as the given automata.
    '''
    def __init__(self, config, all_in_vars, all_out_vars, automata):
        self.config = config
        self.logger = rclpy.logging.get_logger('SMGenConfig')
        # List of in/out vars of the substates
        self.all_in_vars = all_in_vars
        self.all_out_vars = all_out_vars

        self.automata = automata

        # Map from what a substate thinks is the final transition to what it
        # actually should be (e.g. "done" -> "finished")
        self.sm_fake_out_to_real_out = config['output']

        # List of outputs of the entire SM
        self.sm_fake_outputs = self.sm_fake_out_to_real_out.keys()

        # To exit this SM, we find states that should exit. At the end, we'll
        # make any transition that goes to one of these states go to the real
        # output.
        self.state_name_to_sm_output = self.get_state_name_to_sm_output()

        '''
        Populate various dictionaries to future functions. These lines
        should should be at the end of the initializer
        '''
        # what class declaration goes with an input variable?
        self.in_var_to_class_decl = {}
        # to map response variables back to their activation variable
        self.in_var_to_out_var = {}
        # the map from class declaration to its out_map
        self.class_decl_to_out_map = {}

        try:
            print(self.config, flush=True, file=sys.stderr)
            for out_var, var_config in self.config.items():
                # If class_decl isn't in var_config,
                # it's not configuration for a state
                if 'class_decl' not in var_config:
                    continue
                class_decl = var_config['class_decl']
                out_map = var_config['output_mapping']
                class_decl_str = class_decl_to_string(class_decl)
                self.class_decl_to_out_map[class_decl_str] = out_map
                for in_var, state_outcome in out_map.items():
                    # check if this is a response variable
                    if in_var in self.all_in_vars and out_var in self.all_out_vars:
                        self.in_var_to_class_decl[in_var] = class_decl
                        self.in_var_to_out_var[in_var] = out_var
            # Make one more pass to handle environment propositions
            # print("in_var_to_class_decl: {}".format(self.in_var_to_class_decl), flush=True, file=sys.stderr)
            # print("all_in_vars: {}".format(self.all_in_vars), flush=True, file=sys.stderr)
            # for in_var in self.all_in_vars:
            #     if not self.is_response_var(in_var):
            #         print("not response_var: {}".format(in_var), flush=True, file=sys.stderr)

            for in_var in self.all_in_vars:
                if not self.is_response_var(in_var):  # it's a sensor prop
                    print(in_var, flush=True, file=sys.stderr)
                    # var_config = self.config[in_var]
                    # class_decl = var_config['class_decl']
                    # out_map = var_config['output_mapping']
                    # class_decl_str = class_decl_to_string(class_decl)
                    # self.class_decl_to_out_map[class_decl_str] = out_map

        except KeyError as e:
            self.logger.error('The configuration file is invalid.')
            print(e, flush=True, file=sys.stderr)
            raise SMGenError(SynthesisErrorCodes.CONFIG_FILE_INVALID)

    def get_init_states(self):
        '''
        Return the initial states. For now, a state is an initial state if it
        has nothing no incoming transitions.
        '''

        # States that do have incoming transitions
        transitioned_to = set()
        for state in self.automata:
            transitioned_to = transitioned_to.union(state.transitions)

        # Remove any initial states that have no True output valuation
        self.removed_states = list()
        for state in self.automata:
            if state.name not in transitioned_to and not any(state.output_valuation):
                self.automata.remove(state)
                self.removed_states.append(state)
                print('Removing state: {}'.format(state.name))

        # States of the (possibly reduced) automaton with no incoming transitions
        not_transitioned_to = list()
        for state in self.automata:
            if state.name not in transitioned_to:
                not_transitioned_to.append(state.name)

        if len(not_transitioned_to) == 0:
            self.logger.warn('[Could not figure out the (real) initial state(s).')
            # raise SMGenError(SynthesisErrorCodes.AUTOMATON_NO_INITIAL_STATE)
            # TEMP: The set above is empty because the init state was removed:
            not_transitioned_to = self.removed_states[0].transitions
            print('New init states: {}'.format(not_transitioned_to))

        # print("Removed States: {}".format(self.removed_states), flush=True, file=sys.stderr)
        # print("Not Transitioned To: {}".format(not_transitioned_to), flush=True, file=sys.stderr)
        # print("Transitioned To {}".format(transitioned_to), flush=True, file=sys.stderr)
        return not_transitioned_to

    def get_state_name_to_sm_output(self):
        '''Create the mapping from the name of a substate that represents
        an exit, to the specific output. E.g. "State5" -> "finished"
        '''
        d = {}
        for state in self.automata:
            outputs = self.get_state_output_vars(state)
            if self.is_sm_output(outputs):
                in_both = [k for k in self.sm_fake_outputs if k in outputs]
                if len(in_both) > 1:
                    self.logger.error('This substate represent multiple final outputs.')
                    raise SMGenError(SynthesisErrorCodes.AUTOMATON_INVALID)
                if len(in_both) == 0:
                    self.logger.error(
                        'This substate represent no final outputs, but expected it to.')
                    raise SMGenError(SynthesisErrorCodes.AUTOMATON_INVALID)
                d[state.name] = in_both[0]

        return d

    def get_sm_real_outputs(self):
        print("Made to sm_real_outputs", flush=True, file=sys.stderr)
        return self.sm_fake_out_to_real_out.values()

    def get_automaton(self, name):
        '''
        Returns the automaton of a given automaton name. Returns None if
        there's no automaton with that name.

        @param name The name of the automaton of interest.
        '''
        names = [a.name for a in self.automata]
        if name not in names:
            return None

        i = names.index(name)
        return self.automata[i]

    def get_transitions(self, state):
        '''
        Deduce the transition needed to go to the next states.

        @param state The state to transitions away from
        @returns A dictionary, that maps next state to the input variable
                that need to be true to go to that next state.

                 For example:
            {
                'State1': ['stand_prep_c'], # input var 1 & 2 need to be true
                'State2': ['stand_prep_c', 'stand_c'],
                ...
            }
        '''
        next_states = [str(x) for x in state.transitions]
        next_states = list(set(next_states) - set([state.name]))  # remove self loop

        transitions = {}
        for next_state_name in next_states:
            next_state = self.get_automaton(next_state_name)
            if next_state is None:
                self.logger.error('Problem trying to get the next state: {0}.'
                                  .format(next_state_name))
                raise SMGenError(SynthesisErrorCodes.AUTOMATON_NEXT_STATE_INVALID)
            input_vals = next_state.input_valuation
            conditions = []
            for idx, val in enumerate(input_vals):
                if val == 1:  # only look at active inputs
                    in_var = self.get_in_var_name(idx)
                    if self.is_response_var(in_var):
                        # only consider this response variable if the source
                        # state activated this response variable.
                        if self.does_state_activate(state, in_var):
                            conditions.append(in_var)
                    else:
                        conditions.append(in_var)
            if len(conditions) > 0:
                transitions[next_state_name] = conditions
            else:
                self.logger.warn('Empty conditions for state: {} with next states: {}'
                                 .format(state.name, next_states))
        return transitions

    def get_substate_name(self, in_var):
        '''
        Return the readable name of the substate associated with an input
        variable.

        If the input variable is associated with an activation variable,
        use the activation variable. Otherwise, just use the input variable.

        @param in_var Input variable associated with this substate.
        '''
        if self.is_response_var(in_var):
            return self.in_var_to_out_var[in_var]
        return in_var

    def get_in_var_name(self, i):
        '''Get the input variable name at index [i] in [in_vars] dict.'''
        if i >= len(self.all_in_vars):
            self.logger.error('There are not {0} input variables.'.format(i))
            raise SMGenError(SynthesisErrorCodes.AUTOMATON_INPUT_VALUATION_INVALID)
        return self.all_in_vars[i]

    def get_out_var_name(self, i):
        '''Get the output variable name at index [i] in [out_vars] dict.'''
        if i >= len(self.all_out_vars):
            self.logger.error('There are not {0} output variables.'.format(i))
            raise SMGenError(SynthesisErrorCodes.AUTOMATON_OUTPUT_VALUATION_INVALID)
        return self.all_out_vars[i]

    def get_state_output_vars(self, state):
        '''Extract what output variables a state is outputting.'''
        out_vals = state.output_valuation
        return [self.get_out_var_name(i) for i, v in enumerate(out_vals) if v == 1]

    def is_sm_output(self, outputs):
        '''Return true iff this state's output valuations indicate that this
        state should transition out of the SM completely.'''
        check = set(outputs)
        return any(k in check for k in self.sm_fake_outputs)

    def get_outcome_name(self, is_concurrent, state_name, condition):
        '''Returns a name needed to transition to the next state.

        is_concurrent: Boolean, which equals if this state is a concurrent
                       state.
        state_name: The name of the next state in the automaton (e.g. "0")
        condition: A map from proposition to value needed for this outcome
                   to happen.
        If it's a concurrent state, combine the conditions in a meaningful way.
        '''
        if not is_concurrent:
            (k, v) = list(condition.items())[0]
            return v

        # Output format is "key1_value1__key2_value2__key3_value3__..."
        outcome_str = '__'.join(['{0}_{1}'.format(clean_variable(k), v)
                                 for (k, v) in condition.items()])
        return outcome_str

    def get_real_name(self, state_name):
        '''Returns the real name of a state. The only time a name isn't real
        is when this is a fake state, at which point this returns the
        corresponding output.
        '''
        if self.is_fake_state(state_name):
            # Could actually be "State 5" -> "done" -> "finished"
            fake_output = self.state_name_to_sm_output[state_name]
            return self.sm_fake_out_to_real_out[fake_output]
        else:
            return state_name

    def get_autonomy_list(self, conditions):
        # return [self.config[out_var]['autonomy']
        #         for out_var, _ in conditions.items()]
        try:
            return [self.config[out_var]['autonomy']
                    for out_var, _ in conditions.items()]
        except KeyError as k:
            self.logger.error('{0} is not in config or variable config dictionary'
                              .format(k.args[0]))
            raise SMGenError(SynthesisErrorCodes.CONFIG_AUTONOMY_INVALID)

    def get_userdata_keys(self, var):
        try:
            if var in self.config:
                var_config = self.config[var]
                if 'userdata_keys' in var_config:
                    return var_config['userdata_keys']
            return ''
        except Exception as e:
            self.logger.error('Failed to get userdata_keys from config: {0}'.format(e.args[0]))
            raise SMGenError(SynthesisErrorCodes.CONFIG_USERDATA_INVALID)

    def get_userdata_remapping(self, var):
        try:
            if var in self.config:
                var_config = self.config[var]
                if 'userdata_remapping' in var_config:
                    return var_config['userdata_remapping']
            return ''
        except Exception as e:
            self.logger.error('Failed to get userdata_remapping from config: {0}'.format(e.args[0]))
            raise SMGenError(SynthesisErrorCodes.CONFIG_USERDATA_INVALID)

    def is_fake_state(self, name):
        '''Returns if a state is a placeholder for an output.'''
        return name in self.state_name_to_sm_output

    def is_response_var(self, in_var):
        '''Returns if an input variable is a response variable.
        A response variable is the counter part to an activation variable
        (e.g. completion, failure, etc.). '''
        return in_var in self.in_var_to_class_decl

    def is_activation_var(self, out_var):
        '''Returns if an output variable is an activation variable.'''
        var_config = self.config[out_var]
        # class_decl = var_config['class_decl']
        out_map = var_config['output_mapping']
        if 'class_decl' not in var_config or 'output_mapping' not in var_config:
            self.logger.error('"class_decl" or "output_mapping" not in var_config')
            raise SMGenError(SynthesisErrorCodes.CONFIG_VARIABLE_CONFIG_INVALID)
        return any([in_var in self.all_in_vars
                    for in_var, _ in out_map.items()])

    def get_out_map(self, class_decl):
        '''Get the output mapping associated with a class declaration.'''
        class_decl_str = class_decl_to_string(class_decl)
        return self.class_decl_to_out_map[class_decl_str]

    def get_class_decl(self, var):
        '''Get the class declaration associated with a variable.'''
        if var in self.config:
            var_config = self.config[var]
            if 'class_decl' not in var_config:
                self.logger.error('"class_decl" not in var_config')
                raise SMGenError(SynthesisErrorCodes.CONFIG_VARIABLE_CONFIG_INVALID)
            return var_config['class_decl']
        elif var in self.in_var_to_class_decl:
            return self.in_var_to_class_decl[var]

    def does_state_activate(self, state, in_var):
        '''Returns if [state] activate something that has [in_var] as a
        potential response.'''
        # If in_var is not in this directionary, then it's not even a response
        # variable.
        if in_var not in self.in_var_to_out_var:
            return False
        out_var = self.in_var_to_out_var[in_var]
        state_out_vars = self.get_state_output_vars(state)
        return out_var in state_out_vars
