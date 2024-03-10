'''
Useful functions for state machine generation.
'''

from flexbe_msgs.msg import StateInstantiation
import sys


def remove_duplicate_pairs(lst1, lst2):
    '''Remove duplicates in two lists. Visually, put the two lists on top of
    each other and a pair is two elements in a vertical line.'''
    if len(lst1) == 0:
        return [], []
    pairs = zip(lst1, lst2)
    pairs = list(set(pairs))
    new_lists = zip(*pairs)
    lists_to_return = list(new_lists)
    return lists_to_return[0], lists_to_return[1]


def new_si(state_path, state_class, behavior_class, outcomes, transitions, initial_state,
           p_names, p_vals, autonomy=[],
           userdata_keys=[], userdata_remapping=[]):
    ''' Create a new SI. '''
    si = StateInstantiation()
    si.state_path = state_path
    si.state_class = state_class
    si.behavior_class = behavior_class
    if len(transitions) > 0:  # it's not the top level SM
        outcomes, transitions = remove_duplicate_pairs(outcomes, transitions)
    si.outcomes = outcomes
    si.transitions = transitions
    if initial_state is not None:
        si.initial_state_name = initial_state
    si.parameter_names = p_names
    si.parameter_values = p_vals
    si.autonomy = []  # temporary until fix on FlexBE side
    si.userdata_keys = userdata_keys
    si.userdata_remapping = userdata_remapping
    # print("**** finished new SI ****", flush=True, file=sys.stderr)
    return si


def class_decl_to_string(class_decl):
    '''Convert the class_decl map to one string by combining the
    declaration with the parameters passed.'''
    decl = class_decl['name']
    decl += '('
    decl += ','.join(class_decl['param_values'])
    decl += ')'
    # TODO this is probably what needs to be changed
    if decl == ':BEHAVIOR()':
        decl = class_decl['behavior_class']
    return decl


def clean_variable(var):
    ''' Remove the _* suffix if it it present. '''
    if var[-2] == '_':
        return var[:-2]
    return var
