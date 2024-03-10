#!/usr/bin/env python

from respec.spec.robot_specification import RobotConfiguration, ActionSpecification
from respec.spec.ic_specification import InitialConditionsSpecification
from respec.spec.goal_specification import GoalSpecification
from respec.spec.ts_specification import TransitionSystemSpecification
from respec.spec.gr1_specification import GR1Specification

SM_OUTCOME_SUCCESS = 'finished'
SM_OUTCOME_FAILURE = 'failed'
ALL_SM_OUTCOMES = [SM_OUTCOME_SUCCESS, SM_OUTCOME_FAILURE]


class CompleteSpecification(GR1Specification):
    '''
    Upon construction, this class generates LTL specifications for individual
    subcomponents of simple (BDI control mode transition system, action
    preconditions) as well as LTL specifications for the objective and the
    initial conditions. It then merges them onto the object itself.
    '''

    def __init__(self, name, initial_conditions, goals,
                 action_outcomes=['completed', 'failed'],
                 sm_outcomes=[SM_OUTCOME_SUCCESS, SM_OUTCOME_FAILURE],
                 strict_order=True):

        super(CompleteSpecification, self).__init__(spec_name=name,
                                                    env_props=[],
                                                    sys_props=[])

        self._check_input_arguments(initial_conditions, goals,
                                    action_outcomes, sm_outcomes)

        # Load control modes and action preconditions from config file
        simple_config = RobotConfiguration('simple')
        control_mode_ts = simple_config.ts
        simple_preconditions = simple_config.preconditions

        # Generate a LTL specification governing BDI control modes
        # FIX: infer control modes of interest from input arguments and actions

        modes_of_interest = []
        ts_spec = TransitionSystemSpecification(
                                    ts=control_mode_ts,
                                    props_of_interest=modes_of_interest,
                                    outcomes=action_outcomes)

        # Generate LTL specification governing action and preconditions
        action_spec = ActionSpecification(preconditions=simple_preconditions)
        for goal in goals:
            if goal not in ts_spec.ts.keys():  # topology is handled above
                action_spec.handle_new_action(action=goal,
                                              act_out=True,
                                              outcomes=action_outcomes)

        # Generate LTL specification governing the achievement of goals ...
        goal_spec = GoalSpecification()
        goal_spec.handle_single_liveness(goals=goals,
                                         outcomes=sm_outcomes,
                                         strict_order=True)

        # All the things that can fail:
        failure_conditions = ts_spec.ts.keys() + action_spec.all_actions

        # This is causing the error, without this the synthesis is unrealizable
        # in the current configuration
        assert len(failure_conditions) == len(set(failure_conditions))
        if SM_OUTCOME_FAILURE in sm_outcomes:
            # Add LTL formula tying all the things that can fail to SM outcome
            goal_spec.handle_any_failure(conditions=failure_conditions,
                                         failure=SM_OUTCOME_FAILURE)
        else:
            # If anything fails, retry (re-activate) until it succeeds
            goal_spec.handle_retry_after_failure(failures=failure_conditions)

        # Merge these specifications. Initial conditions are still missing.
        print('simple specification line 77', flush=True)
        self.merge_gr1_specifications([ts_spec, action_spec, goal_spec])

        # Now generate LTL formulas encoding all of the initial conditions
        ic_spec = InitialConditionsSpecification()
        ic_spec.set_ics_from_spec(spec=self,
                                  true_props=initial_conditions)

        # Finally, also merge the initial conditions specification
        self.merge_gr1_specifications([ic_spec])

    def _check_input_arguments(self, initial_conditions, goals,
                               action_outcomes, sm_outcomes):

        # if len(action_outcomes) > len(sm_outcomes):
        #     raise NotImplementedError('The specification cannot handle ' \
        #                               'more action outcomes {0} ' \
        #                               'than State Machine outcomes {1}'
        #                               .format(action_outcomes, sm_outcomes))

        if any([out not in ALL_SM_OUTCOMES for out in sm_outcomes]):
            raise NotImplementedError('Some SM outcomes: {0} are unknown. '
                                      'Expected them to be subseteq of {1}'
                                      .format(sm_outcomes, ALL_SM_OUTCOMES))

# =========================================================
# Entry point
# =========================================================


def main():  # pragma: no cover

    import pprint

    specification = CompleteSpecification(name='simple_example',
                                          initial_conditions=['prep_pay'],
                                          goals=['soda'])

    print('[INPUT]')
    pprint.pprint(specification.env_props)
    print('[OUTPUT]')
    pprint.pprint(specification.sys_props)
    print('SYS_INIT]')
    pprint.pprint(specification.sys_init)
    print('[ENV_INIT]')
    pprint.pprint(specification.env_init)
    print('[SYS_TRANS]')
    pprint.pprint(specification.sys_trans)
    print('[ENV_TRANS]')
    pprint.pprint(specification.env_trans)
    print('[SYS_LIVENESS]')
    pprint.pprint(specification.sys_liveness)
    print('[ENV_LIVENESS]')
    pprint.pprint(specification.env_liveness)


if __name__ == '__main__':  # pragma: no cover
    main()
