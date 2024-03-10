#!/usr/bin/env python

from ...respec.formula.gr1_formulas import GR1Formula, SimpleLivenessRequirementFormula

import unittest


class FormulaGenerationTests(unittest.TestCase):

    # ==========================================
    # Tests for generic GR(1) formulas
    # ==========================================

    def setUp(self):

        self.env_props = ['x1', 'x2']
        self.sys_props = ['y1', 'y2', 'y3']

    def tearDown(self):

        del self.env_props
        del self.sys_props

    def test_props_from_ts(self):

        ts = {'a1': ['a1', 'a2'],
              'a2': ['a2', 'a3'],
              'a3': ['a3', 'a2']}

        expected_props = self.sys_props + ts.keys()

        formula = GR1Formula(self.env_props, self.sys_props, ts)

        self.assertItemsEqual(formula.sys_props, expected_props)

    def test_mutex(self):
        '''Test whether mutual exclusion formulas are generated correctly.'''

        formula = GR1Formula(self.env_props, self.sys_props)

        mutex = formula.gen_mutex_formulas(self.sys_props, False)

        expected_mutex = ['y1 <-> (! y2 & ! y3)',
                          'y2 <-> (! y1 & ! y3)',
                          'y3 <-> (! y1 & ! y2)']

        self.assertItemsEqual(mutex, expected_mutex)

    def test_sys_trans(self):
        '''Test the adjacency relation that encodes the transition system.'''

        ts = {'y1': ['y1', 'y2'],
              'y2': ['y2', 'y3'],
              'y3': ['y3', 'y2']}

        formula = GR1Formula(self.env_props, self.sys_props, ts)

        adj_relation = formula.gen_sys_trans_formulas()

        expected_adj_relation = [
            'y1 -> (next(y1 & ! y3 & ! y2) | next(y2 & ! y1 & ! y3))',
            'y2 -> (next(y2 & ! y1 & ! y3) | next(y3 & ! y1 & ! y2))',
            'y3 -> (next(y3 & ! y1 & ! y2) | next(y2 & ! y1 & ! y3))'
        ]

        self.assertItemsEqual(adj_relation, expected_adj_relation)


class SystemLivenessTests(unittest.TestCase):

    # ==========================================
    # Tests for system liveness requirements
    # ==========================================

    def setUp(self):

        pass

    def tearDown(self):

        pass

    def test_system_liveness_formula_single_goal(self):

        formula = SimpleLivenessRequirementFormula(goals=['finished'])

        self.assertEqual('sys_liveness', formula.type)

        self.assertItemsEqual(['finished'], formula.formulas)

    def test_system_liveness_formula_conjunction(self):

        formula = SimpleLivenessRequirementFormula(goals=['finished', '! failed'])

        self.assertEqual('sys_liveness', formula.type)

        self.assertItemsEqual(['(finished & ! failed)'], formula.formulas)

    def test_system_liveness_formula_disjunction(self):

        formula = SimpleLivenessRequirementFormula(goals=['finished', 'failed'],
                                                   disjunction=True)

        self.assertEqual('sys_liveness', formula.type)

        self.assertItemsEqual(['(finished | failed)'], formula.formulas)

    def test_system_liveness_formula_conjunction_2(self):

        formula = SimpleLivenessRequirementFormula(goals=['happy', 'victorious'],
                                                   disjunction=False)

        self.assertEqual('sys_liveness', formula.type)

        self.assertItemsEqual(['(happy & victorious)'], formula.formulas)


if __name__ == '__main__':
    # Run all tests
    unittest.main()
