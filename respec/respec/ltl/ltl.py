#!/usr/bin/env python

"""
Boolean operators and the 'next' LTL operator.

The remaining LTL operators are not (currently) needed because
the specification module is using the .structuredslugs format.
"""


def conj(terms):
    # print('**line 12 in ltl.py')
    terms_list = list(terms)
    # print('list terms 13 in ltl.py ', list(terms))
    # print('list terms 14 in ltl.py ', list(terms))  # calling list(terms) twice gives [] !
    # print(' list terms 16 in ltl.py ', terms_list, flush=True)
    try:
        if len(terms_list) > 1:
            # print(f'   conj with {len(terms_list)} ...')
            return paren(' & '.join(terms_list))
        else:
            if len(terms_list) > 0:
                # print('list terms 19 ltl.py ', terms_list)
                # print('line 20 ltl.py prefail')
                return terms_list[0]
            else:
                # print(f' returning None with no terms in conj')
                # print('list terms 25 ltl.py ', terms_list)
                return None
    except Exception as exc:
        print('line 22  in ltl.py', exc, flush=True)
        raise exc


def disj(terms):
    terms_list = list(terms)
    if len(terms_list) > 1:
        return paren(' | '.join(terms_list))
    else:
        return terms_list[0]


def neg(term):
    return '! ' + term


def next(term):
    # print(type(term))
    # print('line 41 ltl.py"{}"'.format(term))
    if term[0] == '(' and term[-1] == ')':
        return 'next' + term
    else:
        return 'next' + paren(term)


def implication(left_hand_side, right_hand_side):
    return left_hand_side + ' -> ' + right_hand_side


def iff(left_hand_side, right_hand_side):
    return left_hand_side + ' <-> ' + right_hand_side


def paren(term):
    return '(' + term + ')'
