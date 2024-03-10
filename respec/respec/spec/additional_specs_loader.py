#!/usr/bin/env python
import yaml
import pprint
from respec.spec.gr1_specification import GR1Specification


class SpecsLoader(GR1Specification):
    """
    Loads all provided specs found in the yaml path provided.
    """

    def __init__(self, filePath):
        if filePath.endswith('.yaml'):
            name = filePath[filePath.rfind('/')+1:filePath.rfind('.yaml')]
            super(SpecsLoader, self).__init__(spec_name=name,
                                              env_props=[],
                                              sys_props=[])
            self.filePath = filePath
            self.sys_props = []
            self.env_props = []
            self.sys_init = []
            self.env_init = []
            self.sys_trans = []
            self.env_trans = []
            self.sys_liveness = []
            self.env_liveness = []
            self._extract()
        else:
            raise TypeError("Invalid filePath provided. Needs to be a '.yaml' file")

    def _extract(self):
        with open(self.filePath, 'r') as file:
            specs = yaml.safe_load(file)['specs']
        if 'INPUT' in specs:
            self.env_props.extend(specs['INPUT'])
        if 'OUTPUT' in specs:
            self.sys_props.extend(specs['OUTPUT'])
        if 'SYS_INIT' in specs:
            self.sys_init.extend(specs['SYS_INIT'])
        if 'ENV_INIT' in specs:
            self.env_init.extend(specs['ENV_INIT'])
        if 'SYS_TRANS' in specs:
            self.sys_trans.extend(specs['SYS_TRANS'])
        if 'ENV_TRANS' in specs:
            self.env_trans.extend(specs['ENV_TRANS'])
        if 'SYS_LIVENESS' in specs:
            self.sys_liveness.extend(specs['SYS_LIVENESS'])
        if 'ENV_LIVENESS' in specs:
            self.env_liveness.extend(specs['ENV_LIVENESS'])


def main():  # DEMO purposes
    fpath = '/home/neec23/chrislab/install/respec/share/respec/additional_specs/wgcf.yaml'
    specification = SpecsLoader(fpath)
    print('[INPUT]')
    pprint.pprint(specification.env_props)
    print('[OUTPUT]')
    pprint.pprint(specification.sys_props)
    print('[SYS_INIT]')
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
