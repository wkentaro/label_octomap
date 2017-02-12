#!/usr/bin/env python

from collections import OrderedDict
import time

import numpy as np
import yaml


OBJECTS = [
    2,
    4,
    6,
    7,
    10,
    14,
    22,
    24,
    27,
    28,
    34,
    35,
    36,
    38,
    39,
]


np.random.seed(int(time.time()))
chosen = np.random.choice(OBJECTS, 2).tolist()
target_id = int(chosen[-1])

import yaml

from collections import OrderedDict

def represent_ordereddict(dumper, data):
    value = []
    for item_key, item_value in data.items():
        node_key = dumper.represent_data(item_key)
        node_value = dumper.represent_data(item_value)
        value.append((node_key, node_value))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', value)

yaml.add_representer(OrderedDict, represent_ordereddict)
data = [OrderedDict([
    ('target_id', target_id),
    ('all_id', chosen),
    ('find_target', None),
    ('find_non_target', None),
    ('remove_non_target', None),
    ('grasp', None),
    ('carry', None),
])]
print(yaml.dump(data, default_flow_style=False))