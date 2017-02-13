#!/usr/bin/env python

from collections import OrderedDict
from collections import Counter
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


data = yaml.load(open('log.yaml'))
all_ids = []
for datum in data:
    all_ids += datum['all_ids']
count = Counter(all_ids)

for obj_id in OBJECTS:
    if obj_id not in count:
        count[obj_id] = 0
    # print('%d: %d' % (obj_id, count[obj_id]))


min_count = sorted(count.items(), key=lambda x: x[1])[1][1]
candidates = []
for obj_id, cnt in count.items():
    if cnt <= min_count:
        candidates.append(obj_id)
chosen = np.random.choice(candidates, 2, replace=False).tolist()
target_id = chosen[-1]


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
    ('all_ids', chosen),
    ('find_target', None),
    ('find_non_target', None),
    ('remove_non_target', None),
    ('grasp', None),
    ('carry', None),
])]
print(yaml.dump(data, default_flow_style=False))