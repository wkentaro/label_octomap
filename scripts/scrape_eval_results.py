#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import glob
import os.path as osp
import yaml

import pandas


this_dir = osp.dirname(osp.realpath(__file__))

res_files = glob.glob(osp.join(this_dir, '../eval_result/*.yaml'))

data = []
for fn in res_files:
    label_value = int(osp.splitext(osp.basename(fn))[0])
    with open(fn, 'r') as f:
        datum = yaml.load(f)
    data.append({
        'label_value': label_value,
        'single_res_voxel_max': 100 * datum['sr_v_max'],
        'single_res_voxel_mean': 100 * datum['sr_v_mean'],
        'single_res_box_max': 100 * datum['sr_b_max'],
        'single_res_box_mean': 100 * datum['sr_b_mean'],
        'label_oct_voxel': 100 * datum['lo_v'],
        'label_oct_box': 100 * datum['lo_b'],
        'img': 'img_%d.jpg' % label_value,
        'seg': 'seg_%d.jpg' % label_value,
    })
columns = ['label_value', 'single_res_box_max', 'single_res_box_mean',
           'single_res_box_max', 'single_res_box_mean',
           'label_oct_voxel', 'label_oct_box',
           'img', 'seg']
df = pandas.DataFrame(data=data, columns=columns)
df = df.sort_values('label_value')
df = df.set_index('label_value')
df = df.round(decimals=2)

print(df.to_latex())
