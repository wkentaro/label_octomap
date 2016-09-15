#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import glob
import os.path as osp
import yaml

import pandas


this_dir = osp.dirname(osp.realpath(__file__))

result_files = glob.glob(osp.join(this_dir, '../eval_result/*.yaml'))

columns = ['label', 'iu_view_0', 'iu_view_1', 'iu_view_2', 'iu_label_octomap']
data = []
filename_to_label = lambda x: int(osp.splitext(osp.basename(x))[0])
for result_file in sorted(result_files, key=filename_to_label):
    iu_data = yaml.load(open(result_file))
    label_value = filename_to_label(result_file)
    row = [
        label_value,
        iu_data['viewpoint_0'],
        iu_data['viewpoint_1'],
        iu_data['viewpoint_2'],
        iu_data['label_octomap'],
    ]
    data.append(row)
df = pandas.DataFrame(data=data, columns=columns)

df_single_view = df[['iu_view_0', 'iu_view_1', 'iu_view_2']]
mean_of_max_iu_single_view = df_single_view.max(axis=1).mean()
mean_iu_single_view = df_single_view.sum(axis=1).sum() / df_single_view.size

mean_iu_label_octomap = df['iu_label_octomap'].mean()

print('mean_iu_single_view:', mean_iu_single_view)
print('mean_of_max_iu_single_view:', mean_of_max_iu_single_view)
print('mean_iu_label_octomap:', mean_iu_label_octomap)
