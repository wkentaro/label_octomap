#!/usr/bin/env python

from __future__ import print_function

import glob
import os.path as osp
import yaml
import numpy as np


this_dir = osp.dirname(osp.realpath(__file__))

single_view_iu = []
label_octomap_iu = []
max_single_view_iu = []
label_octomap_exceeds_max = []
label_octomap_exceeds_mean = []
for result_file in glob.glob(osp.join(this_dir, '../eval_result/*.yaml')):
    label = int(osp.splitext(osp.basename(result_file))[0])
    result = yaml.load(open(result_file))
    iu_view0 = result['viewpoint_0']
    iu_view1 = result['viewpoint_1']
    iu_view2 = result['viewpoint_2']
    three_view_iu = [iu_view0, iu_view1, iu_view2]
    single_view_iu.append(iu_view0)
    single_view_iu.append(iu_view1)
    single_view_iu.append(iu_view2)
    max_iu = max(three_view_iu)
    max_single_view_iu.append(max_iu)
    iu_octomap = result['label_octomap']
    label_octomap_iu.append(iu_octomap)
    label_octomap_exceeds_max.append(iu_octomap > max_iu)
    label_octomap_exceeds_mean.append(iu_octomap > np.mean(three_view_iu))
print('mean IU for single view: ', np.mean(single_view_iu))
print('mean of max IUs of three single views: ', np.mean(max_single_view_iu))
print('mean IU for LabelOctoMap: ', np.mean(label_octomap_iu))
print('ratio of LabelOctoMap exceeds max IU of three single views: ', np.mean(label_octomap_exceeds_max))
print('ratio of LabelOctoMap exceeds mean IU of three single views: ', np.mean(label_octomap_exceeds_mean))
