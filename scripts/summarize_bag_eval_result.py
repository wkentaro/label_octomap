#!/usr/bin/env python

from __future__ import print_function
import argparse
import os.path as osp

import numpy as np
import pandas as pd

import rosbag


this_dir = osp.dirname(__file__)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file')
    args = parser.parse_args()

    bag_file = args.bag_file

    bag = rosbag.Bag(bag_file)
    data = []
    for msg in bag.read_messages():
        try:
            timestamp = msg.header.stamp
        except:
            timestamp = msg.timestamp
        try:
            value = msg.message.accuracy
        except:
            value = '<EMPTY>'
        data.append({
            'timestamp': timestamp,
            msg.topic: value,
        })

    df = pd.DataFrame(data, dtype=object)
    df = df.set_index('timestamp')

    # print(df.columns)

    n_moved = 0
    accuracies = ['accl_v', 'accl_b', 'accr_v', 'accr_b']
    # single_registration, view1-3, box/voxel
    sr_b = {'view1': [], 'view2': [], 'view3': []}
    sr_v = {'view1': [], 'view2': [], 'view3': []}
    # label_octomap
    lo_b = {'view1': [], 'view2': [], 'view3': []}
    lo_v = {'view1': [], 'view2': [], 'view3': []}
    for index, row in df.iterrows():
        start = row['/look_around_bin_main/output/start']
        stop = row['/look_around_bin_main/output/stop']
        accl_v = row['/label_octomap/evaluate_voxel_seg_by_gt/output']
        accl_b = row['/label_octomap/evaluate_box_seg_by_gt/output']
        accr_v = row['/single_registration/evaluate_voxel_seg_by_gt/output']
        accr_b = row['/single_registration/evaluate_box_seg_by_gt/output']
        if pd.notnull(stop):
            n_moved += 1
        if n_moved not in (2, 3, 4):
            continue
        view_name = 'view%d' % (n_moved - 1)
        if pd.notnull(accr_b):
            sr_b[view_name].append(accr_b)
        if pd.notnull(accr_v):
            sr_v[view_name].append(accr_v)
        if pd.notnull(accl_b):
            lo_b[view_name].append(accl_b)
        if pd.notnull(accl_v):
            lo_v[view_name].append(accl_v)

    sr_b = {k: np.max(v) for k, v in sr_b.items()}
    sr_v = {k: np.max(v) for k, v in sr_v.items()}
    lo_b = {k: v[-1] for k, v in lo_b.items()}
    lo_v = {k: v[-1] for k, v in lo_v.items()}
    # print('sr_b_view1: ', sr_b['view1'])
    # print('sr_b_view2: ', sr_b['view2'])
    # print('sr_b_view3: ', sr_b['view3'])
    # print('lo_b:       ', lo_b['view3'])
    print('sr_v_view1: ', sr_v['view1'])
    print('sr_v_view2: ', sr_v['view2'])
    print('sr_v_view3: ', sr_v['view3'])
    print('lo_v:       ', lo_v['view3'])

    label_value = osp.splitext(osp.basename(bag_file))[0]
    out_file = osp.join(this_dir, '../eval_result/%s.yaml' % label_value)
    with open(out_file, 'w') as f:
        f.write('label_octomap: %f\n' % lo_v['view3'])
        f.write('viewpoint_0: %f\n' % sr_v['view1'])
        f.write('viewpoint_1: %f\n' % sr_v['view2'])
        f.write('viewpoint_2: %f\n' % sr_v['view3'])


if __name__ == '__main__':
    main()
