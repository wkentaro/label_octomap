#!/usr/bin/env python

from __future__ import print_function
import argparse
import os.path as osp

import numpy as np
import pandas as pd
import yaml

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
    sr_b = []
    sr_v = []
    # label_octomap
    lo_b = []
    lo_v = []
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
        if pd.notnull(accr_b):
            sr_b.append(accr_b)
        if pd.notnull(accl_b):
            lo_b.append(accl_b)
        if pd.notnull(accr_v):
            sr_v.append(accr_v)
        if pd.notnull(accl_v):
            lo_v.append(accl_v)

    res = {
        'sr_b_max': float(np.max(sr_b)),
        'sr_b_mean': float(np.mean(sr_b)),
        'lo_b': float(lo_b[-1]),
        'sr_v_max': float(np.max(sr_v)),
        'sr_v_mean': float(np.mean(sr_v)),
        'lo_v': float(lo_v[-1]),
    }

    this_dir = osp.dirname(__file__)
    label_value = osp.splitext(osp.basename(bag_file))[0]
    out_file = osp.join(this_dir, '../eval_result', label_value + '.yaml')
    with open(out_file, 'w') as f:
        yaml.safe_dump(res, f, default_flow_style=False)


if __name__ == '__main__':
    main()
