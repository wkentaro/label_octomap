#!/usr/bin/env python

import yaml
import pandas as pd


data = yaml.load(open('log.yaml'))
df = pd.DataFrame(data)

task_states = [
    'find_target',
    'find_non_target',
    'remove_non_target',
    'grasp',
    'carry',
]
print('# Result in {} experiments'.format(len(df)))

recog = ['find_target', 'find_non_target']
df_recog = df[recog]
a = df_recog.all(axis=1).sum()
b = len(df_recog)
rate_recog = 1. * a / b
print(' recog success rate      : {0:.1f} % ({1}/{2})'.format(100 * rate_recog, a, b))

remove = ['remove_non_target']
df_remove = df[remove].dropna()
a = df_remove.values.sum()
b = len(df_remove)
rate_remove = 1. * a / b
print('remove success rate      : {0:.1f} % ({1}/{2})'.format(100 * rate_remove, a, b))
df_remove_total = df[recog + remove]
a = df_remove_total.all(axis=1).sum()
b = len(df_remove_total)
rate_remove_total = 1. * a / b
print('remove success rate total: {0:.1f} % ({1}/{2})'.format(100 * rate_remove_total, a, b))

pick = ['grasp', 'carry']
df_pick = df[pick].dropna(how='all')
a = df_pick.all(axis=1).sum()
b = len(df_pick)
rate_pick = 1. * a / b
print('  pick success rate      : {0:.1f} % ({1}/{2})'.format(100 * rate_pick, a, b))
df_pick_total = df[recog + remove + pick]
a = df_pick_total.all(axis=1).sum()
b = len(df_pick_total)
rate_pick_total = 1. * a / b
print('  pick success rate total: {0:.1f} % ({1}/{2})'.format(100 * rate_pick_total, a, b))
