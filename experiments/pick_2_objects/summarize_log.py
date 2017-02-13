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
rate_recog = 100. * df_recog.all(axis=1).sum() / len(df_recog)
print(' recog success rate: {:.1f} %'.format(rate_recog))

remove = ['remove_non_target']
df_remove = df[remove].dropna()
rate_remove = 100. * df_remove.values.sum() / len(df_remove)
df_remove_acm = df[recog + remove]
rate_remove_acm = 100. * df_remove_acm.all(axis=1).sum() / len(df_remove_acm)
print('remove success rate: {:.1f} % ({:.1f} %)'.format(rate_remove, rate_remove_acm))

pick = ['grasp', 'carry']
df_pick = df[pick].dropna(how='all')
rate_pick = 100. * df_pick.all(axis=1).sum() / len(df_pick)
df_pick_acm = df[recog + remove + pick]
rate_pick_acm = 100. * df_pick_acm.all(axis=1).sum() / len(df_pick_acm)
print('  pick success rate: {:.1f} % ({:.1f} %)'.format(rate_pick, rate_pick_acm))
