#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
import pandas as pd

file_path = 'track_lane.csv'
df=pd.read_csv(file_path)

x= df['x'].tolist()
y = df['y'].tolist()
yaw = df['yaw'].tolist()

wx,wy,wz = [],[],[]

num = len(x)
dist = 3 # 6m 

for i in range(int(num/dist)):
    wx.append(x[dist*i])
    wy.append(y[dist*i])
    wz.append(yaw[dist*i])

df = pd.DataFrame({'x':wx,'y':wy, 'yaw':wz})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')

