#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from pyproj import CRS
import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt

file_path = 'global.csv'
df=pd.read_csv(file_path)

x = df['X-axis'][df['Link']==3].tolist()
y = df['Y-axis'][df['Link']==3].tolist()

ax = -3.5
ay = 1.04

x0 = [i+3*ax for i in x]
y0 = [i+3*ay for i in y]

wx=[]
wy=[]
wz=[]

for i in range(21):
    for j in range(len(x)):
        wx.append(x[j]+i*ax)
        wy.append(y[j]+i*ay)
        wz.append(i)


df = pd.DataFrame({'X-axis':wx,'Y-axis':wy, 'Link':wz})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')
