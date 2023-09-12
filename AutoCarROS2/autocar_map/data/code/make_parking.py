#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from pyproj import CRS
import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt

file_path = 'revpark.csv'
df=pd.read_csv(file_path)

# offset = [-0.4423572639935003, -0.8308233860879995] # kcity
offset = [-0.256388029315985, -0.904206226115093] # htech

x0 = df['X-axis'][df['Link']==0].tolist()
y0 = df['Y-axis'][df['Link']==0].tolist()

x = [i + 0.256388029315985 * 0.5 for i in x0]
y = [i + 0.904206226115093 * 0.5 for i in y0]

num = len(x)
d = 5.2

wx=[]
wy=[]
wz=[]

for i in range(3):
    for j in range(num):
        wx.append(x[j] + i * d * offset[0])
        wy.append(y[j] + i * d * offset[1])
        wz.append(i)


df = pd.DataFrame({'X-axis':wx,'Y-axis':wy, 'Link':wz})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')
