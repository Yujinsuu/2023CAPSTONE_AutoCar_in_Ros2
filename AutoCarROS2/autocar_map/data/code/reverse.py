#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from pyproj import CRS
import pandas as pd
import geopandas as gpd
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

file_path = 'h_ccw.csv'
df=pd.read_csv(file_path)

x= df['X-axis'].tolist()
wx = list(i-962897.516413939 for i in x)
wx.reverse()
y = df['Y-axis'].tolist()
wy = list(j-1958728.3104721 for j in y)
wy.reverse()

# Cubic Spline 보간
# cs_x = CubicSpline(range(len(wx)), wx)
# cs_y = CubicSpline(range(len(wy)), wy)
# dx = cs_x(np.arange(0, len(wx) - 1, 0.05))
# dy = cs_y(np.arange(0, len(wy) - 1, 0.05))
# yaw = np.arctan2(dy[1:] - dy[:-1], dx[1:] - dx[:-1])

# plt.figure(1)
# plt.plot(np.rad2deg(yaw))
# plt.show()

# df = pd.DataFrame({'X-axis':dx[:-1],'Y-axis':dy[:-1],'Yaw':yaw})
df = pd.DataFrame({'X-axis':wx,'Y-axis':wy})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')

