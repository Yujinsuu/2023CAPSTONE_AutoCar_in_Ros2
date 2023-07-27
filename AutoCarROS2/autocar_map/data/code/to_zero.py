#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np
from pyproj import CRS
import pandas as pd
import geopandas as gpd
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

file_path = 'global.csv'
df=pd.read_csv(file_path)

x= df['X-axis'].tolist()
wx = list(i-962897.516413939 for i in x)
y = df['Y-axis'].tolist()
wy = list(j-1958728.3104721 for j in y)

link_ = df['Link'].tolist()
link = list(k for k in link_)

df = pd.DataFrame({'X-axis':wx,'Y-axis':wy,'Link':link})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')

