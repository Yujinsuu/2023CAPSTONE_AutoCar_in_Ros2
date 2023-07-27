#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import os
os.environ['SHAPE_RESTORE_SHX'] = 'YES'

from pyproj import CRS
import pandas as pd
import geopandas as gpd

file_path = 'M.shp'

gdf = gpd.read_file(file_path)
gdf.crs = "EPSG:5181"
crs = CRS("EPSG:5179")
gdf = gdf.to_crs(crs)
coor_dict={}
for i in range(len(gdf)):
    ty=(gdf.type == 'Polygon')
    if ty[0]==1:
       data=gdf['geometry'][i].exterior.xy
    else:
       data=gdf['geometry'][i].xy
    coor_dict[i]={'x':data[0], 'y':data[1]}
coor_dict
wayp_dict={}
wx=[]
wy=[]
for i in range(len(coor_dict)):
	wx.append(coor_dict[i]['x'][0])
	wy.append(coor_dict[i]['y'][0])

df = pd.DataFrame({'X-axis':wx,'Y-axis':wy})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')
