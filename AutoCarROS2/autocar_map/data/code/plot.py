#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt

plt.figure(1)

df = pd.read_csv('track.csv')

x = df[df['Link']==1]['X-axis'].values.tolist()
y = df[df['Link']==1]['Y-axis'].values.tolist()

d = 25
r = 100
plt.plot(x[d:d+r],y[d:d+r],'o-',linewidth=0.5)
plt.plot(x[d],y[d],'ro',linewidth=0.5)
for i in range(d,d+r):
    plt.text(x[i], y[i], str(i), ha='center', va='bottom')
plt.show()
