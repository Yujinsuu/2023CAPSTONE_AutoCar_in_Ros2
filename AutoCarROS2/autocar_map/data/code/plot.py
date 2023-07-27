#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt

plt.figure(1)

df = pd.read_csv('output.csv')

x = df['X-axis'].values.tolist()
y = df['Y-axis'].values.tolist()


plt.plot(x,y,'o-',linewidth=0.5)

plt.show()
