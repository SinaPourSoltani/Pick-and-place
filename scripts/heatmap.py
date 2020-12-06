import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import seaborn as sb
import numpy as np
from functools import reduce

objects = ['red', 'green', 'blue']

def dataframe_from_text_file(*objects):
    dfs = []
    for obj in objects:
        df = pd.read_table(os.path.join(os.pardir, 'Project_WorkCell', 'collisions_heatmap', f'ra_{obj}.txt'),
                           skiprows=2,
                           header=0,
                           sep=' ',
                           )
        dfp = df.pivot('x', 'y', '#collisionFreeSolutions')
        dfs.append(dfp)
    return dfs

def plot_dataframe(df, color):
    ax = sb.heatmap(df, cmap=color)
    plt.show()


dfr, dfg, dfb = dataframe_from_text_file(*objects)
dfs = [dfr, dfg, dfb]

dfc = dfr.add(dfg.add(dfb))
print(dfc)



plot_dataframe(dfr,'Reds')
plot_dataframe(dfg,'Greens')
plot_dataframe(dfb,'Blues')
plot_dataframe(dfc,"hot")

