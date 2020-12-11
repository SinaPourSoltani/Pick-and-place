import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import StrMethodFormatter
import seaborn as sb
import numpy as np
from functools import reduce

objects = ['red', 'green', 'blue']


def dataframe_from_text_file(*objects, place=False):
    dfs = []
    for obj in objects:
        char = 'p' if place else ''
        df = pd.read_table(os.path.join(os.pardir, 'Project_WorkCell', 'collisions_heatmap', f'ns_ra{char}_{obj}.txt'),
                           skiprows=2,
                           header=0,
                           sep=' ',
                           )
        dfp = df.pivot('x', 'y', '#collisionFreeSolutions')
        dfs.append(dfp)
    return dfs


def plot_dataframe(df, color, title='Pick', filepath=""):
    ax = sb.heatmap(df, cmap=color, cbar_kws={'label': '# of collision-free joint configurations'})

    def fmt(s):
        try:
            n = "{:.2f}".format(float(s))
        except:
            n = ""
        return n

    ax.set_xticklabels([fmt(label.get_text()) for label in ax.get_xticklabels()])
    ax.set_yticklabels([fmt(label.get_text()) for label in ax.get_yticklabels()])

    f = plt.gcf()
    plt.title(title)
    plt.show()
    if filepath != "":
        f.savefig(f'{filepath}.pdf', bbox_inches='tight')


def normalize_dataframe(df):
    min_value = np.min(df.min())
    max_value = np.max(df.max())
    normalized_df = (df - min_value) / (max_value - min_value)
    return normalized_df


dfr, dfg, dfb = dataframe_from_text_file(*objects)
dfrp, dfgp, dfbp = dataframe_from_text_file(*objects, place=True)

dfc = dfr.add(dfg.add(dfb))
dfcp = dfrp.add(dfgp.add(dfbp))

plot_dataframe(dfr,'Reds',filepath='red_pick')
plot_dataframe(dfg,'Greens',filepath='green_pick')
plot_dataframe(dfb,'Blues',filepath='blue_pick')

plot_dataframe(dfrp,'Reds', 'Place',filepath='red_place')
plot_dataframe(dfgp,'Greens', 'Place',filepath='green_place')
plot_dataframe(dfbp,'Blues', 'Place',filepath='blue_place')

normalized_dfr = normalize_dataframe(dfr)
normalized_dfg = normalize_dataframe(dfg)
normalized_dfb = normalize_dataframe(dfb)

normalized_dfrp = normalize_dataframe(dfrp)
normalized_dfgp = normalize_dataframe(dfgp)
normalized_dfbp = normalize_dataframe(dfbp)

dfcn = normalized_dfr.add(normalized_dfg.add(normalized_dfb))
dfcpn = normalized_dfrp.add(normalized_dfgp.add(normalized_dfbp))

normalized_dfcn = normalize_dataframe(dfcn)
normalized_dfcpn = normalize_dataframe(dfcpn)

plot_dataframe(normalized_dfcn,"coolwarm", 'Normalised CoM Pick',filepath='com_rgb_pick_normalized')
plot_dataframe(normalized_dfcpn,"coolwarm", 'Normalised CoM Place',filepath='com_rgb_place_normalized')

com_norm_dfcs = normalized_dfcn.add(normalized_dfcpn)
plot_dataframe(normalize_dataframe(com_norm_dfcs), "afmhot", 'Normalised CoM Pick & Place',filepath='com_rgb_pnp_normalized')