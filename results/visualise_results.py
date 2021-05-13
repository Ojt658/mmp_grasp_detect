"""
This script utilises matplotlib and wordcloud to produce visualisations of the results to aid in analysis.
When using PyCharm the plots are generated within the IDE and can be saved from there. They are shown in the results folder.
Author: olt13
"""

import pandas as pd
import matplotlib.pyplot as plt
from wordcloud import WordCloud


def load():
    return pd.read_csv('results.csv')  ## GT or model results


def word_cloud(data):
    wc = WordCloud(max_font_size=50, background_color="white").generate_from_frequencies(frequencies=data)
    plt.imshow(wc, interpolation='bilinear')
    plt.axis("off")
    plt.show()


def bot_15(data):
    bot = sorted(data, key=lambda x: data[x])[:15]
    vals = []
    for n in bot:
        vals.append(data[n])

    colours = ['blue', 'red', 'purple', 'green', 'orange']
    plt.bar(bot, vals, color=colours)
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    plt.gca().set_ylim([0.0, 1.0])
    plt.show()


def top_15(data):
    top = sorted(data, key=lambda x: data[x], reverse=True)[:15]
    vals = []
    for n in top:
        vals.append(data[n])

    colours = ['blue', 'red', 'purple', 'green', 'orange']
    plt.bar(top, vals, color=colours)
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    plt.gca().set_ylim([0.0, 1.0])
    plt.show()


def main():
    data = load()
    df_dict = data[['Type of Object', 'Average']].to_dict('list')
    words_freq = {}
    for index, w in enumerate(df_dict['Type of Object']):
        words_freq[w] = df_dict['Average'][index]

    word_cloud(words_freq)
    top_15(words_freq)
    bot_15(words_freq)


if __name__ == '__main__':
    main()
