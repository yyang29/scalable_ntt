import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sympy import *
import seaborn as sns


def gen_solinas_matrix(i, j, sign=1):
    if j == 1 and sign == -1:
        return 2

    gcd = math.gcd(i, j)
    d = i // gcd

    #print('degree:', d)

    c = (d + 1) * [0]
    c[d] = -1 * sign
    #c[j // gcd + 1] = 1
    c[d - j // gcd] = 1

    #print('c array: ', c)

    m = []
    for i in range(0, d):
        m.append([])
        for j in range(0, d):
            m[i].append(0)

    for j in range(0, d):
        m[0][j] = c[d - j]

    for i in range(1, d):
        for j in range(0, d):
            if j == 0:
                m[i][j] = m[i-1][d-1] * c[d]
            else:
                m[i][j] = m[i-1][j-1] + m[i-1][d-1] * c[d-j]

    print('solinas matrix:')
    print(np.matrix(m))
    print(gcd)

    num_pos = d * [0]
    num_neg = d * [0]

    for j in range(0, d):
        for i in range(0, d):
            if m[i][j] > 0:
                num_pos[j] += m[i][j]
            elif m[i][j] < 0:
                num_neg[j] -= m[i][j]

    print('pos array: ', num_pos, 'max mod add:', max(num_pos))
    print('neg array: ', num_neg, 'max mod sub:', max(num_neg))

    return max(num_pos) + max(num_neg) + 1


gen_solinas_matrix(13, 1, 1)


def plot():

    sns.set_theme(style='darkgrid')

    visited = set()

    with open('prime.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',')
        csv_writer.writerow(
            ['prime', 'i', 'j', 'sign', 'bit width', 'mod add/sub'])
        for i in range(12, 65):
            for j in range(1, i):
                for sign in [-1, 1]:
                    num = 2**i - 2**j + sign * 1
                    if isprime(num) and num not in visited:
                        visited.add(num)
                        print(num, i, j, sign, 'bit length:',
                              num.bit_length(), 'total weights:', gen_solinas_matrix(i, j, sign))
                        csv_writer.writerow(
                            [num, i, j, sign, num.bit_length(), gen_solinas_matrix(i, j, sign)])

    data_df = pd.read_csv('prime.csv')

    df1 = data_df.groupby(['bit width', 'mod add/sub'],
                          as_index=False)['prime'].count()

    fig, ax = plt.subplots(figsize=(6, 4))
    ax = sns.scatterplot(x='bit width', y='mod add/sub', hue='prime', size='prime',
                         sizes=(40, 300), data=df1, palette=sns.color_palette(palette='ch:2,r=.2,l=.6', n_colors=7, as_cmap=True))
    plt.ylim(0, 10.5)
    ax.set_xlabel('bit width', fontsize=12)
    ax.set_ylabel('adder tree inputs', fontsize=12)
    ax.tick_params(axis='both', which='major', labelsize=12)

    ax.legend(title='num of\nprimes', bbox_to_anchor=(
        1.0, 0.8), facecolor='white', edgecolor='white', markerscale=0.9)

    fig.tight_layout(pad=0.05)
    fig.savefig('output.eps')


plot()
