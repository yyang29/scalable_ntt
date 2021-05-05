import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sympy import *
import seaborn as sns


def gen_solinas_matrix(i, j, sign=1):
    # format: one line for each operand
    # start_idx, end_idx, offset, start_idx, end_idx, offset, ..., operand width, sign
    ret = []
    if j == 1 and sign == 1:
        ret.append([0, i-1, 0, i, 1])
        ret.append([i, 2*i-1, 0, i, 1])
        print(np.matrix(ret))
        return ret

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

    added = 0
    while added < max(num_pos):
        for i in range(0, d):
            vector = []
            start_idx = (i, 0)
            interval = [-1, -1]
            offset = 0
            for j in range(0, d):
                row = start_idx[0] + j
                row = row if row < d else row - d
                col = start_idx[1] + j
                col = col if col < d else col - d

                if m[row][col] > 0 and interval[0] == -1:
                    interval[0] = row * gcd
                    interval[1] = (row + 1) * gcd - 1
                    offset = col * gcd
                elif m[row][col] > 0 and interval[1] == row * gcd - 1:
                    interval[1] = (row + 1) * gcd - 1
                elif m[row][col] > 0:
                    vector.append(interval[0])
                    vector.append(interval[1])
                    vector.append(offset)
                    interval[0] = row * gcd
                    interval[1] = (row + 1) * gcd - 1
                    offset = col * gcd
                elif m[row][col] == 0 and interval[0] != -1:
                    vector.append(interval[0])
                    vector.append(interval[1])
                    vector.append(offset)
                    interval = [-1, -1]
                    offset = 0

                if m[row][col] > 0:
                    m[row][col] -= 1

            if interval[0] != -1:
                vector.append(interval[0])
                vector.append(interval[1])
                vector.append(offset)
                interval = [-1, -1]
                offset = 0

            if vector:
                vector.append(1)
                ret.append(vector)
                added += 1

    added = 0
    while added < max(num_neg):
        for i in range(0, d):
            vector = []
            start_idx = (i, 0)
            interval = [-1, -1]
            offset = 0
            for j in range(0, d):
                row = start_idx[0] + j
                row = row if row < d else row - d
                col = start_idx[1] + j
                col = col if col < d else col - d

                if m[row][col] < 0 and interval[0] == -1:
                    interval[0] = row * gcd
                    interval[1] = (row + 1) * gcd - 1
                    offset = col * gcd
                elif m[row][col] < 0 and interval[1] == row * gcd - 1:
                    interval[1] = (row + 1) * gcd - 1
                elif m[row][col] < 0:
                    vector.append(interval[0])
                    vector.append(interval[1])
                    vector.append(offset)
                    interval[0] = row * gcd
                    interval[1] = (row + 1) * gcd - 1
                    offset = col * gcd
                elif m[row][col] == 0 and interval[0] != -1:
                    vector.append(interval[0])
                    vector.append(interval[1])
                    vector.append(offset)
                    interval = [-1, -1]
                    offset = 0

                if m[row][col] < 0:
                    m[row][col] += 1

            if interval[0] != -1:
                vector.append(interval[0])
                vector.append(interval[1])
                vector.append(offset)
                interval = [-1, -1]
                offset = 0

            if vector:
                vector.append(-1)
                ret.append(vector)
                added += 1

    print(ret)
    return ret


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
                        ret = gen_solinas_matrix(i, j, sign)
                        adder_inputs = len(ret)
                        if not (j == 1 and sign == 1):
                            adder_inputs += 1
                        print(num, i, j, sign, 'bit length:',
                              num.bit_length(), 'total weights:', adder_inputs)
                        csv_writer.writerow(
                            [num, i, j, sign, num.bit_length(), adder_inputs])

    data_df = pd.read_csv('prime.csv')

    df1 = data_df.groupby(['bit width', 'mod add/sub'],
                          as_index=False)['prime'].count()

    fig, ax = plt.subplots(figsize=(6, 4))
    ax = sns.scatterplot(x='bit width', y='mod add/sub', hue='prime', size='prime',
                         sizes=(40, 300), data=df1, palette=sns.color_palette(palette='ch:2,r=.2,l=.6', n_colors=7, as_cmap=True))
    plt.ylim(0, 10.5)
    ax.set_xlabel('bit width', fontsize=14)
    ax.set_ylabel('adder tree num of operands', fontsize=14)
    ax.tick_params(axis='both', which='major', labelsize=14)

    ax.legend(title='num of\nprimes', bbox_to_anchor=(
        1.0, 0.8), facecolor='white', edgecolor='white', markerscale=0.9)

    fig.tight_layout(pad=0.05)
    fig.savefig('output.eps')


plot()
