
import math
import numpy as np
from sympy import *


def gen_solinas_matrix(i, j):
    gcd = math.gcd(i, j)
    d = i // gcd

    #print('degree:', d)

    c = (d + 1) * [0]
    c[d] = -1
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

    #print('solinas matrix:')
    print(np.matrix(m))

    num_pos = d * [0]
    num_neg = d * [0]

    for j in range(0, d):
        for i in range(0, d):
            if m[i][j] > 0:
                num_pos[j] += 1
            elif m[i][j] < 0:
                num_neg[j] += 1

    #print('pos array: ', num_pos, 'max mod add:', max(num_pos))
    #print('neg array: ', num_neg, 'max mod sub:', max(num_neg))

    return max(num_pos) + max(num_neg)

gen_solinas_matrix(64, 24)


#for i in range(1, 128):
#    for j in range(1, i):
#        num = 2**i - 2**j + 1
#
#        if isprime(num):
#            print(num, ', i:', i, ', j:', j, ', bit length:',
#                  num.bit_length(), 'total weights:', gen_solinas_matrix(i, j))
