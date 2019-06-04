#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import sys
import pickle

if __name__ == "__main__" :
    if len(sys.argv) != 2:
        for item in sys.argv:
            print(str(item))
        exit()
    filename = str(sys.argv[1])
    print (filename)
    with open(filename, 'rb') as file:
        data = pickle.load(file)

    f, axarr = plt.subplots(3, sharex=True)

    axarr[0].plot(range(len(data)), [e[0] for e in data], color='r', label='ia')
    axarr[0].plot(range(len(data)), [e[1] for e in data], color='g', label='ib')
    axarr[0].plot(range(len(data)), [e[2] for e in data], color='b', label='ic')
    axarr[0].plot(range(len(data)), [sum(e[:3]) for e in data], color='gray', label='sum')
    axarr[0].legend(loc='best')
    #axarr[0].set_ylim(-2, 6)

    axarr[1].plot(range(len(data)), [e[3] for e in data], color='r', label='dc_a')
    axarr[1].plot(range(len(data)), [e[4] for e in data], color='g', label='dc_b')
    axarr[1].plot(range(len(data)), [e[5] for e in data], color='b', label='dc_c')
    #axarr[1].plot(range(len(data)), [e[7] for e in data], color='b', label='angle')
    #axarr[1].plot(range(len(data)), [e[8] for e in data], color='r', label='velocity')
    axarr[1].legend(loc='best')
    #axarr[1].set_ylim(0, 48)

    axarr[2].plot(range(len(data)), [e[9] for e in data], color='r', label='iq')
    axarr[2].plot(range(len(data)), [e[10] for e in data], color='g', label='id')
    axarr[2].legend(loc='best')

    plt.autoscale(axis='y')
    plt.show()
