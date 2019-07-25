#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt
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

    current = []
    for point in data:
        current.append(point[4])

    f, axarr = plt.subplots(2, sharex=True)
    
    
    y = np.array(current)
    axarr[0].plot(y)

    fft_y = np.fft.fft(y)
    axarr[1].plot(fft_y)

    plt.show()
