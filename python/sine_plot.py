#!/usr/bin/env python3

"""Show the plot of a sine wave."""

import numpy as np
import matplotlib.pyplot as plt


def main():
    # Wave parameters
    frequency = 1
    amplitude = 1
    phase = 0
    time_window = 5
    time_step = 0.01

    # Generate wave data
    t = np.arange(0, time_window, time_step)
    y = amplitude * np.sin(2*np.pi*frequency*t + phase)

    # Plot wave
    plt.figure()
    plt.plot(t, y)
    plt.show()


if __name__ == '__main__':
    main()
