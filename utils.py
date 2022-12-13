import time
import numpy as np
from scipy import interpolate


def reading_log_files(filename):
    with open(filename, "r") as f:
        data = f.read().splitlines()
    return data


def log_generator(filename, period=0.2):
    data = reading_log_files(filename)
    while True:
        time.sleep(period)
        new_data = reading_log_files(filename)
        yield new_data[len(data):]
        data = new_data


def smoothen(waypoints):
    x, y = zip(*waypoints)
    tck, *rest = interpolate.splprep([x, y], k=2)
    u = np.linspace(0, 1, num=20)
    x_smooth, y_smooth = interpolate.splev(u, tck)
    return list(zip(x_smooth, y_smooth))