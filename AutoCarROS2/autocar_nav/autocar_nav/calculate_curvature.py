import math
import numpy as np
import statistics
import matplotlib.pyplot as plt

def classify_segments(x, y, threshold):
    num_points = len(x)
    segments = []

    for i in range(num_points - 1):
        if i == 0:
            # First or last point, assuming straight segment
            segments.append('Straight')
        else:
            curvature = calculate_point_curvature(x[0], y[0], x[i], y[i], x[i+1], y[i+1])

            if curvature < threshold:
                segments.append('Straight')
            else:
                segments.append('Curve')

    direction = statistics.mode(segments)

    # if direction == 'Curve':
    #     start_yaw = np.arctan2((y[1] - y[0]), (x[1] - x[0]))
    #     end_yaw = np.arctan2((y[-1] - y[-2]), (x[-1] - x[-2]))

    #     yaw_diff = calculate_yaw_difference(start_yaw, end_yaw)

    #     if yaw_diff < 60:
    #         direction = direction + '_0'
    #     elif yaw_diff < 120:
    #         direction = direction + '_1'
    #     else:
    #         direction = direction + '_2'

    return direction

def calculate_point_curvature(x1, y1, x2, y2, x3, y3):
    length1 = calculate_distance(x1, y1, x2, y2)
    length2 = calculate_distance(x2, y2, x3, y3)
    length3 = calculate_distance(x3, y3, x1, y1)
    try:
        area = calculate_triangle_area(length1, length2, length3)
    except:
        area = 0
    curvature = 2.0 * area / (length1 * length2 * length3)

    return curvature

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_triangle_area(a, b, c):
    s = (a + b + c) / 2.0
    return math.sqrt(s * (s - a) * (s - b) * (s - c))

def calculate_yaw_difference(s, e):
    s = s + np.pi if s < 0 else s
    e = e + np.pi if e < 0 else e

    return abs(np.rad2deg(s - e))

def main():
    import pandas as pd

    df = pd.read_csv('global.csv')
    mx = df['X-axis'].tolist()
    my = df['Y-axis'].tolist()

    num = int(input('start waypoint : '))

    x = mx[num:num+15]
    y = my[num:num+15]

    curv = classify_segments(x,y,0.01)

    print(curv)

    plt.figure(0)
    plt.plot(x,y,'-o')
    plt.plot(x[0], y[0], 'ro')
    plt.show()

if __name__ == '__main__':
    main()
