import numpy as np

def point_offset(x0, y0, px, py):
    # Convert input lists to numpy arrays
    px = np.array(px)
    py = np.array(py)

    # Calculate the coefficients of the linear interpolation line (y = mx + c)
    A = np.vstack([px, np.ones(len(px))]).T
    m, c = np.linalg.lstsq(A, py, rcond=None)[0]

    # Calculate the coordinates of the closest point on the interpolated line
    x_a = (x0 + m * y0 - m * c) / (1 + m**2)
    y_a = m * x_a + c

    # Calculate the offset to move point P to point Q
    offset_x = x_a - x0
    offset_y = y_a - y0

    return offset_x, offset_y

def line_offset(ax, ay, px, py):
    # Convert input lists to numpy arrays
    ax = np.array(ax)
    ay = np.array(ay)
    px = np.array(px)
    py = np.array(py)

    # Calculate the coefficients of the linear interpolation line (y = mx + c) for line A
    A = np.vstack([ax, np.ones(len(ax))]).T
    m_a, c_a = np.linalg.lstsq(A, ay, rcond=None)[0]

    # Calculate the perpendicular distance from each point in P to line A
    distances = []
    for x0, y0 in zip(px, py):
        m_p = -1 / m_a  # Slope of the perpendicular line
        c_p = y0 - m_p * x0  # Y-intercept of the perpendicular line

        # Calculate the intersection point between line A and the perpendicular line
        x_a = (c_p - c_a) / (m_a - m_p)
        y_a = m_a * x_a + c_a

        # Calculate the perpendicular distance between the point (x0, y0) and line A
        distance = np.sqrt((x0 - x_a)**2 + (y0 - y_a)**2)
        distances.append(distance)

    # Calculate the average offset
    offset_x = np.mean(distances) * np.cos(np.arctan(m_a))
    offset_y = np.mean(distances) * np.sin(np.arctan(m_a))

    return offset_x, offset_y

def main():
    # Example usage
    x0 = 2
    y0 = 3
    px = [1, 2, 3, 4, 5]
    py = [2, 4, 6, 8, 10]

    offset_x, offset_y = point_offset(x0, y0, px, py)
    print(f"offset_x: {offset_x}")
    print(f"offset_y: {offset_y}")

if __name__ == '__main__':
    main()
