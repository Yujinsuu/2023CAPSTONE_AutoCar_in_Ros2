import numpy as np

def return_offset(x0, y0, px, py):
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


def main():
    # Example usage
    x0 = 2
    y0 = 3
    px = [1, 2, 3, 4, 5]
    py = [2, 4, 6, 8, 10]

    offset_x, offset_y = return_offset(x0, y0, px, py)
    print(f"offset_x: {offset_x}")
    print(f"offset_y: {offset_y}")

if __name__ == '__main__':
    main()
