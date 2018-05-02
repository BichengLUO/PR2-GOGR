import sys
import numpy as np
import matplotlib.pyplot as plt

def main():
    with open(sys.argv[1], 'rb') as f:
        data = np.fromfile(f, '<f4')
        data = data.reshape((int(sys.argv[3]), int(sys.argv[2])))
        if len(sys.argv) == 5:
            with open(sys.argv[4], 'rb') as f:
                back = np.fromfile(f, '<f4')
                back = back.reshape((int(sys.argv[3]), int(sys.argv[2])))
                data[np.abs(data - back) < 0.05] = 0
        plt.imshow(data)
        plt.colorbar()
        plt.show()

if __name__ == '__main__':
    main()