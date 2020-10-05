from scipy.interpolate import interp1d
import numpy as np
import time
import matplotlib.pyplot as plt


def splinePath(goalsX, goalsY): # generate a path between the current position and the remaining goals
    # x = np.insert(goalsX, 0, position[0])
    # y = np.insert(goalsY, 0, position[1])
    x = goalsX
    y = goalsY
    f = interp1d(x, y, kind='cubic')
    return f
if __name__ == "__main__":
    np.random.seed(234)
    #rng = np.random.default_rng()
    xx = np.array([ 0, 2, 3, 7, 9, 10, 11, 14, 17, 19])
    #xx = np.sort(np.random.randint(1, 15, 10))
    #yy = np.random.randint(0, 15, 10)
    yy = np.array([8, 4, 9, 1, 3, 7, 6, 3, 0, 3])
    func = splinePath(xx, yy)
    xnew = np.linspace(0, max(xx), num=100, endpoint=True)
    plt.plot(xx, yy, 'o', xnew,func(xnew))
    plt.show()