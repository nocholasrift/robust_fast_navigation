import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def setupPlot(a, b, c, d, ax):
    x = np.linspace(-3,3,2)
    y = np.linspace(-3,3,2)

    X, Y, Z = None, None, None

    if c != 0:
        X,Y = np.meshgrid(x,y)
        Z = (d - a*X - b*Y) / c
    elif b != 0:
        X,Z = np.meshgrid(x,y)
        Y = (-d - a*X)/b
    elif a != 0:
        Y,Z = np.meshgrid(x,y)
        X = (-d - b*Y)/a

    if c == 0 and b == 0 and a == 0:
        print("normal vector is all 0")
        return
    
    surf = ax.plot_surface(X, Y, Z)

def main():
    a,b,c,d = 0,-4,0,-8

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # setupPlot(-1.15, -1.18, 0.00, 0.89, ax)
    # setupPlot(0.86, -1.88, -0.00, -5.82, ax)
    # setupPlot(2.00, 1.77, -0.00, -7.36, ax)
    # setupPlot(-0.00, 1.30, -0.00, -1.30, ax)
    # setupPlot(-1.71, -0.00, 0.00, 2.57, ax)
    setupPlot(0,0,1,1, ax)

    plt.show()

if __name__ == "__main__":
    main()
