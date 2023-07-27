import numpy as np
import matplotlib.pyplot as plt

import sympy

def change_interval_bezier(A, interval1, interval2):

    a = interval1[0]
    b = interval1[1]
    c = interval2[0]
    d = interval2[1]

    t = sympy.symbols('t')
    u = (b-a)/(d-c)*(t-c)+a
    
    B1 = sympy.Poly(-(u-1)**3)
    B2 = sympy.Poly(3*u*(u-1)**2)
    B3 = sympy.Poly(-3*u**2*(u-1))
    B4 = sympy.Poly(u**3)

    A_new = np.array([B1.all_coeffs(), 
                      B2.all_coeffs(),
                      B3.all_coeffs(),
                      B4.all_coeffs()], dtype=np.float64)


    return A_new

def main():

    ax = plt.figure().add_subplot(projection='3d')
    A_bernstein = np.array([  [-1,  3,  -3, 1],
                              [3,   -6, 3,  0],
                              [-3,  3,  0,  0],
                              [1,   0,  0,  0]])
    
    A_minvo = np.array([[-3.4416309793565660335445954842726,  6.9895482693324069156659561485867, -4.4622887879670974919932291413716,                  0.91437149799125659734933338484986],
                        [ 6.6792587678886103930153694818728, -11.845989952130473454872117144987,  5.2523596862506065630071816485724, -0.000000000000000055511151231257827021181583404541],
                        [-6.6792587678886103930153694818728,  8.1917863515353577241739913006313, -1.5981560856554908323090558042168,                 0.085628502008743445639282754200394],
                        [ 3.4416309793565660335445954842726,  -3.335344668737291184967830304231, 0.80808518737198176129510329701588, -0.000000000000000012522535092207576212786079850048]])

    interval = [-1,1]
    t = np.linspace(interval[0], interval[1], 100)
    T = np.array([t*t*t,t*t,t,1])

    A_bernstein = change_interval_bezier(A_bernstein, [0,1], interval)
    print(A_bernstein)
    # A_bernstein = np.array([[-0.125,  0.375, -0.375, 0.125],
    #                         [ 0.375, -0.375, -0.375, 0.375],
    #                         [-0.375, -0.375,  0.375, 0.375],
    #                         [ 0.125,  0.375,  0.375, 0.125]])

    # print(np.linalg.inv(A_bernstein))

    P1 = np.array([[.716, -.1370, -.4618, .3999],
                 [.3365, .8679, -.2322, .2508],
                 [.4907, -.0914, -.8137, .4999]])

    P2 = np.array([[.75, -.36, -.8675, 4.6563],
                 [.7530, 1, -.2108, 5.95],
                 [.9214, 0, -.8137, 2.4325]])

    # Ps = [P1, P2]
    Ps = [P1]

    for ind, P in enumerate(Ps):
        
        Q_bernstein = np.matmul(P, np.linalg.inv(A_bernstein))
        converter = np.matmul(A_bernstein, np.linalg.inv(A_minvo))
        Q_minvo = np.matmul(Q_bernstein, converter)

        x = np.matmul(P[0,:], T)
        y = np.matmul(P[1,:], T)
        z = np.matmul(P[2,:], T)

        print(x[-1],y[-1],z[-1])

        ax.plot(x,y,z, label=f'polynomial {ind}')
        ax.scatter(Q_bernstein[0,:], Q_bernstein[1,:], Q_bernstein[2,:], label=f'bezir ctrl{ind}')
        ax.scatter(Q_minvo[0,:], Q_minvo[1,:], Q_minvo[2,:], label=f'minvo ctrl{ind}')

    ax.legend()

    plt.show()



if __name__=="__main__":
    main()
