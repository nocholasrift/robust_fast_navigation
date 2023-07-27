import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy

M_pos_bs2mv_seg0 = np.array([
    1.1023313949144333268037598827505,   0.34205724556666972091534262290224, -0.092730934245582874453361910127569, -0.032032766697130621302846975595457,
    -0.049683556253749178166501110354147,   0.65780347324677179710050722860615,   0.53053863760186903419935333658941,   0.21181027098212013015654520131648,
    -0.047309044211162346038612724896666,  0.015594436894155586093013710069499,    0.5051827557159349613158383363043,   0.63650059656260427054519368539331,
    -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,   0.18372189915240558222286892942066
]).reshape([4,4])

M_pos_bs2mv_seg1 = np.array([
    0.27558284872860833170093997068761,  0.085514311391667430228835655725561, -0.023182733561395718613340477531892, -0.0080081916742826553257117438988644,
         0.6099042761975865811763242163579,   0.63806904207840509091198555324809,   0.29959938009132258684985572472215,    0.12252106674808682651445224109921,
        0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
     -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066
]).reshape([4,4])

M_pos_bs2mv_seg_last2 = np.array([
    0.18372189915240569324517139193631,  0.057009540927778309948870116841135, -0.015455155707597145742226985021261, -0.0053387944495218164764338553140988,
        0.70176522577378952494342456702725,   0.66657381254229453038107067186502,   0.29187180223752412500104469472717,    0.11985166952332593215402312125661,
         0.1225210667480875342816304396365,   0.29959938009132280889446064975346,   0.63806904207840497988968309073243,    0.60990427619758624810941682881094,
     -0.0080081916742826154270717964323012, -0.023182733561395621468825822830695,  0.085514311391667444106623463540018,    0.27558284872860833170093997068761
]).reshape([4,4])

M_pos_bs2mv_seg_last = np.array([
    0.18372189915240555446729331379174, 0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
       0.63650059656260415952289122287766,   0.5051827557159349613158383363043,  0.015594436894155294659469745965907,  -0.047309044211162887272337229660479,
       0.21181027098212068526805751389475,  0.53053863760186914522165579910506,   0.65780347324677146403359984105919,  -0.049683556253749622255710960416764,
     -0.032032766697130461708287185729205, -0.09273093424558248587530329132278,   0.34205724556666977642649385416007,     1.1023313949144333268037598827505
]).reshape([4,4])

M_pos_bs2be_seg0 = np.array([
    1.0000,    0.0000,   -0.0000,         0,
    0,    1.0000,    0.5000,    0.2500,
    0,   -0.0000,    0.5000,    0.5833,
    0,         0,         0,    0.1667
]).reshape([4,4])

M_pos_bs2be_seg1 = np.array([
    0.2500,    0.0000,   -0.0000,         0,
    0.5833,    0.6667,    0.3333,    0.1667,
    0.1667,    0.3333,    0.6667,    0.6667,
    0,         0,         0,    0.1667
]).reshape([4,4])

M_pos_bs2be_seg_last2 = np.array([
     0.1667,         0,   -0.0000,         0,
    0.6667,    0.6667,    0.3333,    0.1667,
    0.1667,    0.3333,    0.6667,    0.5833,
    0,         0,         0,    0.2500
]).reshape([4,4])

M_pos_bs2be_seg_last = np.array([
    0.1667,    0.0000,         0,         0,
    0.5833,    0.5000,         0,         0,
    0.2500,    0.5000,    1.0000,         0,
    0,         0,         0,    1.0000
]).reshape([4,4])

M_pos_bs2mv = [M_pos_bs2mv_seg0, M_pos_bs2mv_seg1, M_pos_bs2mv_seg_last2, M_pos_bs2mv_seg_last]
M_pos_bs2be = [M_pos_bs2be_seg0, M_pos_bs2be_seg1, M_pos_bs2be_seg_last2, M_pos_bs2be_seg_last]

def bspline(cv, n=100, degree=3):
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
    """
    cv = np.asarray(cv)
    count = cv.shape[0]

    # Prevent degree from exceeding count-1, otherwise splev will crash
    degree = np.clip(degree,1,count-1)

    # Calculate knot vector
    kv = np.array([0]*degree + list(range(count-degree+1)) + [count-degree]*degree,dtype='int')

    # Calculate query range
    u = np.linspace(0,(count-degree),n)

    # Calculate result
    return np.array(si.splev(u, (kv,cv.T,degree))).Tdef

def convert_control_pts(cps, segment):
    
    Q_be = np.zeros((3,4))
    for ind, cp in enumerate(cps):

        for i in range(3):
            Q_be[i,ind] = cp[i]

    Q_bs = np.matmul(Q_be, np.linalg.inv(M_pos_bs2be[segment]))
    Q_mv = np.matmul(Q_bs, M_pos_bs2mv[segment])

    return Q_mv


def main():

    arrX = []
    arrY = []
    cpsX = []
    cpsY = []
    cpsZ = []
    with open("/home/nick/catkin_ws/src/robust_fast_navigation/gurobi_dump/solver_output.txt", 'r') as f:
        
        i = 0
        for line in f.readlines():
            # if i < 24:
            #     if i % 3 == 0:
            #         cpsX.append(float(line.strip()))
            #     elif i % 3 == 1:
            #         cpsY.append(float(line.strip()))
            #     else: 
            #         cpsZ.append(float(line.strip()))
            # else:
            #     if i % 3 == 0:
            #         arrX.append(float(line.strip()))
            #     elif i % 3 == 1:
            #         arrY.append(float(line.strip()))
            #     else:
            #         pass

            # if i < 24:
            #     if i % 3 == 0:
            #         cpsX.append(float(line.strip()))
            #     elif i % 3 == 1:
            #         cpsY.append(float(line.strip()))
            #     else: 
            #         cpsZ.append(float(line.strip()))
            # else:
            if i % 3 == 0:
                arrX.append(float(line.strip()))
            elif i % 3 == 1:
                arrY.append(float(line.strip()))
            else:
                pass

            i += 1

        f.close()

    poly0 = np.array([[-4.975,.804],
                      [-4.975,-1.665],
                      [7.706,-1.699],
                      [7.725,1.471],
                      [7.633,2.726],
                      [.949,2.724],
                      [-4.975,.804]])


    poly1 = np.array([[.332,2.644],
                      [1.939,2.69],
                      [3.975,-.025],
                      [.15,-.264],
                      [.332,2.644]])


    # poly2 = np.array([1,            0,            0,      -11.925,
    #                 -0.105388,     0.229673,  2.70733e-06,      1.06789,
    #                 0.000885633,     0.240744,  2.98777e-06,     0.396271,
    #                 -5.55112e-17,           -1,  2.06795e-24,       -9.975,
    #                 -1,            0,            0,        1.925,
    #                 5.55112e-17,            1, -2.06795e-24,       -1.625])

    # polys = [poly0, poly1, poly2]
    polys = [poly0]
    
    plt.plot(arrX, arrY)
    for poly in polys:
        plt.plot(poly[:,0], poly[:,1], color='red')
    
    # plt.plot(np.append(cpsX[:4], cpsX[0]), np.append(cpsY[:4], cpsY[0]), 'g--')
    # plt.scatter(cpsX[4:], cpsY[4:], color='black')
    
    # Q_mv = convert_control_pts(list(zip(cpsX[:4],cpsY[:4],cpsZ[:4])), 0)
    # plt.plot(np.append(Q_mv[0,:], Q_mv[0,0]), np.append(Q_mv[1,:], Q_mv[1,0]), 'r--')
    # plt.scatter(Q_mv[0,:], Q_mv[1,:], color='red')

    # Q_mv = convert_control_pts(list(zip(cpsX[4:],cpsY[4:],cpsZ[4:])), 2)
    # plt.plot(np.append(Q_mv[0,:], Q_mv[0,0]), np.append(Q_mv[1,:], Q_mv[1,0]), 'g--')
    # plt.scatter(Q_mv[0,:], Q_mv[1,:], color='red')
    plt.show()

if __name__ == "__main__":
    main()
