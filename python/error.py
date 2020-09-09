import numpy as np
from cv2 import cv2

if __name__ == "__main__":
    result = "../data/20191223/camlidcal/process/resultFinal.txt"
    ret = {}
    with open(result, "r") as fp:
        lines = fp.readlines()
    for line in lines:
        if line.strip() != "":
            line = line.strip().split(":")
            ret[line[0]] = [float(x) for x in line[1][1:].split()]
    
    HC2B = np.array(ret["HC2B"]+[0,0,0,1], dtype=np.float)
    HC2B.resize(4, 4)
    intrinsic = np.array(ret["intrinsic"], dtype=np.float)
    intrinsic.resize(3,3)
    HC2L = np.array([-7.14592269e-03, -9.22543752e-01, 3.85939245e-01, -3.42776233e+00,\
                     9.99964231e-01, -9.55852102e-04, 6.72321284e-03, -1.90213766e-01,\
                     3.89092445e-03, -3.85889574e-01, -9.22508767e-01, 2.39265027e+02, 
                     0,0,0,1], dtype=np.float)
    HC2L.resize(4, 4)


    HL2B = np.matmul(np.linalg.inv(HC2L), HC2B)

    pointsOnBoard = []
    for i in range(7):
        for j in range(7):
            pointsOnBoard.append((float(i*5),float(j*5),0.0))
    imgPath = "../data/20191223/camlidcal/process/1.bmp"
    img = cv2.imread(imgPath)
    _, grid = cv2.findCirclesGrid(img, (7, 7))
    
