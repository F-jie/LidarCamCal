import numpy as np
from cv2 import cv2

def getR(rPara):
    left = np.stack(rPara[:3]).T
    right = np.stack(rPara[3:]).T
    R = np.matmul(left, np.linalg.inv(right))
    return R

def getT(tPara, R):
    I = np.identity(3, dtype=np.float)
    tmp = tPara[0]
    tmp.resize(3,1)
    tmp1 = tPara[4]
    tmp1.resize(3, 1)
    tmp2 = tPara[2]
    tmp2.resize(3, 1)
    left1 = I-np.matmul(tmp, tmp.T)
    right1 = -np.matmul(I-np.matmul(tmp, tmp.T), np.matmul(R, tmp1)-tmp2)

    tmp = tPara[1]
    tmp.resize(3,1)
    tmp1 = tPara[5]
    tmp1.resize(3, 1)
    tmp2 = tPara[3]
    tmp2.resize(3, 1)
    left2 = I-np.matmul(tmp, tmp.T)
    right2 = -np.matmul(I-np.matmul(tmp, tmp.T), np.matmul(R, tmp1)-tmp2)

    tmp = tPara[6]
    tmp.resize(1,3)
    tmp1 = tPara[8]
    tmp1.resize(3,1)
    left3 = tmp
    right3 = -np.matmul(tmp, np.matmul(R, tmp1))-tPara[7]

    left = np.concatenate([left1, left2, left3])
    right = np.concatenate([right1, right2, right3])

    q, r = np.linalg.qr(left)
    qb = np.matmul(q.T, right)
    t = np.matmul(np.linalg.inv(r), qb)
    return t

if __name__ == "__main__":
    param = {}
    paramFile = "../data/20191223/camlidcal/process/resultCanUse.txt"
    with open(paramFile, "r") as fp:
        lines = fp.readlines()
    for line in lines:
        if line.strip() != "":
            line = line.strip().split(":")
            param[line[0]] = [float(x) for x in line[1][1:].split()]
    
    direc = np.array(param["leftCC"][3:6], dtype=np.float)- \
            np.array(param["leftCC"][8:11], dtype=np.float32)
    direc = direc/np.linalg.norm(direc)

    intrinsic = np.array(param["intrinsic"], dtype=np.float)
    intrinsic.resize(3,3)
    point1 = np.array(param["leftCC"][3:6], dtype=np.float)
    point1.resize(3, 1)
    uv1 = np.matmul(intrinsic, point1)
    point2 = np.array(param["leftCC"][8:11], dtype=np.float)
    point2.resize(3, 1)
    uv2 = np.matmul(intrinsic, point2)

    img = cv2.imread("../data/20191223/camlidcal/process/3.bmp")
    # cv2.line(img, (int(param["leftCC"][6]), int(param["leftCC"][7])), \
    #               (int(param["leftCC"][11]), int(param["leftCC"][12])), \
    #               (0,0,255), 1)
    mid = (int((param["leftCC"][6]+param["leftCC"][11])/2), 
           int((param["leftCC"][7]+param["leftCC"][12])/2))
    # cv2.line(img, (int(param["leftCC"][6]), int(param["leftCC"][7])), \
    #                mid, (0,255,255), 2)
    
    point1 = np.array([0,0,0,1], dtype=np.float)
    point1.resize(4,1)
    point2 = np.array([0,5,0, 1], dtype=np.float)
    point2.resize(4,1)
    HC2B = np.array(param["HC2B"], dtype=np.float)
    HC2B.resize(3,4)
    uv1 = np.matmul(np.matmul(intrinsic, HC2B), point1)
    uv2 = np.matmul(np.matmul(intrinsic, HC2B), point2)
    # cv2.line(img, (int(uv1[0]/uv1[2]), int(uv1[1]/uv1[2])), \
    #               (int(uv2[0]/uv2[2]), int(uv2[1]/uv2[2])), \
    #               (0,0,255), 1)
    
    point1 = np.array(param["planeLC"][4:7], dtype=np.float)
    point1.resize(3, 1)
    point2 = np.array(param["planeLC"][7:10], dtype=np.float)
    point2.resize(3, 1)
    point3 = np.array(param["planeLC"][10:13], dtype=np.float)
    point3.resize(3, 1)
    point4 = np.array(param["planeLC"][13:16], dtype=np.float)
    point4.resize(3, 1)
    # dierc1 = (point1-point2)/np.linalg.norm(point1-point2)
    # dierc2 = (point3-point4)/np.linalg.norm(point3-point4)
    # norm = np.cross(dierc1, dierc2)/np.linalg.norm(np.cross(dierc1, dierc2))

    rPara = [-np.array(param["topCC"][0:3], dtype=np.float), \
             -np.array(param["leftCC"][0:3], dtype=np.float), \
             np.array(param["planeCC"][0:3], dtype=np.float), \
             -np.array(param["topLC"][0:3], dtype=np.float), \
             -np.array(param["leftLC"][0:3], dtype=np.float), \
             np.array(param["planeLC"][0:3], dtype=np.float)]
    R = getR(rPara)
    print(R)

    tPara = [np.array(param["topCC"][0:3], dtype=np.float), \
             np.array(param["leftCC"][0:3], dtype=np.float), \
             np.array(param["topCC"][3:6], dtype=np.float), \
             np.array(param["leftCC"][3:6], dtype=np.float), \
             np.array(param["topLC"][3:], dtype=np.float), \
             np.array(param["leftLC"][3:], dtype=np.float), \
             np.array(param["planeCC"][0:3], dtype=np.float), \
             np.array(param["planeCC"][3], dtype=np.float), \
             np.array(param["planeLC"][4:7], dtype=np.float)]
    t = getT(tPara, R)
    print(t)

    point1 = -np.array(param["topLC"][0:3], dtype=np.float)*30+\
             np.array(param["topLC"][3:6], dtype=np.float)
    point1.resize(3, 1)
    point2 = np.array(param["topLC"][3:6], dtype=np.float)
    point2.resize(3, 1)
    uv1 = np.matmul(intrinsic, np.matmul(R, point1)+t)
    uv1.resize(3)
    uv2 = np.matmul(intrinsic, np.matmul(R, point2)+t)
    uv2.resize(3)
    # cv2.line(img, (int(uv1[0]/uv1[2]), int(uv1[1]/uv1[2])), \
    #               (int(uv2[0]/uv2[2]), int(uv2[1]/uv2[2])), (255,0,0), 3)  

    with open("../data/20191223/camlidcal/3.txt", "r") as fp:
        lines = fp.readlines()
    for line in lines:
        if line.strip() != "":
            line = [float(x) for x in line.strip().split()]
            point = np.array(line, dtype=np.float)
            point.resize(3,1)
            uv = np.matmul(intrinsic, np.matmul(R, point)+t)
            cv2.circle(img, (int(uv[0]/uv[2]), int(uv[1]/uv[2])), 1, (255,0,0), 1)
    # with open("../data/20191223/camlidcal/processed/plane1Fine.pcd", "r") as fp:
    #     lines = fp.readlines()
    # for line in lines:
    #     if line.strip() != "":
    #         line = [float(x) for x in line.strip().split()]
    #         point = np.array(line, dtype=np.float)
    #         point.resize(3,1)
    #         uv = np.matmul(intrinsic, np.matmul(R, point)+t)
    #         cv2.circle(img, (int(uv[0]/uv[2]), int(uv[1]/uv[2])), 1, (0,0,255), 1)
    # cv2.imshow("TEST", img)
    # cv2.waitKey()
    # cv2.imwrite("./result3.bmp", img)



    