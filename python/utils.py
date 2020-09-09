import random
import numpy as np
from cv2 import cv2
import itertools as it

def getHomo(R, T):
    data = np.zeros((3,4), dtype=np.float)
    R = np.array(R, dtype=np.float)
    R.resize(3,3)
    data[:, :3] = R
    T = np.array(T, dtype=np.float)
    T.resize(3)
    data[:, 3] = T
    data.resize(12)
    return list(data)

def get3DparamInLC(idx, boundaryDir):
    res = []
    for i in range(4):
        data_path = boundaryDir+"{}{}.pcd".format(idx, i+1)
        with open(data_path, "r") as fp:
            lines = fp.readlines()

        dist_func_names = it.cycle('DIST_HUBER DIST_L2 DIST_L1 DIST_L12 DIST_FAIR \
                                    DIST_WELSCH'.split())
        func = getattr(cv2, next(dist_func_names))

        data = []
        for line in lines:
            if(line.strip() != ""):
                data.append([float(x) for x in line.strip().split()])
        data = np.array(data)

        res.append(cv2.fitLine(data, func, 0, 0.01, 0.01))
    return res

def get3DRepPlane(rawData, cameraIntrinsic):
    cameraIntrinsicInv = np.linalg.inv(cameraIntrinsic)
    extend = np.array([1.0], dtype=np.float)
    PlaneNorms = []
    for data in rawData:
        data = np.array([float(x) for x in data.split()], dtype=np.float)
        PlaneNorm = np.cross(np.matmul(cameraIntrinsicInv, np.concatenate([data[:2], extend])), \
                             np.matmul(cameraIntrinsicInv, np.concatenate([data[2:], extend])))
        PlaneNorms.append(PlaneNorm/np.linalg.norm(PlaneNorm))
    if len(PlaneNorms) == 1:
        return [0.0], PlaneNorms
    else:
        error = []
        for i in range(len(PlaneNorms)-1):
            for j in range(i+1, len(PlaneNorms)):
                error.append(1.0-abs(np.matmul(PlaneNorms[i].T, PlaneNorms[j])))
        return error, PlaneNorms

def getNorm(UVs, intrinsic):
    res = []
    intrinsic = np.array(intrinsic, dtype=np.float)
    intrinsic.resize(3,3)
    if isinstance(UVs[0], list):
        for uv in UVs:
            uv0 = np.ones(3, dtype=np.float)
            uv1 = np.ones(3, dtype=np.float)
            uv0[:2] = np.array(uv[:2], dtype=np.float)
            uv1[:2] = np.array(uv[2:], dtype=np.float)
            uv0.resize(3, 1)
            uv1.resize(3, 1)
            direc0 = np.matmul(np.linalg.inv(intrinsic), uv0)
            direc1 = np.matmul(np.linalg.inv(intrinsic), uv1)
            direc0.resize(3)
            direc1.resize(3)
            norm = np.cross(direc0, direc1)/np.linalg.norm(np.cross(direc0, direc1))
            res.append(norm)
    else:
        uv0 = np.ones(3, dtype=np.float)
        uv1 = np.ones(3, dtype=np.float)
        uv0[:2] = np.array(UVs[:2], dtype=np.float)
        uv1[:2] = np.array(UVs[2:], dtype=np.float)
        uv0.resize(3, 1)
        uv1.resize(3, 1)
        direc0 = np.matmul(np.linalg.inv(intrinsic), uv0)
        direc1 = np.matmul(np.linalg.inv(intrinsic), uv1)
        direc0.resize(3)
        direc1.resize(3)
        norm = np.cross(direc0, direc1)/np.linalg.norm(np.cross(direc0, direc1))
        return norm
    ret = res[0]
    for item in res[1:]:
        if ret.dot(item) < 0:
            ret += -item
        else:
            ret += item
    return ret/len(res)

def solveEquations(norm1, inter1, norm2, inter2, cameraIntrinsic):
    norm1.resize(3)
    norm2.resize(3)
    inter1.resize(1)
    intrinsic = np.array(cameraIntrinsic, dtype=np.float)
    intrinsic.resize(3,3)
    norm1.resize(3)
    norm2.resize(3)
    direc = np.cross(norm1, norm2)
    direc = direc/np.linalg.norm(direc)
    x = 1.0
    tmp = norm1/norm1[2]-norm2/norm2[2]
    y = -(inter1/norm1[2]-inter2/norm2[2]+tmp[0])/tmp[1]
    z = -(inter1+norm1[0]*x+norm1[1]*y)/norm1[2]
    point = np.array([x,y,z],dtype=np.float)

    points = []
    counter = 0
    randPoint = point
    while counter < 2:
        step = random.randint(-1000, 1000)
        randPoint = direc*step+randPoint
        uv = np.matmul(intrinsic, randPoint)
        u = uv[0]/uv[2]
        v = uv[1]/uv[2]
        if u > 0 and u < 1248:
            if v> 0 and v < 736:
                points.append([randPoint, (int(u), int(v))])
                counter += 1
    ret = []
    direc.resize(3)
    ret = ret+list(direc)
    for item in points:
        tmp = item[0]
        uv = item[1]
        tmp.resize(3)
        ret = ret+list(tmp)+[uv[0], uv[1]]
    return ret

def get3DparamInCC(idx, result, H):
    homo = np.array(H, dtype=np.float)
    homo.resize(3, 4)
    cameraIntrinsic = result["newIntrinsic"]
    boardPlaneNormBC = np.array([0,0,1], dtype=np.float)
    boardPlaneNormBC.resize(3, 1)
    boardPointBC = np.array([0,0,0,1],dtype=np.float).T
    boardPointBC.resize(4, 1)
    # boardPlaneInterBC = 0.0
    boardPlaneNormCC = np.matmul(homo[:3, :3], boardPlaneNormBC)
    boardPointCC = np.matmul(homo, boardPointBC)
    boardPlaneInterCC = -np.matmul(boardPlaneNormCC.T, boardPointCC)

    repPlaneInterCC = 0.0
    topPlaneNormCC = getNorm(result["topUV{}".format(idx)], cameraIntrinsic)
    downPlaneNormCC = getNorm(result["downUV{}".format(idx)], cameraIntrinsic)
    leftPlaneNormCC = getNorm(result["leftUV{}".format(idx)], cameraIntrinsic)
    rightPlaneNormCC = getNorm(result["rightUV{}".format(idx)], cameraIntrinsic)
    topPara = solveEquations(boardPlaneNormCC, boardPlaneInterCC, \
                            topPlaneNormCC, repPlaneInterCC, cameraIntrinsic)
    downPara = solveEquations(boardPlaneNormCC, boardPlaneInterCC, \
                            downPlaneNormCC, repPlaneInterCC, cameraIntrinsic)
    leftPara = solveEquations(boardPlaneNormCC, boardPlaneInterCC, \
                            leftPlaneNormCC, repPlaneInterCC, cameraIntrinsic)
    rightPara = solveEquations(boardPlaneNormCC, boardPlaneInterCC, \
                            rightPlaneNormCC, repPlaneInterCC, cameraIntrinsic)

    boardPlaneNormCC.resize(3)
    boardPlaneInterCC.resize(1)
    boardPointCC.resize(3)
    planePara = list(boardPlaneNormCC)+list(boardPlaneInterCC)+list(boardPointCC)
    return topPara, downPara, leftPara, rightPara, planePara

def rawResultProcess(rawResultPath):
    with open(rawResultPath, 'r') as fp:
        rawData = fp.readlines()
    rawResult = {}
    for line in rawData:
        if line.strip() != "":
            line = line.strip().split(":")
            if len(line) != 1:
                if line[0] in rawResult:
                    rawResult[line[0]].append(line[1][1:])
                else:
                    rawResult[line[0]] = [line[1][1:]]
    
    result = {}
    for key in rawResult.keys():
        tmp = rawResult[key]
        if len(tmp) > 1:
            result[key] = []
            for item in tmp:
                item = item.split()
                result[key].append([float(x) for x in item])
        else:
            item = tmp[0].split()
            result[key] = [float(x) for x in item]
    return result

def getRTParam(result, patternViewNum):
    rootDir = "../data/20191223/camlidcal/process/"
    RTParam = {}
    RTParam["intrinsic"] = result["newIntrinsic"]
    
    for i in range(patternViewNum):
        RTParam["HC2B{}".format(i+1)] = getHomo(result["RC2B{}".format(i+1)], \
                                                result["TC2B{}".format(i+1)])
        # print(RTParam["HC2B{}".format(i+1)])
        paramCC = get3DparamInCC(i+1, result, RTParam["HC2B{}".format(i+1)])
        print("图像中四条边缘直线的参数为(n,p): ")
        for i in range(4):
            print("第{}条".format(i+1), paramCC[i][:6])
        return

        RTParam["topCC{}".format(i+1)] = paramCC[0]
        RTParam["downCC{}".format(i+1)] = paramCC[1]
        RTParam["leftCC{}".format(i+1)] = paramCC[2]
        RTParam["rightCC{}".format(i+1)] = paramCC[3]
        RTParam["planeCC{}".format(i+1)] = paramCC[4]
        linesLC = get3DparamInLC(i+1, rootDir)
        tmp = linesLC[0]
        tmp.resize(6)
        RTParam["line1LC{}".format(i+1)] = list(tmp)
        tmp = linesLC[1]
        tmp.resize(6)
        RTParam["line2LC{}".format(i+1)] = list(tmp)
        tmp = linesLC[2]
        tmp.resize(6)
        RTParam["line3LC{}".format(i+1)] = list(tmp)
        tmp = linesLC[3]
        tmp.resize(6)
        RTParam["line4LC{}".format(i+1)] = list(tmp)
        planePath = rootDir+"finePlane{}.pcd".format(i+1)
        with open(planePath, "r") as fp:
            lines = fp.readlines()[100:-100]
        numPoints = len(lines)
        RTParam["planeLC{}".format(i+1)] = result["plane1"]+ \
                    [float(x) for x in lines[0].strip().split()]+ \
                    [float(x) for x in lines[numPoints//3].strip().split()]+ \
                    [float(x) for x in lines[2*numPoints//3].strip().split()]+ \
                    [float(x) for x in lines[numPoints-1].strip().split()]
        
    return RTParam

def showDirec(idx, result):
    img = cv2.imread("../data/20191223/camlidcal/processed/{}.bmp".format(idx))
    end1 = result[1][0][1]
    
    mid = (int((result[1][0][1][0]+result[1][1][1][0])/2), \
           int((result[1][0][1][1]+result[1][1][1][1])/2))
    cv2.line(img, end1, mid, (0,0,255), 2)
    cv2.line(img, result[1][0][1], result[1][1][1], (0,255,0), 1)
    cv2.imshow("TEST", img)
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # idx = 1
    # bPath = "../data/20191223/camlidcal/process/"
    # res = get3DparamInLC(idx, bPath)
    # print("四条边界的参数为：")
    # for i in range(len(res)):
    #     print("第{}条: ".format(i+1), res[i].reshape(-1))
    rootDir = "../data/20191223/camlidcal/process/"
    result = rawResultProcess(rootDir+"result.txt")
    RTParam = getRTParam(result, 3)
