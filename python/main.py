import numpy as np
from cv2 import cv2
from utils import rawResultProcess, getRTParam

def main():
    rootDir = "../data/20191223/camlidcal/process/"
    result = rawResultProcess(rootDir+"result.txt")
    RTParam = getRTParam(result, 3)
    # with open("../data/20191223/camlidcal/process/resultFinal.txt", "w") as fp:
    #     fp.write("topCC: ")
    #     for item in RTParam["topCC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("downCC: ")
    #     for item in RTParam["downCC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("leftCC: ")
    #     for item in RTParam["leftCC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("rightCC: ")
    #     for item in RTParam["rightCC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("planeCC: ")
    #     for item in RTParam["planeCC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")

    #     fp.write("intrinsic: ")
    #     for item in RTParam["intrinsic"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("HC2B: ")
    #     for item in RTParam["HC2B1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")

    #     fp.write("line1LC: ")
    #     for item in RTParam["line1LC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("line2LC: ")
    #     for item in RTParam["line2LC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("line3LC: ")
    #     for item in RTParam["line3LC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("line4LC: ")
    #     for item in RTParam["line4LC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")
    #     fp.write("planeLC: ")
    #     for item in RTParam["planeLC1"]:
    #         fp.write(str(item)+" ")
    #     fp.write("\n")

    

if __name__ == "__main__":
    main()