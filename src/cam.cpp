#include "cam.h"
#include "utils.h"

bool cameraCal(
    const std::string& rootDir, 
    const int& calPicNum,
    const cv::Size& patternSize,
    const std::string& undistImgDir,
    const int& undistImgNum
) {
    cv::Size imgSize;
    std::vector<std::vector<cv::Point2f> > pointsOnImg;
    std::vector<std::vector<cv::Point3f> > pointsOnBoard;

    int counter = 0;
    std::vector<int> availablePattern;
    for(int i = 0; i < calPicNum; ++i) {
        std::vector<cv::Point2f> result;
        std::string imgPath = rootDir+std::to_string(i+1)+".bmp";
        cv::Mat img = cv::imread(imgPath);
        if(imgSize.empty()) {
            imgSize.width = img.cols;
            imgSize.height = img.rows;
        }
        cv::Mat grayImg;
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);
        if(cv::findCirclesGrid(grayImg, patternSize, result)) {
            availablePattern.push_back(i);
            cv::drawChessboardCorners(grayImg, patternSize, result, true);
            cv::imshow("result", grayImg);
            cv::waitKey();
            pointsOnImg.push_back(result);
            counter++;
        } else continue;
    }

    std::cout << "可用的标定图像索引：" << std::endl;
    for(int i = 0; i < availablePattern.size(); ++i) {
        std::cout << availablePattern[i] << " ";
    }
    std::cout << std::endl;
    
    std::vector<cv::Point3f> tmp;
    for(int j = 0; j < 7; ++j) {
        for(int k = 0; k < 7; ++k) {
            tmp.push_back(cv::Point3f(float(j*5),float(k*5),0.0));
        }
    }
    for(int i = 0; i < counter; ++i) pointsOnBoard.push_back(tmp);

    assert(pointsOnBoard.size() == pointsOnImg.size());

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F);
    cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F);
    cv::Mat rvecs;
    cv::Mat tvecs;
    double reprojectError;
    reprojectError = cv::calibrateCamera(pointsOnBoard, pointsOnImg, imgSize, 
                                         cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout << "Type: " << cameraMatrix.type() << std::endl;

    cv::Mat newMtx;
    newMtx = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, 
                                             imgSize, 1, imgSize);

    std::cout << "标定结果：" << std::endl;
    std::cout << "Intrinsic matrix：" << std::endl;
    std::cout << cameraMatrix << std::endl;
    std::cout << "Value at [0,0]: " << cameraMatrix.at<double>(0,0) << std::endl;
    std::cout << "相机畸变参数：" << std::endl;
    std::cout << distCoeffs << std::endl;
    std::cout << "重投影误差：" << reprojectError << std::endl;
    std::string paramDir = undistImgDir+"process/result.txt";
    write(paramDir, "intrinsic", cameraMatrix);
    write(paramDir, "distCoeffs", distCoeffs);
    write(paramDir, "newIntrinsic", newMtx);

    for(int i = 0; i < undistImgNum; ++i) {
        std::string imgPath = undistImgDir+std::to_string(i+1)+".bmp";
        cv::Mat img = cv::imread(imgPath);
        cv::Mat undistImg;
        cv::undistort(img, undistImg, cameraMatrix, distCoeffs, newMtx);
        std::string undistImgPath = undistImgDir+"process/"+std::to_string(i+1)+".bmp";
        cv::imwrite(undistImgPath, undistImg);

        cv::Mat rvecs;
        cv::Mat tvecs;
        std::vector<cv::Point2f> result;
        cv::Mat grayImg;
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);
        if(cv::findCirclesGrid(grayImg, patternSize, result)) {
            cv::solvePnP(tmp, result, cameraMatrix, 
                            distCoeffs, rvecs, tvecs);
            cv::Mat rotation;
            cv::Rodrigues(rvecs, rotation);
            std::cout << "估计标定板相对于相机坐标系的位姿："<< std::endl;
            std::cout << "旋转参数为：" << rotation << std::endl;
            std::cout << "平移参数为：" << tvecs << std::endl;
            write(paramDir, "RC2B"+std::to_string(i+1), rotation);
            write(paramDir, "TC2B"+std::to_string(i+1), tvecs);
        } else {
            std::cout << "Can't detect the centers of pic: " 
                    << std::to_string(i+1)+".bmp!" << std::endl;
            return false;
        }
    }
}

bool lsdDetector(
    const std::string& rootDir,
    const int& calPicNum
) {
    std::cout << "Input '1' for top;'2' for down;'3' for left;'4' for right;'q' for deprecated!" << std::endl;
    for(int i=0;i<calPicNum;++i) {
        std::string imgPath = rootDir+std::to_string(i+1)+".bmp";
        cv::Mat grayImg, img = cv::imread(imgPath);
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);
        cv::blur(grayImg, grayImg, cv::Size(3, 3));
        cv::Canny(grayImg, grayImg, 20, 200, 3);
        cv::Ptr<cv::LineSegmentDetector> lsd = 
                            cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
        std::vector<cv::Vec4f> lines;
        lsd->detect(grayImg, lines);
        std::vector<cv::Vec4f> result;
        std::string paramDir = rootDir+"result.txt";
        if(lines.size() > 0) {

            std::cout << lines.size() << " lines is detected!" << std::endl;
            
            for(int j = 0; j < lines.size(); j++) {
                if(std::abs(lines[j][0]-lines[j][2]) > 100 || 
                std::abs(lines[j][1]-lines[j][3]) > 100) {
                    cv::Mat tmpImg = cv::imread(imgPath);
                    cv::line(tmpImg, cv::Point2f(lines[j][0], lines[j][1]),
                             cv::Point2f(lines[j][2],lines[j][3]), 
                             cv::Scalar(0,0,255),2);
                    cv::imshow("LINE ON IMG", tmpImg);
                    while(true) {
                        int key = cv::waitKey(0);
                        if(char(key) == '1') {
                            write(paramDir, "topUV"+std::to_string(i+1), lines[j]);
                            cv::destroyAllWindows();
                            break;
                        } else if(key == '2') {
                            write(paramDir,"downUV"+std::to_string(i+1), lines[j]);
                            cv::destroyAllWindows();
                            break;
                        } else if(key == '3') {
                            write(paramDir, "leftUV"+std::to_string(i+1), lines[j]);
                            cv::destroyAllWindows();
                            break;
                        } else if(key == '4') {
                            write(paramDir, "rightUV"+std::to_string(i+1), lines[j]);
                            cv::destroyAllWindows();
                            break;
                        } else {
                            cv::destroyAllWindows();
                            break;
                        }
                    }
                }
            }
        }
    }
    return true;
}
