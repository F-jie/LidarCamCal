#include "cam.h"
#include "lidar.h"
#include "pickArea.h"

int main() {
    std::cout << "--------------------开始标定相机内参--------------------" 
              << std::endl;
    std::string rootDir = "../data/20191223/camcal/";
    std::string undistImgDir = "../data/20191223/camlidcal/";
    int patternViewNum = 36;
    cv::Size patternSize(7, 7);

    if(cameraCal(rootDir, patternViewNum, patternSize, undistImgDir, 3)) {
        std::cout << "--------------------相机内参标定结束--------------------" 
                  << std::endl;
    }
    
    // std::cout << std::endl;
    // std::cout << std::endl;

    // std::cout << "--------------------提取图像中标定板边缘--------------------" 
    //           << std::endl;
    // rootDir = "../data/20191223/camlidcal/process/";
    // patternViewNum = 3;
    // if(lsdDetector(rootDir, patternViewNum)) {
    //     std::cout << "--------------------图像参数提取完成--------------------" 
    //               << std::endl;
    // }


    // std::cout << std::endl;
    // std::cout << std::endl;

    // std::cout << "--------------------提取点云中标定板平面--------------------" 
    //           << std::endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // rootDir = "../data/20191223/camlidcal/";
    // for(int i=0;i<3;++i) {
    //     std::string cloudPath = rootDir+std::to_string(i+1)+".pcd";
    //     if(pcl::io::loadPCDFile(cloudPath, *cloud) == -1) {
    //         PCL_ERROR("Could not read file!\n");
    //         return -1;
    //     }
    //     pickArea pickViewer;
    //     pickViewer.setInputCloud(cloud);
    //     pickViewer.simpleViewer();
    // }
    // rootDir = "../data/20191223/camlidcal/process/";
    // for(int i=0;i<3;++i) {
    //     std::string coarsePlanePath = rootDir+"coarsePlane"+std::to_string(i+1)+".pcd";
    //     cloudViewer(coarsePlanePath);
    //     planeDetection(i+1, rootDir);
    //     std::string finePlanePath = rootDir+"finePlane"+std::to_string(i+1)+".pcd";       
    //     cloudViewer(finePlanePath);
    // }

    // rootDir = "../data/20191223/camlidcal/process/";
    // for(int i=0;i<3;++i) {
    //     std::string finePlanePath = rootDir+"finePlane"+std::to_string(i+1)+".pcd";
    //     detecBoundary(i+1, rootDir);
    //     std::string boundaryPath = rootDir+"boundary"+std::to_string(i+1)+".pcd";
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     if(pcl::io::loadPCDFile(boundaryPath, *cloud) == -1) {
    //         PCL_ERROR("Could not read file!\n");
    //         return -1;
    //     }
    //     pickArea pickViewer;
    //     pickViewer.setInputCloud(cloud);
    //     pickViewer.simpleViewer();
    // }

    // std::string cloudPath = "../data/20191223/camlidcal/process/11.pcd";
    // cloudViewer(cloudPath);
    return 0;
} 
