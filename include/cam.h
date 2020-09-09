#include <string>
#include <vector>
#include<assert.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#ifndef CAM_H_
#define CAM_H_

/** 
 * @brief                相机内参标定函数，使用圆点标定板<7x7>
 * @param rootDir        使用的图像的根路径
 * @param calPicNum      标定使用的图像数量
 * @param patternSize    标定板的模式<7x7>
 * @param undistImgDir   待去畸变的图像路径
 * @param undistImgNum   待去畸变的图像数量
 *
 * @return 标定结果，成功或者失败
 *     -<em>false</em> 标定失败
 *     -<em>true</em>  标定成功
 */
bool cameraCal(
    const std::string& rootDir, 
    const int& calPicNum,
    const cv::Size& patternSize,
    const std::string& undistImgDir,
    const int& undistImgNum
);

/** 
 * @brief                检测图像中标定板的四条边缘线
 * @param rootDir        使用的图像的根路径
 * @param calPicNum      联合标定图像数量
 *
 * @return 计算结果，成功或者失败
 *     -<em>false</em> 计算失败
 *     -<em>true</em>  计算成功
 */
bool lsdDetector(
    const std::string& rootDir,
    const int& calPicNum
);

#endif