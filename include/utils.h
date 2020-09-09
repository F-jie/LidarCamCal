#include <fstream>

#include <cam.h>

#ifndef UTILS_H_
#define UTILS_H_

bool write(
    const std::string& saveDir, 
    const std::string& itemName, 
    const cv::Mat& data
);

bool write(
    const std::string& saveDir, 
    const std::string& itemName, 
    const cv::Vec4f& data
);

std::vector<std::string> split(const std::string &text, char* sep);

#endif