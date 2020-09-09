#include <utils.h>

bool write(
    const std::string& saveDir, 
    const std::string& itemName, 
    const cv::Mat& data
) {
    std::ofstream fp(saveDir, std::ios::app);
    if(data.isContinuous()) {
        fp << itemName << ": ";
        int nCols = data.cols, nRows = data.rows;
        nCols *= nRows;
        nRows = 1;
        const double* p;
        for(int i=0; i<nRows; ++i)
        {
            p = data.ptr<double>(i);
            for (int j = 0; j<nCols-1; ++j)
            {
                fp << p[j] << " ";
            }
            fp << p[nCols-1] << "\n";
        }
        fp.close();
        return true;
    }
    fp.close();
    return false;
}

bool write(
    const std::string& saveDir, 
    const std::string& itemName, 
    const cv::Vec4f& data
) {
    std::ofstream fp(saveDir, std::ios::app);
    fp << itemName << ": ";
    for(int i = 0;i<3;++i) {
        fp << data[i] << " ";
    }
    fp << data[3] << "\n";
    fp.close();
    return true;
}

std::vector<std::string> split(const std::string &text, char* sep) {
    std::vector<std::string> tokens; 
    std::size_t start = 0, end = 0; 
    while ((end = text.find(*sep, start)) != std::string::npos) { 
        tokens.push_back(text.substr(start, end - start));  
        start = end + 1; 
    } 
    tokens.push_back(text.substr(start)); 
    return tokens; 
}
