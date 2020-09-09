#include "pickArea.h"

pickArea::pickArea() {
    rootDir = "../data/20191223/camlidcal/process/";
    cloudId = 0;
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer", true));
    viewer->registerAreaPickingCallback(&pickArea::pickCallback, *this);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    selectArea = tmp;
}

pickArea::~pickArea() {}

void pickArea::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
    cloud = inCloud;
}

bool pickArea::isSelected() {
    if(selectArea->size()) {
        return true;
    }
    return false;
}

void pickArea::saveSelectArea() {
    selectArea->width = 1;
    selectArea->height = selectArea->points.size();
    std::cout << "Save or not('Y' to save others discard!):";
    std::string save;
    std::cin >> save;
    if(save == "Y") {
        std::string fileNmae;
        std::cout << "Input a name for the selected cloud: ";
        std::cin >> fileNmae;
        pcl::io::savePCDFileASCII(rootDir+fileNmae+".pcd", *selectArea);
        selectArea->clear();
        std::cout << "Saved to: " << rootDir+fileNmae+".pcd" << std::endl;
    } else {
        viewer->removePointCloud(cloudName);
        selectArea->clear();
        std::cout << "Discard the selected area!"<< std::endl;
    }
}

void pickArea::simpleViewer() {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "Cloud"); 
    viewer->resetCameraViewpoint ("Cloud"); 
    while (!viewer->wasStopped()){   
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        if(isSelected()) {
            saveSelectArea();
        }
    }
} 

void pickArea::pickCallback(
    const pcl::visualization::AreaPickingEvent& event, 
    void*
) {
    std::vector<int> indices;
    if (event.getPointsIndices(indices)==-1)
        return;
  
    for (int i = 0; i < indices.size(); ++i)
    {
        selectArea->points.push_back(cloud->points.at(indices[i]));
    }
    
  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
                                                    red(selectArea, 255, 0, 0);
  
    cloudName = std::to_string(cloudId)+"_cloudName";
    cloudId++;
    viewer->addPointCloud(selectArea, red, cloudName);
    viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}
