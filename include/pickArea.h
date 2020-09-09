#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef PICKAREA_H_
#define PICKAREA_H_

class pickArea {
private:
    int cloudId;
    std::string cloudName;
    std::string rootDir;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr selectArea;

public:
    pickArea();
    ~pickArea();
    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
    bool isSelected();
    void saveSelectArea();
    void simpleViewer();

protected:
    void pickCallback(const pcl::visualization::AreaPickingEvent& event, void*);
};

#endif