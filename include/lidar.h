#include <string>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

#ifndef LIDAR_H_
#define LIDAR_H_

/** 
 * @brief                检测点云中的平面
 * @param cloudPath      原始点云路径
 * @param planePath      检测结果保存路径
 *
 * @return 标定结果，成功或者失败
 *     -<em>false</em> 标定失败
 *     -<em>true</em>  标定成功
 */
bool planeDetection(int idx, std::string rootDir);


/** 
 * @brief                检测点云偏高面的边界点
 * @param planePath      平面对应的点云的路径
 * @param boundaryPath      检测结果保存路径
 *
 * @return 标定结果，成功或者失败
 *     -<em>false</em> 标定失败
 *     -<em>true</em>  标定成功
 */
bool detecBoundary(int idx, std::string rootDir);

/** 
 * @brief                显示点云
 * @param cloudPath      原始点云路径
 *
 * @return None
 */
void cloudViewer(std::string cloudPath);

/** 
 * @brief                显示点云
 * @param cloudPath      原始点云路径
 *
 * @return None
 */
void showLineInCloud(std::string cloudPath, std::string linePath);

#endif