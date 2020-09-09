#include <utils.h>
#include <lidar.h>

bool planeDetection(int idx, std::string rootDir) {
    std::string rawPlanePath = rootDir+"coarsePlane"+std::to_string(idx)+".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(rawPlanePath, *cloud) == -1) {
        PCL_ERROR("Could not read file!\n");
        return true;
    }
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMaxIterations(300);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0) {
        std::cout << "No plane found!" << std::endl;
        return true;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*plane_cloud);

    std::string planeFinePath = rootDir+"finePlane"+std::to_string(idx)+".pcd";
    pcl::io::savePCDFileASCII(planeFinePath, *plane_cloud);

    std::string paraDir = rootDir+"result.txt";
    cv::Vec4f data;
    data[0] = coefficients->values[0];
    data[1] = coefficients->values[1];
    data[2] = coefficients->values[2];
    data[3] = coefficients->values[3];

    std::cout << "平面参数为(n, inter): " << data << std::endl;
    write(paraDir, "plane"+std::to_string(idx), data);

    return true;
}

bool detecBoundary(int idx, std::string rootDir) {
    std::string planeFinePath = rootDir+"finePlane"+std::to_string(idx)+".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(planeFinePath, *cloud) == -1) {
        PCL_ERROR("Could not read file!\n");
        return true;
    }
    pcl::PointCloud<pcl::Boundary> boundary;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEst;
    normalEst.setInputCloud(cloud);
    normalEst.setRadiusSearch(2);
    normalEst.compute(*normal);

    if(normal->size() == 0) {void cloudViewer(std::string cloudPath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(cloudPath, *cloud) == -1) {
        PCL_ERROR("Could not read file!\n");
        return;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    viewer->addPointCloud(cloud);
    while(!viewer->wasStopped()) {
        viewer->spinOnce (1000);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
}
        std::cout << "Normal estimation failed!" << std::endl;
        return true;
    }
    std::cout << "Normal estimation succeed!" << std::endl;

    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normal);
    boundEst.setRadiusSearch(10);
    boundEst.setAngleThreshold(M_PI/4);
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    boundEst.compute(boundary);
    
    for(int i = 0; i < cloud->size(); ++i) {
        uint8_t tmp = (boundary.points[i].boundary_point);
        int isBoundary = static_cast<int>(tmp);
        if(isBoundary == 1) {
            boundaryPoints->push_back(cloud->points[i]);
        }
    }
    if(boundaryPoints->size() == 0) {
        std::cout << "No boundary points found!" << std::endl;
        return true;
    }
    std::string boundaryPath = rootDir+"boundary"+std::to_string(idx)+".pcd";
    std::cout << "Boundary estimation succeed!" << std::endl;
    std::cout << "Got " << boundaryPoints->size() <<" boundary points!" << std::endl;
    pcl::io::savePCDFileBinary(boundaryPath, *boundaryPoints);
    std::cout << "Save boundary points to '" << boundaryPath << "'!" << std::endl;
    return true;
}

void cloudViewer(std::string cloudPath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(cloudPath, *cloud) == -1) {
        PCL_ERROR("Could not read file!\n");
        return;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    viewer->addPointCloud(cloud);
    while(!viewer->wasStopped()) {
        viewer->spinOnce (1000);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
}

void showLineInCloud(std::string cloudPath, std::string linePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile(cloudPath, *cloud) == -1) {
        PCL_ERROR("Could not read file!\n");
        return;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    viewer->addPointCloud(cloud);

    ifstream fp(linePath);
    std::string line;
    std::vector< std::vector<float> > linePara;
    while(getline(fp, line)) {
        std::vector<float> lineData;
        std::vector<std::string> lineDataStr = split(line, " ");
        for(int i=0;i<lineDataStr.size();++i) {
            lineData.push_back(std::stof(lineDataStr[i]));
        }
        linePara.push_back(lineData);
    }
    int counter = 0;
    std::string nextLine = "C";
    std::cout << "Continue or not: ";
    int idx;
    int step;
    while(!viewer->wasStopped()){
        viewer->spinOnce(1000);
        while(nextLine == "C") {
            std::cin >> nextLine;
            std::cin >> idx;
            std::cin >> step;
            pcl::PointXYZ point1(linePara[idx][3]+linePara[idx][0]*counter*step, 
                                 linePara[idx][4]+linePara[idx][1]*counter*step, 
                                 linePara[idx][5]+linePara[idx][2]*counter*10);
            pcl::PointXYZ point2(linePara[idx][3]+linePara[idx][0]*(++counter)*step, 
                                 linePara[idx][4]+linePara[idx][1]*(++counter)*step, 
                                 linePara[idx][5]+linePara[idx][2]*(++counter)*step);
            viewer->addLine(point1, point2, std::to_string(counter));
            viewer->spinOnce(1000);
        }
    }
}
