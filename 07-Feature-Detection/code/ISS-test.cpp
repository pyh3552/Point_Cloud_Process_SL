//
// Created by pyh on 2022/8/17.
//
#include "ISS.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
using namespace std;
int main() {
    // -----------------------------------加载点云数据------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/07-Feature-Detection/doc/Armadillo-5k.pcd",
                *cloud);//读取PCD文件
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    auto mx_in = cloud->getMatrixXfMap(3, 8, 0);
    Eigen::MatrixXd mx1;
    mx1 = mx_in.transpose().cast<double>();
    auto *mx_ptr = &mx1;
    //--------------------------------------构建ISS检测器----------------------------------------
    ISS_detector iss(mx_ptr, 10, 0.83, 0.83, 1, 1);
    vector<std::size_t> feature_point_index;   //最后输出的特征点的索引
    feature_point_index = iss.detect();
    //====================================着色=========================================
    for (int i = 0; i < feature_point_index.size(); ++i) {
        cloud->points[feature_point_index[i]].r = (uint8_t) 255;
        cloud->points[feature_point_index[i]].g = (uint8_t) 0;
        cloud->points[feature_point_index[i]].b = (uint8_t) 0;
    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/07-Feature-Detection/doc/Armadillo-5k-ISS.pcd", *cloud);
    return 0;
}