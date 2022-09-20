//
// Created by pyh on 2022/8/18.
//
#include "FPFH.h"
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
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/07-Feature-Detection/doc/bunny.pcd",
                *cloud);//读取PCD文件
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    auto mx_in = cloud->getMatrixXfMap(3, 8, 0);
    Eigen::MatrixXd mx1;
    mx1 = mx_in.transpose().cast<double>();
    auto *mx_ptr = &mx1;
    //--------------------------------------构建ISS检测器----------------------------------------
    ISS_detector iss(mx_ptr, 0.05, 0.83, 0.83, 1, 1);
    vector<std::size_t> feature_point_index;   //最后输出的特征点的索引
    feature_point_index = iss.detect();
    //---------------------------------------计算FPFH描述子---------------------------------------
    FPFH_descriptor fpfh(mx_ptr, 11, feature_point_index);
    fpfh.cmp_normal(mx_ptr, 0.05);
    cout << "cmp_normal finished" << endl;
    cout << "describe begin" << endl;
    fpfh.describe();
    cout << "describe finished" << endl;
    ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/08-Feature-Description/doc/descriptor.txt");
    outfile << "feature_point_index.size() = " << feature_point_index.size() << endl;
    outfile << fpfh.get_descriptors() << endl;
    //====================================着色=========================================
    for (int i = 0; i < feature_point_index.size(); ++i) {
        cloud->points[feature_point_index[i]].r = (uint8_t) 255;
        cloud->points[feature_point_index[i]].g = (uint8_t) 0;
        cloud->points[feature_point_index[i]].b = (uint8_t) 0;
    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/07-Feature-Detection/doc/bunny-ISS.pcd", *cloud);
    return 0;
}