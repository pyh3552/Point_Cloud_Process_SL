//
// Created by pyh on 2022/8/12.
//
#include "RANSAC.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
using namespace std;
int main(){
    // -----------------------------------加载点云数据------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/narrow_scene_plane.pcd", *cloud);//读取PCD文件
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    auto mx_in = cloud->getMatrixXfMap(3,8,0);
    Eigen::MatrixXd mx1;
    mx1 = mx_in.transpose().cast<double>();
    auto * mx_ptr = &mx1;
    //-----------------------------------RANSAC------------------------------
    ransac_p rs_p(mx_ptr, 0.1, 300);
    cout << "ransac constructed" << endl;
    rs_p.cmp_normal(mx_ptr, 0.1);
    cout << "normal computed" << endl;
    rs_p.fit(mx_ptr);
    cout << "fit over" << endl;
    vector<size_t> final_inlier_index = rs_p.get_inlier_idx();
    //====================================着色=========================================
    ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/ransac.txt");
    for (int i = 0; i < final_inlier_index.size(); ++i) {
        cloud->points[final_inlier_index[i]].r = (uint8_t) 255;
        cloud->points[final_inlier_index[i]].g = (uint8_t) 0;
        cloud->points[final_inlier_index[i]].b = (uint8_t) 0;
        outfile << final_inlier_index[i] << endl;
    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/narrow_scene_plane-ransac.pcd", *cloud);
    return 0;
}
