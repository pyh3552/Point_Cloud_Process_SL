//
// Created by pyh on 2022/8/9.
//
#include "dbscan.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
using namespace std;
//功能：点云转换为矩阵
void PointConversionEigen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::MatrixXd &cloudMat)
{
    cloudMat.resize(cloud->points.size(), 3);    //定义点云的行数，3列
    for (int itr = 0; itr < cloud->points.size(); itr++)
    {
        cloudMat(itr, 0) = cloud->points[itr].x;
        cloudMat(itr, 1) = cloud->points[itr].y;
        cloudMat(itr, 2) = cloud->points[itr].z;
    }
}
int main(){
    // -----------------------------------加载点云数据------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/narrow_scene-1k.pcd", *cloud);//读取PCD文件
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    Eigen::MatrixXd mx_in;
    PointConversionEigen(cloud, mx_in);
    Eigen::MatrixXd *mx = &mx_in;
    cout << "mx in:\n" << *mx << endl;
    Eigen::MatrixXd mx2;
    mx2.resize(17, 3);
    mx2 << 1,2,3,
           1,2,5,
           2,3,5,
           6,7,4,
           4,5,6,
           3,6,2,
           7,4,2,
           6,7,8,
           8,7,9,
           6,7,5,
           5,2,5,
            111,240,663,
            114,241,662,
            124,242,665,
            134,244,665,
            112,242,665,
             1000,1222,3300;

    //-----------------------------------构建DBSCAN------------------------------
    dbscan ds(mx, 0.45,4);
    cout << "1" << endl;
    ds.fit_core(mx);
    cout << "1" << endl;
    ds.fit_cluster();
    cout << "1" << endl;
    //====================================着色=========================================
    ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/idx.txt");
    for (int i = 0; i < mx->rows(); ++i) {
        outfile << "cluster_index[" << i << "]: " << ds.get_cluster_index()(i,0) << endl;
        cloud->points[i].r = (uint8_t) ((ds.get_cluster_index()(i,0)/ds.get_cluster_num())*255);
        cloud->points[i].g = (uint8_t) ((ds.get_cluster_index()(i,0)/ds.get_cluster_num())*255);
        cloud->points[i].b = (uint8_t) ((ds.get_cluster_index()(i,0)/ds.get_cluster_num())*255);


    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/narrow_scene-1k-ds.pcd", *cloud);
    return 0;
}
