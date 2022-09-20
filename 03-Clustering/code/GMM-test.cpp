//
// Created by pyh on 2022/8/6.
//
#include "GMM.h"
//#include <Eigen/Eigen>
//#include <Eigen/Eigenvalues>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/bunny-double.pcd", *cloud);//读取PCD文件
//    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/.pcd", *output);
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    Eigen::MatrixXd mx_in;
//    mx_in.resize(3,3);
//    mx_in << 1,2,3,
//             4,5,6,
//             11,56,90
//             ;
    PointConversionEigen(cloud, mx_in);
    // -----------------------------------构建GMM-------------------------------------
    GMM gmm(mx_in, 2);
    cout << "GMM_web construct successfully" << endl;
    vector<std::size_t> result;
    gmm.fit(result);
    cout << "fit successfully" << endl;

    cout << "result produced successfully" << endl;

    //====================================着色=========================================

    for (int i = 0; i < mx_in.rows(); ++i) {
        cout << "result[i]: " << result[i] << endl;
        if (result[i] == 0){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 255;
        }
        if (result[i] == 1){
            cloud->points[i].r = (uint8_t) 0;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 255;
        }
        if (result[i] == 2){
            cloud->points[i].r = (uint8_t) 0;
            cloud->points[i].g = (uint8_t) 0;
            cloud->points[i].b = (uint8_t) 0;
        }
        if (result[i] == 3){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 0;
            cloud->points[i].b = (uint8_t) 0;
        }

    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/bunny-double-2-gmm.pcd", *cloud);


    return 0;
}