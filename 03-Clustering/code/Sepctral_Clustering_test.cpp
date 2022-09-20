//
// Created by pyh on 2022/8/7.
//
#include "Kmeans.h"
#include "Spectral_Clustering.h"
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
int main()
{
    // -----------------------------------加载点云数据------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/narrow_scene.pcd", *cloud);//读取PCD文件
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    Eigen::MatrixXd mx_in;
    PointConversionEigen(cloud, mx_in);
    Eigen::MatrixXd *mx = &mx_in;
    cout << "mx in:\n" << *mx << endl;
    //-----------------------------------构建Spectral Clustering------------------------------
    spectral_clustering sc(mx, 4);
    //计算要用的k个特征向量
    sc.fit();
    //对{yi}进行K-means
    Eigen::Matrix<double , Eigen::Dynamic, 1>  y_index = sc.km_on_vec();
    sc.print_info();
    //====================================着色=========================================

    for (int i = 0; i < mx_in.rows(); ++i) {
//        cout << "y_index[" << i << "]: " << y_index(i,0) << endl;
        if (y_index(i,0) == -1){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 255;
        }
        if (y_index(i,0) == 0){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 255;
        }
        if (y_index(i,0) == 1){
            cloud->points[i].r = (uint8_t) 0;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 255;
        }
        if (y_index(i,0) == 2){
            cloud->points[i].r = (uint8_t) 0;
            cloud->points[i].g = (uint8_t) 0;
            cloud->points[i].b = (uint8_t) 0;
        }
        if (y_index(i,0) == 3){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 0;
            cloud->points[i].b = (uint8_t) 0;
        }

    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/narrow_scene-4-sc.pcd", *cloud);

    return 0;
}
