//
// Created by pyh on 2022/8/3.
//
#include "Kmeans-K.h"
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
using namespace std;
//int main(){
////    cout << "hello" << endl;
//    Eigen::Matrix<double, 6, 3> mx;
////    cout << "hello" << endl;
//    mx << 1,   2,   0,
//         1.5, 1.8, 0,
//         5,   8,   0,
//         8,   8,   0,
//         1,   0.6, 0,
//         9,   11,  0;
////    mx << 1,2,0,1.5,1.8,0,5,8,0,8,8,0,1,0.6,0,9,11,0;
//    cout << "mx: \n" << mx << endl;
//    K_means k_means(2);
//    cout << "class over" << endl;
//    k_means.fit(mx);
//    cout << "fit over" << endl;
//    cout<< k_means.predict(mx) << endl;
//    return 0;
//}

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
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/bunny.pcd", *cloud);//读取PCD文件
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    Eigen::MatrixXd mx_in;
    PointConversionEigen(cloud, mx_in);
    // -----------------------------------构建K-means-------------------------------------
    K_means_K k_means_k(4);
    Eigen::MatrixXd min_distance_index = k_means_k.fit(mx_in);
    //====================================着色=========================================

    for (int i = 0; i < mx_in.rows(); ++i) {
        if (min_distance_index(i,0) == 0){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 255;
        }
        if (min_distance_index(i,0) == 1){
            cloud->points[i].r = (uint8_t) 0;
            cloud->points[i].g = (uint8_t) 255;
            cloud->points[i].b = (uint8_t) 0;
        }
        if (min_distance_index(i,0) == 2){
            cloud->points[i].r = (uint8_t) 0;
            cloud->points[i].g = (uint8_t) 0;
            cloud->points[i].b = (uint8_t) 0;
        }
        if (min_distance_index(i,0) == 3){
            cloud->points[i].r = (uint8_t) 255;
            cloud->points[i].g = (uint8_t) 0;
            cloud->points[i].b = (uint8_t) 0;
        }

    }
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/bunny-4-Kmeans-K.pcd", *cloud);


    return 0;
}