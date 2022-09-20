//
// Created by pyh on 2022/8/1.
//
#include "octree.h"
#include <iostream>
#include <vector>
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
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny.pcd", *cloud);//读取PCD文件
//    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny.pcd", *output);
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    Eigen::MatrixXd mx_in;
    PointConversionEigen(cloud, mx_in);
    // -----------------------------------创建Octree-------------------------------------
    auto *OcTree = new octant;
    octree_construction(OcTree, mx_in, 1, 0.000001);
    OcTree->display();
    OcTree->children[0]->display();
    OcTree->children[0]->children[0]->display();
    OcTree->children[1]->display();
    OcTree->children[2]->display();
    //------------------------------------目标点--------------------------------------------
    Eigen::Matrix<double,1,3> search_point = mx_in.row(1);
    cout << "search_point: " << search_point << endl;
//    RadiusNNResultSet radius_result_set(0.05);
    KnnResultSet result_set(20);
    //查找
    octree_knn_search(OcTree, mx_in, result_set, search_point);
//    octree_radius_search(OcTree, mx_in, radius_result_set, search_point);

    //====================================着色=========================================
//    cout << "count = " << radius_result_set.count << endl;
//    for (int i = 0; i < radius_result_set.count; ++i) {
//
//        radius_result_set.dist_index_list[i].dis_DistIndex();
//        cloud->points[radius_result_set.dist_index_list[i].index].r = (uint8_t) 255;
//        cloud->points[radius_result_set.dist_index_list[i].index].g = (uint8_t) 0;
//        cloud->points[radius_result_set.dist_index_list[i].index].b = (uint8_t) 0;
//    }
//    cloud->points[1].r = 255;
//    cloud->points[1].g = 255;
//    cloud->points[1].b = 255;
    cout << "count = " << result_set.count << endl;
    for (int i = 0; i < result_set.count; ++i) {

        result_set.dist_index_list[i].dis_DistIndex();
        cloud->points[result_set.dist_index_list[i].index].r = (uint8_t) 255;
        cloud->points[result_set.dist_index_list[i].index].g = (uint8_t) 0;
        cloud->points[result_set.dist_index_list[i].index].b = (uint8_t) 0;
    }
    cloud->points[1].r = 255;
    cloud->points[1].g = 255;
    cloud->points[1].b = 255;
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny-octree.pcd", *cloud);
    return 0;
}