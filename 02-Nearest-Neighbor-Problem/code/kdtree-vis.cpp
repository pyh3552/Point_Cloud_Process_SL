//
// Created by pyh on 2022/7/31.
//
#include <iostream>
#include <vector>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "kdtree.h"

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
int main(int argc, char** argv)
{
    // -----------------------------------加载点云数据------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny.pcd", *cloud);//读取PCD文件
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny.pcd", *output);
    cout << "点云数据中一共有: " << cloud->points.size() << "个点。" << endl;
    //-----------------------------------将输入点云转化为一个矩阵-------------------------------
    Eigen::MatrixXd mx_in;
    PointConversionEigen(cloud, mx_in);
    // -----------------------------------创建KD-tree-------------------------------------
    //赋值
    vector<vector<double> > train(mx_in.rows(), vector<double>(3, 0));
    for (unsigned i = 0; i < mx_in.rows(); ++i)
        for (unsigned j = 0; j < 3; ++j)
            train[i][j] = mx_in(i, j);
    //建树
    auto *kdTree = new kdtree;
    build_kd_tree(kdTree, train, 0);

    print_kd_tree(kdTree, 0);

    //-------------------------------------目标点-----------------------------------------
    Eigen::Matrix<double,3,1> search_point = mx_in.row(1);
    KnnResultSet resultset(20);
    //查找
    kdtree_knn_search(kdTree, mx_in, resultset, search_point);
    resultset.display();


    // -------------------------------------可视化----------------------------------------
    // 注：这里的可视化是创建可视化变量，并开辟一个窗口来展示图像。在后面的代码才会显示点云文件。
    /*for (int i = 0; i < mx_in.rows(); ++i) {
        output->points[i] = cloud->points[i];
        if(i = resultset.dist_index_list[i].index) {
            cloud->points[resultset.dist_index_list[i].index].r = (uint8_t) 255;
            cloud->points[resultset.dist_index_list[i].index].g = (uint8_t) 0;
            cloud->points[resultset.dist_index_list[i].index].b = (uint8_t) 0;
        }
    }*/
    for (int i = 0; i < resultset.count; ++i) {
            cloud->points[resultset.dist_index_list[i].index].r = (uint8_t) 255;
            cloud->points[resultset.dist_index_list[i].index].g = (uint8_t) 0;
            cloud->points[resultset.dist_index_list[i].index].b = (uint8_t) 0;
    }
    cloud->points[1].r = 255;
    cloud->points[1].g = 255;
    cloud->points[1].b = 255;
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny-kdcol2.pcd", *cloud);




    //-----------------------------PCL KDTREE----------------------------------
    //建立kdtree对象
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(output);
    std::vector<int> neighbor_index_k;//领域索引
    std::vector<float> neighbor_square_distance_k;//领域距离大小

    kdtree.nearestKSearch(output->points[1], 20, neighbor_index_k, neighbor_square_distance_k);
    for (int i = 0; i < neighbor_index_k.size(); ++i) {
        output->points[neighbor_index_k[i]].r = (uint8_t) 255;
        output->points[neighbor_index_k[i]].g = (uint8_t) 0;
        output->points[neighbor_index_k[i]].b = (uint8_t) 0;
    }
    output->points[1].r = 255;
    output->points[1].g = 255;
    output->points[1].b = 255;
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/02-Nearest-Neighbor-Problem/doc/bunny-kdcol-pcl.pcd", *output);

    return 0;
}


