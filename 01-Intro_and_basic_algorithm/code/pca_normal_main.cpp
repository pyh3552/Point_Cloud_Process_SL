//
// Created by pyh on 2022/7/26.
//
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include "pca_normal.h"
using namespace std;
int main(void){
    //-----------加载点云-------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/doc/bunny.pcd", *cloud) == -1){
//        PCL_ERROR("Could not read file\n");
        cerr << "Error!!!" << endl;
    }

    //-----------计算法向量-------------------
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>); //申请一段内存，用于存放带有normal和xyz的点云
        //建立kdtree进行近邻点的搜索
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pca_normal pca;
    pca.setInputCloud(cloud);
    pca.setKSearch(20);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudOut( new pcl::PointCloud<pcl::PointNormal> );
    pca.cal_nor(*cloudOut);


    //-----------存储点云到pcd文件----------------
    pcl::io::savePCDFile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/doc/bunny-nor.pcd",*cloudOut);
}