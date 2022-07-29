//
// Created by pyh on 2022/7/27.
//
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include "voxel_filter.h"
using namespace std;
int main(void){
    //-----------加载点云-------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/doc/scene.pcd", *cloud) == -1){
//        PCL_ERROR("Could not read file\n");
        cerr << "Error!!!" << endl;
    }

    //-----------voxel down-sample-------------------
    voxel_filter vf;
    vf.setInputCloud(cloud);
    vf.setLeafSize(0.01, 0.01, 0.01);
    vf.setRandom(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut( new pcl::PointCloud<pcl::PointXYZ> );
    vf.voxel_fil(*cloudOut);

    //-----------存储点云到pcd文件----------------
    pcl::io::savePCDFile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/doc/scene-vox.pcd",*cloudOut);
}
