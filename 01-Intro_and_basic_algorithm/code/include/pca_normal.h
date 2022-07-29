//
// Created by pyh on 2022/7/26.
//
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#ifndef POINT_CLOUD_PROCESS_SL_PCA_NORMAL_H
#define POINT_CLOUD_PROCESS_SL_PCA_NORMAL_H

using PointT = pcl::PointXYZ;//方便修改类型？

class pca_normal{
public:
    //设定输入点云
    bool setInputCloud(const pcl::PointCloud<PointT>::Ptr & input_cloud);
    //K近邻搜索
    inline void setKSearch( int k ) { _k = k; };
    //半径搜索
    inline void setRadiusSearch( double radius ) { _search_radius = radius; }
    //计算法向量
//    inline void pca_nor();

    void cal_nor(pcl::PointCloud<pcl::PointNormal> & m_output);

    //功能：点云转换为矩阵
    void PointConversionEigen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::MatrixXd &cloudMat);

private:
    pcl::PointCloud<PointT>::Ptr m_input;

    int _k = 0;
    float _search_radius = 0.0;
};
#endif //POINT_CLOUD_PROCESS_SL_PCA_NORMAL_H
