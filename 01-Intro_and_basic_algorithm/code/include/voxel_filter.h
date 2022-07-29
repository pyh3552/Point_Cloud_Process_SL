//
// Created by pyh on 2022/7/27.
//
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#ifndef POINT_CLOUD_PROCESS_SL_VOXEL_FILTER_H
#define POINT_CLOUD_PROCESS_SL_VOXEL_FILTER_H
using PointT = pcl::PointXYZ;
class voxel_filter{
public:
    //设定输入点云
    bool setInputCloud(const pcl::PointCloud<PointT>::Ptr & input_cloud);
    //功能：点云转换为矩阵
    void PointConversionEigen(pcl::PointCloud<PointT>::Ptr cloud, Eigen::MatrixXd &cloudMat);
    //设置最小体素边长
    void setLeafSize(float _leaf_size_x, float _leaf_size_y, float _leaf_size_z) {
        leaf_size_x = _leaf_size_x;
        leaf_size_y = _leaf_size_y;
        leaf_size_z = _leaf_size_z;
    }
    //体素滤波函数
    void voxel_fil(pcl::PointCloud<PointT> & m_output);
    //是否random选取点
    void setRandom( int mode ) { _mode = mode; };

private:
    pcl::PointCloud<PointT>::Ptr m_input;
    float leaf_size_x, leaf_size_y, leaf_size_z;
    int _mode;

};



//对vector进行排序并返回排序前的下标
template <typename T>
std::vector<size_t> argsort(const std::vector<T> &v)
{
    // 建立下标数组
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);
    // 调用sort函数，匿名函数自动捕获待排序数组
    sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
}
#endif //POINT_CLOUD_PROCESS_SL_VOXEL_FILTER_H
