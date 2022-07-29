#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h> // 生成高斯分布的点云
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/doc/bunny.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file \n");
        return (-1);
    }

    // 设置XYZ各纬度的均值和标准差
    float xmean = 0, ymean = 0, zmean = 0;
    float xstddev = 0.002, ystddev = 0.002, zstddev = 0.002;
    // ---------------------------生成高斯分布的点云数据---------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr gauss_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::NormalGenerator<float> > generator;
    uint32_t seed = static_cast<uint32_t> (time(NULL));
    pcl::common::NormalGenerator<float>::Parameters x_params(xmean, xstddev, seed++);
    generator.setParametersForX(x_params);
    pcl::common::NormalGenerator<float>::Parameters y_params(ymean, ystddev, seed++);
    generator.setParametersForY(y_params);
    pcl::common::NormalGenerator<float>::Parameters z_params(zmean, zstddev, seed++);
    generator.setParametersForZ(z_params);
    generator.fill((*cloud).width, (*cloud).height, *gauss_cloud);
    // ---------------------------添加高斯分布的随机噪声--------------------------------------
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        gauss_cloud->points[i].x += cloud->points[i].x;
        gauss_cloud->points[i].y += cloud->points[i].y;
        gauss_cloud->points[i].z += cloud->points[i].z;
    }
    printf("高斯噪声添加完毕！！！");
    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/doc/bunny-noise.pcd", *gauss_cloud);


    return 0;
}

