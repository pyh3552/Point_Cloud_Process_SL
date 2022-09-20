//
// Created by pyh on 2022/8/22.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h> // icp算法
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // 利用控制台计算时间

using namespace std;

int
main(int argc, char** argv)
{
    pcl::console::TicToc time;
    // --------------------加载源点云-----------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny.pcd", *source);

    cout << "从源点云中读取 " << source->size() << " 个点" << endl;

    // -------------------加载目标点云----------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny-moved.pcd", *target);

    cout << "从目标点云中读取 " << target->size() << " 个点" << endl;

    time.tic();
    //--------------------初始化ICP对象--------------------
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    //----------------------icp核心代码--------------------
    icp.setInputSource(source);            // 源点云
    icp.setInputTarget(target);            // 目标点云
    icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
    icp.setMaxCorrespondenceDistance(1);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setEuclideanFitnessEpsilon(0.001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setMaximumIterations(35);           // 最大迭代次数
    icp.setUseReciprocalCorrespondences(true);//设置为true,则使用相互对应关系
    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    icp.align(*icp_cloud);
    cout << "Applied " << 35 << " ICP iterations in " << time.toc() << " ms" << endl;
    cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
    cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
    // 使用创建的变换对为输入源点云进行变换
    pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());
    //====================================着色icp后的点=========================================
    for (int i = 0; i < icp_cloud->size(); ++i) {
        icp_cloud->points[i].r = (uint8_t) 255;
        icp_cloud->points[i].g = (uint8_t) 0;
        icp_cloud->points[i].b = (uint8_t) 0;
    }
    pcl::io::savePCDFileASCII ("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny-icp.pcd", *icp_cloud);


    return (0);
}

