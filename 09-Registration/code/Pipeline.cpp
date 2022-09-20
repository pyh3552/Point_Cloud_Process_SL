//
// Created by pyh on 2022/8/21.
//

#include "FPFH.h"
#include "ISS.h"
#include "RANSAC-Registration.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
using namespace std;

//根据给定的索引从点云中提取出一部分点云
Eigen::MatrixXd extract_points(Eigen::MatrixXd* data, vector<std::size_t> feature_points_index){
    Eigen::MatrixXd features;
    features.resize(feature_points_index.size(), 3);
    for (int i = 0; i < feature_points_index.size(); ++i) {
        features.row(i) = data->row(feature_points_index[i]);
    }
    return features;
}

int main() {
    // -----------------------------------加载点云数据------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny.pcd",*source_cloud);//读取PCD文件
    cout << "源点云数据中一共有: " << source_cloud->points.size() << "个点。" << endl;
    reader.read("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny-moved.pcd",*target_cloud);//读取PCD文件
    cout << "目标点云数据中一共有: " << target_cloud->points.size() << "个点。" << endl;
    //-----------------------------------将源点云转化为一个矩阵-------------------------------
    auto mx_in1 = source_cloud->getMatrixXfMap(3, 8, 0);
    Eigen::MatrixXd mx_source;
    mx_source = mx_in1.transpose().cast<double>();
    auto *source_ptr = &mx_source;
    //-----------------------------------将目标点云转化为一个矩阵-------------------------------
    auto mx_in2 = target_cloud->getMatrixXfMap(3, 8, 0);
    Eigen::MatrixXd mx_target;
    mx_target = mx_in2.transpose().cast<double>();
    auto *target_ptr = &mx_target;
    //--------------------------------------构建ISS检测器，检测源点云的特征点--------------------
    ISS_detector iss_source(source_ptr, 0.05, 0.83, 0.83, 1, 1);
    vector<std::size_t> feature_point_index_source;   //最后输出的特征点的索引
    feature_point_index_source = iss_source.detect();
    //--------------------------------------构建ISS检测器，检测目标点云的特征点--------------------
    ISS_detector iss_target(target_ptr, 0.05, 0.83, 0.83, 1, 1);
    vector<std::size_t> feature_point_index_target;   //最后输出的特征点的索引
    feature_point_index_target = iss_target.detect();
    //---------------------------------------计算源点云的特征点的FPFH描述子---------------------------------------
    FPFH_descriptor fpfh_source(source_ptr, 11, feature_point_index_source);
    fpfh_source.cmp_normal(source_ptr, 0.05);
    cout << "cmp_normal finished" << endl;
    cout << "describe begin" << endl;
    fpfh_source.describe();
    cout << "describe finished" << endl;
    ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/source-descriptor.txt", ios_base::app);
    outfile << "feature_point_index.size() = " << feature_point_index_source.size() << endl;
    outfile << fpfh_source.get_descriptors() << endl;
    //---------------------------------------计算目标点云的特征点的FPFH描述子---------------------------------------
    FPFH_descriptor fpfh_target(target_ptr, 11, feature_point_index_target);
    fpfh_target.cmp_normal(target_ptr, 0.05);
    cout << "cmp_normal finished" << endl;
    cout << "describe begin" << endl;
    fpfh_target.describe();
    cout << "describe finished" << endl;
    ofstream outfile2("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/target-descriptor.txt", ios_base::app);
    outfile2 << "feature_point_index.size() = " << feature_point_index_target.size() << endl;
    outfile2 << fpfh_target.get_descriptors() << endl;
    //====================================着色源点云中的特征点=========================================
    for (int i = 0; i < feature_point_index_source.size(); ++i) {
        source_cloud->points[feature_point_index_source[i]].r = (uint8_t) 255;
        source_cloud->points[feature_point_index_source[i]].g = (uint8_t) 0;
        source_cloud->points[feature_point_index_source[i]].b = (uint8_t) 0;
    }
    //====================================着色目标点云中的特征点=========================================
    for (int i = 0; i < feature_point_index_target.size(); ++i) {
        target_cloud->points[feature_point_index_target[i]].r = (uint8_t) 0;
        target_cloud->points[feature_point_index_target[i]].g = (uint8_t) 255;
        target_cloud->points[feature_point_index_target[i]].b = (uint8_t) 0;
    }
    //-------------------------------------构建RANSAC_Build-------------------------------------------
    RANSAC_build ransac_build(source_ptr, target_ptr);
    cout << "RANSAC built" << endl;
    //-------------------------------------构建RANSAC_ICP---------------------------------------------
    RANSAC_ICP ransac_icp(ransac_build);
    cout << "racsac_icp initialized" << endl;
    ransac_icp.set_source_points(source_ptr);
    cout << "source set" << endl;
    ransac_icp.set_target_points(*target_ptr);
    cout << "target set" << endl;
    //输入fpfh描述子,由于描述子按列存储，故需要进行转置
    ransac_icp.set_source_fpfh(fpfh_source.get_descriptors().transpose());
    cout << "source fpfh set" << endl;
    ransac_icp.set_target_fpfh(fpfh_target.get_descriptors().transpose());
    cout << "target fpfh set" << endl;
    //RANSAC得到内点最多的变换
    ransac_icp.ransac_match();
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 1, 3> t;

    R = ransac_icp.get_Trans_final().block(0,0,3,3);
    t = ransac_icp.get_Trans_final().block(3,0,1,3);
    Eigen::MatrixXd source_transformed = ( R * source_ptr->transpose() ).transpose().rowwise() + t;
    ofstream outfile3("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/Rt.txt");
    outfile3 << "R : \n" << R << endl;
    outfile3 << "t : \n" << t << endl;
    outfile3 << "source : \n" << source_ptr->transpose() << endl;
    outfile3 << "Rotated : \n" << ( R * source_ptr->transpose() ).transpose() << endl;
    //------------------------------------从矩阵中写入数据到点云---------------------------------------------
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = source_transformed.rows();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    for (int i = 0; i < source_transformed.rows(); ++i)
    {
        cloud.points[i].x = source_transformed(i, 0);
        cloud.points[i].y = source_transformed(i, 1);
        cloud.points[i].z = source_transformed(i, 2);
    }

    //保存PCD文件操作
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny_transformed.pcd", cloud);

    // -----------------------------------保存点云---------------------------------------------
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny-ISS.pcd", *source_cloud);
    pcl::io::savePCDFileASCII("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/bunny-moved-ISS.pcd", *target_cloud);
    return 0;
}
