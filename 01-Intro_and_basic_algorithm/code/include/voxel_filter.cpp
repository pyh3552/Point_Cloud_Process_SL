//
// Created by pyh on 2022/7/27.
//
#include "voxel_filter.h"
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
//设定输入点云
bool voxel_filter::setInputCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud) {
    m_input = input_cloud;
    return true;
}

//功能：点云转换为矩阵
void voxel_filter::PointConversionEigen(pcl::PointCloud<PointT>::Ptr cloud, Eigen::MatrixXd &cloudMat)
{
    cloudMat.resize(cloud->points.size(), 3);    //定义点云的行数，3列
    for (int itr = 0; itr < cloud->points.size(); itr++)
    {
        cloudMat(itr, 0) = cloud->points[itr].x;
        cloudMat(itr, 1) = cloud->points[itr].y;
        cloudMat(itr, 2) = cloud->points[itr].z;
    }
}


//voxel_filter算法实现
void voxel_filter::voxel_fil(pcl::PointCloud<PointT> &m_output) {
    //将输入点云转化为一个矩阵
    Eigen::MatrixXd mx_in;
    PointConversionEigen(m_input, mx_in);

    //分离三个轴的数据
    Eigen::MatrixXd x_array = mx_in.col(0);
    Eigen::MatrixXd y_array = mx_in.col(1);
    Eigen::MatrixXd z_array = mx_in.col(2);

    //找到三个轴各自的最大值和最小值
    double x_max = x_array.maxCoeff();
    double x_min = x_array.minCoeff();
    double y_max = y_array.maxCoeff();
    double y_min = y_array.minCoeff();
    double z_max = z_array.maxCoeff();
    double z_min = z_array.minCoeff();

    //计算网格数目
    int D_x = ceil( (x_max - x_min) / leaf_size_x );
    int D_y = ceil( (y_max - y_min) / leaf_size_y );
    int D_z = ceil( (z_max - z_min) / leaf_size_z );

    //计算每个点落在了哪个格子
    std::vector<int> h_vec;

    for( int idx = 0; idx < m_input->size(); ++idx){
        Eigen::Matrix<double, 1, 3> cur_point = mx_in.row(idx);
        int h_x = floor( (cur_point(0, 0) - x_min) /  leaf_size_x );
        int h_y = floor( (cur_point(0, 1) - y_min) /  leaf_size_y );
        int h_z = floor( (cur_point(0, 2) - z_min) /  leaf_size_z );
        //计算得到current Point所在格子的坐标值后，计算它的hash值。
        int h = h_x + h_y*D_x + h_z*D_x*D_y;

        h_vec.push_back(h);
    }



    //对h_vec进行排序,并将排序前的索引按相应顺序保存在vector里面
    std::vector<size_t> sorted_idx = argsort(h_vec);
    sort(h_vec.begin(), h_vec.end());
    //将代表点云的矩阵按照隶属格子的次序重新排列。
    Eigen::MatrixXd mx_sorted;
    mx_sorted.resize(mx_in.rows(),3);
    for(int ix = 0; ix < sorted_idx.size(); ++ix){

        mx_sorted(ix, 0) = 1;
        mx_sorted(ix, 1) = 2;
        mx_sorted(ix, 2) = 3;

        mx_sorted(ix, 0) = mx_in(sorted_idx[ix], 0);
        mx_sorted(ix, 1) = mx_in(sorted_idx[ix], 1);
        mx_sorted(ix, 2) = mx_in(sorted_idx[ix], 2);


    }


//    std::cout << h_vec[0] << std::endl;
//    std::cout << h_vec[1] << std::endl;
//    std::cout << h_vec[2] << std::endl;
//    std::cout << h_vec[3] << std::endl;
//    std::cout << h_vec[4] << std::endl;
//    std::cout << h_vec[5] << std::endl;
//    std::cout << h_vec[6] << std::endl;
//    std::cout << h_vec[7] << std::endl;
//    std::cout << h_vec[8] << std::endl;
//    std::cout << h_vec[9] << std::endl;

    //在每个网格中选取一个点
    Eigen::MatrixXd mx_filtered;
//    Eigen::MatrixXd voxel;
    for (int i = 0; i < mx_sorted.rows(); ++i){
        //eg. 0,0,0,3,3,3,3,.......,n,n
        //
        if(i < mx_sorted.rows() - 1 and h_vec[i] == h_vec[i+1]){
            continue;
        }
        else {
            int num = count(h_vec.begin(), h_vec.end(), h_vec[i]);
//            std::cout << "num:";
//            std::cout << num << std::endl;
//            voxel = mx_sorted.block(i - num + 1, 0, num, 3);

//            i = i + num;
            if (_mode == 0){
                //采用中心点作为代表
                PointT filtered_point;

//                mx_filtered.row(i - num + 1) =  (mx_sorted.block(i-num+1,0,long(num),3).colwise().mean();
                filtered_point.x  = (mx_sorted.block(i - num + 1, 0, num, 3)).colwise().mean()(0,0);
                filtered_point.y  = (mx_sorted.block(i - num + 1, 0, num, 3)).colwise().mean()(0,1);
                filtered_point.z  = (mx_sorted.block(i - num + 1, 0, num, 3)).colwise().mean()(0,2);
                m_output.push_back(filtered_point);


            }
        }
    }
}



