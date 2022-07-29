//
// Created by pyh on 2022/7/29.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d_omp.h>

//计算当前点的权重
inline double kernel(double d, double sigma){
    //高斯函数公式 f(x) = a * exp ( -(x-b)^2 / 2 * c^2 )
    return (std::exp( (- (d * d)) / (2 * sigma * sigma) ));
}

//计算法向量
pcl::PointCloud<pcl::Normal>::Ptr  computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud(target_cloud);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.compute(*normals);

    return normals;
}


int main(int argc, char* argv[]){
    std::string incloudfile;
    std::string outcloudfile;
    std::cout<<"请键入输入点云文件名。"<<std::endl;
    std::cin >> incloudfile;
    std::cout<<"请键入输出点云文件名。"<<std::endl;
    std::cin >> outcloudfile;
    //-------------------------------------载入点云--------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(incloudfile.c_str(),*cloud);

    //-------------------------------------设置参数阈值-----------------------------------
    float sigma_s = 0.5;   //sigma_s是测点pi到其邻域点pj的距离对该店的影响因子，是一个高斯核函数的标准差
    float sigma_r = 10;     //sigma_r是测点pi到其邻域点pj的距离向量在该点pi法向量ni上的投影对pi的影响因子，是另一个高斯核函数的标准差
    pcl::PointCloud<pcl::PointXYZ>::Ptr BFcloud(new pcl::PointCloud<pcl::PointXYZ>);
    BFcloud = cloud;

    //-------------------------------------建立KD树索引-----------------------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    pcl::Indices k_indices;             //用以存放某点的邻域点的索引
    std::vector<float> k_distances;     //用以存放某点到邻域点的距离的平方

    //-------------------------------------基于法向量的双边滤波-----------------------------

    //计算输入点云的法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals_input = computeNormal(cloud);

    for (int point_id = 0; point_id < cloud->size(); ++point_id){
        float BF = 0;
        float W  = 0;

        tree->radiusSearch(point_id, 2 * sigma_s, k_indices, k_distances);
        Eigen::Vector3f normal = (*normals_input)[point_id].getNormalVector3fMap();
        //遍历每一个邻域点
        for(std::size_t n_id = 0; n_id < k_indices.size(); ++n_id){
            //对于vector使用at，和[]差不多，但是at会做边界检查，越界会抛出异常。https://blog.csdn.net/qq_40692109/article/details/104294572?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-104294572-blog-115533286.pc_relevant_multi_platform_whitelistv3&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-104294572-blog-115533286.pc_relevant_multi_platform_whitelistv3&utm_relevant_index=1
            int id = k_indices.at(n_id);                //id为第n_id个邻域点的索引
            float dist = sqrt(k_distances.at(n_id));    //dist为第n_id个邻域点到测点的距离

            Eigen::Vector3f point_p = cloud->points[point_id].getVector3fMap();         //测点p的坐标位置
            Eigen::Vector3f point_q = cloud->points[k_indices[n_id]].getVector3fMap();  //测点p的第n_id个邻域点的坐标位置
            float normal_dist = normal.dot(point_q - point_p);                          //计算测点p到邻域点q的距离向量在p的法向量方向上的投影。例如，p和q在一条线上，投影就应当几乎为0。

            //计算高斯核函数
            float w_a = kernel(dist, sigma_s);              //第n_id个邻域点的距离核函数的值
            float w_b = kernel(normal_dist, sigma_r);       //第n_id个邻域点的法向量投影距离核函数的值
            float weight = w_a * w_b;

            BF += weight * normal_dist;         //似乎是偏移量？
            W  += weight;                       //测点的所有邻域点的权重的加和？
        }

        // 滤波后的点就是将测点沿法向量移动一定的距离
        Eigen::Vector3f point_filtered = cloud->points[point_id].getVector3fMap() + ((BF / W) * normal);
        BFcloud->points[point_id].x = point_filtered[0];
        BFcloud->points[point_id].y = point_filtered[1];
        BFcloud->points[point_id].z = point_filtered[2];
    }

    //存储滤波后的点云
    pcl::io::savePCDFile(outcloudfile.c_str(), *BFcloud);

    return 0;

}
