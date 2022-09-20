//
// Created by pyh on 2022/8/3.
//
#include <vector>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#ifndef POINT_CLOUD_PROCESS_SL_KMEANS_K_H
#define POINT_CLOUD_PROCESS_SL_KMEANS_K_H
class K_means_K{
public:
    int k_;                                              //k_为分组数
    double tolerance_;                                   //tolerance为中心点误差
    int max_iter_;                                       //max_iter为迭代次数
    Eigen::MatrixXd min_dist_index;                      //n*1的矩阵，存储每个点所属聚类的索引
    std::vector<Eigen::Matrix<double,1,Eigen::Dynamic> > centers;     //centers为一个vector，存储聚类中心的坐标
    K_means_K(int n_clusters = 2,double tolerance = 0.0001, int max_iter = 300){
        k_ = n_clusters;
        tolerance_ = tolerance;
        max_iter_ = max_iter;
    }

    Eigen::MatrixXd fit(Eigen::MatrixXd data){
        size_t num_points = data.rows();
        //随机选取k个点云中的点作为初始的聚类中心点；
        srand(time(0));
        for (int i = 0; i < k_; ++i) {
            centers.push_back(data.row(rand()%(data.rows())));         //将k个中心点存入centers中
        }

        //开始循环聚类
        int num_iter = 0;                                   //迭代次数
        bool tolerance_achieve = false;                     //是否已经到达误差的容差
        Eigen::MatrixXd center_dist;                        //保存每个点到每个聚类中心的距离。
        center_dist.resize(data.rows(), k_);                //有多少点就有多少行，有多少聚类中心就有多少列
        Eigen::MatrixXd cluster_assiment;                   //有多少点就有多少行，
        cluster_assiment.resize(data.rows(), 2);        //第一列为每个点对应的聚类中心的索引，第二列为对应的到聚类中心的距离

        //开始迭代
        while (num_iter < max_iter_ && (! tolerance_achieve)){
            //遍历每个聚类中心
            for (int center_index = 0; center_index < k_; ++center_index) {
                for (int i = 0; i < data.rows(); ++i) {
//                    diff = data.row(i) - centers[i];                                              //每个点的坐标值减去聚类中心
                    center_dist(i,center_index) = (data.row(i) - centers[center_index]).norm();     //保存每个点到每个聚类中心的距离
                }
            }

            //获取每个点所属聚类的索引
            Eigen::MatrixXd::Index minRow;
            Eigen::MatrixXd::Index minCol;
            min_dist_index.resize(data.rows(), 1);
            for (int i = 0; i < data.rows(); ++i) {
                center_dist.row(i).minCoeff(&minRow,&minCol);
                min_dist_index(i,0) = size_t(minCol);
            }

            //获取每个点到各个聚类中心的距离中最小的那一个
            //也就是得到了点到所属聚类中心的距离
            Eigen::MatrixXd min_dist;
            min_dist.resize(data.rows(), 1);
            min_dist = center_dist.rowwise().minCoeff();

            //保存每个点对应的聚类中心和对应的距离
            for (int point_index = 0; point_index < data.rows(); ++point_index) {
                cluster_assiment(point_index, 0) = min_dist_index(point_index, 0);
                cluster_assiment(point_index, 1) = min_dist(point_index, 0);
            }

            tolerance_achieve = true;

            for (int center_index = 0; center_index < k_; ++center_index) {
                //取出属于对应聚类中心的数据点
                std::vector<Eigen::Matrix<double,1, Eigen::Dynamic> >point_in_k_cluster;
                for (int i = 0; i < data.rows(); ++i) {
                    if (int(cluster_assiment(i,0)) == center_index){
                        point_in_k_cluster.push_back(data.row(i));
                    }
                }
                //如果数据点的个数大于0
                //按照分配好的数据点重新计算各自的聚类中心
                if (point_in_k_cluster.size() > 0){
                    //计算出数据点的均值
                    Eigen::MatrixXd sum;
                    sum.resize(point_in_k_cluster.size(),data.cols());
                    for (int i = 0; i < point_in_k_cluster.size(); ++i) {
                        sum.row(i) = point_in_k_cluster[i];
                    }
                    Eigen::Matrix<double, 1, Eigen::Dynamic> new_mean = sum.colwise().mean();
                    //判断是否聚类中心的移动距离大于给定的阈值，大于则是有效移动
                    if ((centers[center_index] - new_mean).norm() > tolerance_){
                        tolerance_achieve = false;
                    }
                    centers[center_index] = new_mean;
                }
            }
            num_iter += 1;
        }
        return min_dist_index;
    }
};
#endif //POINT_CLOUD_PROCESS_SL_KMEANS_K_H
