//
// Created by pyh on 2022/8/3.
//
#include <vector>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#ifndef POINT_CLOUD_PROCESS_SL_KMEANS_H
#define POINT_CLOUD_PROCESS_SL_KMEANS_H
class K_means{
public:
    //k_为分组数，tolerance为中心点误差，max_iter为迭代次数
    int k_;
    double tolerance_;
    int max_iter_;
    Eigen::MatrixXd min_dist_index;
    //centers为一个vector，存储聚类中心的坐标
    std::vector<Eigen::Matrix<double,1,3> > centers;
    K_means(int n_clusters = 2,double tolerance = 0.0001, int max_iter = 300){
        k_ = n_clusters;
        tolerance_ = tolerance;
        max_iter_ = max_iter;
    }

    Eigen::MatrixXd fit(Eigen::MatrixXd data){
        size_t num_points = data.rows();
        //随机选取k个点云中的点作为初始的聚类中心点；
        srand(time(0));
//        std::vector<size_t> random_num;
        for (int i = 0; i < k_; ++i) {
//            random_num.push_back(rand() % (num_points));       //将选中的点的索引存入vector random_num中
            centers.push_back(data.row(rand()%(data.rows())));         //将k个中心点存入centers中
        }

        //开始循环聚类
        int num_iter = 0;                                   //迭代次数
        bool tolerance_achieve = false;                     //是否已经到达误差的容差
        Eigen::MatrixXd center_dist;                        //保存每个点到每个聚类中心的距离。
        center_dist.resize(data.rows(), k_);                //有多少点就有多少行，有多少聚类中心就有多少列
        Eigen::MatrixXd cluster_assiment;                   //有多少点就有多少行，第一列为每个点对应的聚类中心的索引 第二列为对应的到聚类中心的距离
        cluster_assiment.resize(data.rows(), 2);        //

        //开始迭代
        while (num_iter < max_iter_ && (! tolerance_achieve)){
            //遍历每个聚类中心
//            std::vector<Eigen::Matrix<double,1,3> > diff;
//            Eigen::Matrix<double,1,3> diff;
            for (int center_index = 0; center_index < k_; ++center_index) {
                for (int i = 0; i < data.rows(); ++i) {
//                    diff = data.row(i) - centers[i];                                   //每个点的坐标值减去聚类中心
                    center_dist(i,center_index) = (data.row(i) - centers[center_index]).norm();     //保存每个点到每个聚类中心的距离
//                    std::cout<<"center_dist_fit:"<<std::endl;
//                    std::cout << center_dist << std::endl;
                }
            }

            //获取每个点对应聚类中心的索引
            Eigen::Matrix<double,1, 8>::Index minRow;
            Eigen::Matrix<double,1, 8>::Index minCol;
//            Eigen::MatrixXd min_dist_index;
            min_dist_index.resize(data.rows(), 1);
            for (int i = 0; i < data.rows(); ++i) {
                center_dist.row(i).minCoeff(&minRow,&minCol);
                min_dist_index(i,0) = size_t(minCol);
            }

            //获取每个点到聚类中心的最小距离
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
//                Eigen::MatrixXd point_in_k_cluster;
                std::vector<Eigen::Matrix<double,1,3> >point_in_k_cluster;
                for (int i = 0; i < data.rows(); ++i) {
                    if (int(cluster_assiment(i,0)) == center_index){
                        point_in_k_cluster.push_back(data.row(i));
                    }
                }
                //如果数据点的个数大于0
                if (point_in_k_cluster.size() > 0){
                    //计算出数据点的均值
                    Eigen::MatrixXd sum;
                    sum.resize(point_in_k_cluster.size(),3);
                    for (int i = 0; i < point_in_k_cluster.size(); ++i) {
                        sum.row(i) = point_in_k_cluster[i];
                    }
                    Eigen::Matrix<double, 1, 3> new_mean = sum.colwise().mean();
                    //判断是否聚类中心的移动距离大于给定的阈值
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

    Eigen::MatrixXd predict(Eigen::MatrixXd processed_data){
        //用于存放每个点到每个聚类中心的距离
        Eigen::MatrixXd center_dist;
        center_dist.resize(processed_data.rows(), k_);
        //遍历每个聚类中心
        for (int center_index = 0; center_index < k_; ++center_index) {
            //计算点到对应聚类中心的距离
            for (int i = 0; i < processed_data.rows(); ++i) {
//              diff = data.row(i) - centers[i];                                   //每个点的坐标值减去聚类中心
                center_dist(i, center_index) = (processed_data.row(i) - centers[center_index]).norm();     //保存每个点到每个聚类中心的距离
            }
        }
//        std::cout << "center_dist:" << center_dist << std::endl;
        //获取每个点对应聚类中心的索引
        Eigen::Matrix<double,1, 8>::Index minRow;
        Eigen::Matrix<double,1, 8>::Index minCol;
        Eigen::MatrixXd min_dist_index;
        min_dist_index.resize(processed_data.rows(), 1);
        for (int i = 0; i < processed_data.rows(); ++i) {
            center_dist.row(i).minCoeff(&minRow,&minCol);
            min_dist_index(i,0) = size_t(minCol);
        }
//        std::cout << "min_dist_index\n" << min_dist_index << std::endl;
        return min_dist_index;
    }

};
#endif //POINT_CLOUD_PROCESS_SL_KMEANS_H
