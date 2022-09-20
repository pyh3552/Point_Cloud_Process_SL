//
// Created by pyh on 2022/8/17.
//
#include <numeric>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include "result_set.h"
#include "octree.h"
#ifndef POINT_CLOUD_PROCESS_SL_ISS_H
#define POINT_CLOUD_PROCESS_SL_ISS_H
class Index_lambda3{
private:
    size_t index_;
    double lambda3_;
public:
    Index_lambda3(size_t index, double lambda3){
        index_ = index;
        lambda3_ = lambda3;
    }

    size_t get_index(){
        return index_;
    }

    double get_lambda3(){
        return lambda3_;
    }
};
class ISS_detector{
private:
    Eigen::MatrixXd* const  point_cloud_;       //输入的点云数据
    double radius_;                             //邻域搜索半径
    double gamma_32_;                           //确保特征值之间差距的阈值
    double gamma_21_;                           //确保特征值之间差距的阈值
    double lambda3_min_;                        //非极大值抑制时，确保最小的特征值要大于该阈值
    int k_min_;                                 //设定最小邻域的阈值
    double l3_min_;                             //设定最小特征值的阈值

    vector<RadiusNNResultSet> all_RNN_Result;  //保存每个点的近邻点
    vector<double> lambda3;                     //用于非极大化抑制，存储各个点的最小特征值
    vector<Eigen::Matrix<double, Eigen::Dynamic, 1> > lambda_all;                     //存储每个点的协方差矩阵的特征值
    vector<std::size_t> idx_lambda3;            //用于非极大化抑制，将最小特征值降序排列后其对应点索引的顺序
    vector<std::size_t> suppressed_;            //保存被抑制的点的集合
    vector<std::size_t> feature_point_index_;   //最后输出的特征点的索引

    //对vector排序并获得索引
    template <typename T>
    std::vector<size_t> sort_indexes(const std::vector<T> &v) {
        // 初始化索引向量
        std::vector<size_t> idx(v.size());
        //使用iota对向量赋0~？的连续值
        std::iota(idx.begin(), idx.end(), 0);
        // 通过比较v的值对索引idx进行排序
        std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] > v[i2]; });
        return idx;
    }


    //获取query point在邻域中的加权协方差矩阵
    Eigen::MatrixXd get_cov_matrix(Eigen::Matrix<double, 1, Eigen::Dynamic> const query_point, RadiusNNResultSet *radius_result_set){
        //获取近邻点，并将其构建为矩阵
        Eigen::MatrixXd neighbors;
        neighbors.resize(radius_result_set->size(), point_cloud_->cols());
        for (int i = 0; i < radius_result_set->size(); ++i) {
            neighbors.row(i) = point_cloud_->row(radius_result_set->dist_index()[i].index);
        }
        //近邻点到query point的坐标差值
        Eigen::MatrixXd distance;
        distance = neighbors.rowwise() - query_point;
        //计算每个近邻点的权重放入相应行
        Eigen::Matrix<double, Eigen::Dynamic, 1> weight;
        weight = distance.rowwise().norm();
        //获得协方差矩阵将权重对角化，用于后面和矩阵相乘
        Eigen::MatrixXd cov;
        cov = 1.0 / weight.sum() * (distance.transpose() * weight.asDiagonal() * distance );
        return cov;
    }

    //求协方差矩阵的特征值，并将其降序排列
    Eigen::Matrix<double, 3, 1> sorted_eig_cal(Eigen::MatrixXd cov){
        Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
        Eigen::Matrix<double, Eigen::Dynamic, 1> Val = es.eigenvalues().real();
        vector<double> eig_value;
        for (int i = 0; i < Val.rows(); ++i) {
            eig_value.push_back(Val(i,0));
        }
        sort(eig_value.rbegin(), eig_value.rend());
        Eigen::Matrix<double, Eigen::Dynamic, 1> eig_value_sorted;
        eig_value_sorted.resize(Val.rows(),1);
        for (int j = 0; j < Val.rows(); ++j) {
            eig_value_sorted(j, 0) = eig_value[j];
        }
        return eig_value_sorted;
    }

    //非极大化抑制
    void non_maximum_suppression(){
        for (int i = 0; i < point_cloud_->rows(); ++i) {
            size_t idx_center = idx_lambda3[i];
            //判断当前点是否已经被抑制
            if (find(suppressed_.begin(),suppressed_.end(), idx_lambda3[i]) == suppressed_.end()){
                //将有最大lambda3的点的邻域点全列为被抑制点
                for (int j = 1; j < all_RNN_Result[idx_center].size(); ++j) {
                    suppressed_.push_back(all_RNN_Result[idx_center].dist_index()[j].index);
                }
            }
            else{
                continue;
            }
        }
    }

    vector<std::size_t> filter_by_param(){
        vector<std::size_t> feature_index = idx_lambda3;
        for (int i = 0; i < idx_lambda3.size(); ++i) {
            if (find(suppressed_.begin(),suppressed_.end(), idx_lambda3[i]) != suppressed_.end()){
                //如果当前索引属于被抑制的点，那么就不算在特征点之中
                feature_index.erase(  remove(feature_index.begin(),feature_index.end(), idx_lambda3[i]), feature_index.end()  );
            }
            else{
                //如果当前索引不属于被抑制的点，判断其特征值是否合格
                if (!(lambda_all[i](0,0)*gamma_21_ > lambda_all[i](1,0) && lambda_all[i](1,0)*gamma_32_> lambda_all[i](2,0))){
                    //不合格，则从特征点索引中排除
                    feature_index.erase(  remove(feature_index.begin(),feature_index.end(), idx_lambda3[i]), feature_index.end()  );
                }
            }
        }
        return feature_index;
    }


public:
    //构造函数，初始化数据
    ISS_detector(Eigen::MatrixXd* const  point_cloud, double radius, double gamma_32, double gamma_21, double l3_min, int k_min)
    :point_cloud_(point_cloud)
    {

        radius_ = radius;
        gamma_32_ = gamma_32;
        gamma_21_ = gamma_21;
        l3_min_ = l3_min;
        k_min_ = k_min;
    }


    //检测特征点
    vector<std::size_t> detect(){
        Eigen::Matrix<double, 3, 1> sorted_eigVal;
        //被弃用的点的特征值统一划为0
        Eigen::Matrix<double, 3, 1> abandon_point_eigVal;
        abandon_point_eigVal.setZero();
        //协方差矩阵
        Eigen::MatrixXd cov;
        //一个用于存储最小特征值和对应点索引的类

        //构建八叉树
        auto *OcTree = new octant;
        octree_construction(OcTree, *point_cloud_, 1, 0.000001);
        //遍历每一个点
        for (int i = 0; i < point_cloud_->rows(); ++i) {
            //搜索点
            Eigen::Matrix<double, 1, Eigen::Dynamic> cur_point = point_cloud_->row(i);
            //结果数据集
            RadiusNNResultSet RNN_result_set(radius_);
            //搜索
            octree_radius_search(OcTree, *point_cloud_, RNN_result_set, cur_point);
//            cout << "search finished" << endl;
            //将该店的搜索结果保存起来
            all_RNN_Result.push_back(RNN_result_set);
//            cout << "result saved" << endl;
            //排除近邻点个数小于给定阈值的点，由于这些点弃用了，将他们的特征值直接划为0.并直接进入下一个点的判定
            if (RNN_result_set.size() < k_min_){

                lambda_all.push_back(abandon_point_eigVal);
                lambda3.push_back(0);
                continue;
            }
            //计算有计算价值的点的协方差矩阵
//            cout << "cov cal begins" << endl;
            cov = get_cov_matrix(point_cloud_->row(i),&RNN_result_set);
//            cout << "cov cal finished" << endl;
            //计算降序排列后的特征值
//            cout << "eigen cal begins" << endl;
            sorted_eigVal = sorted_eig_cal(cov);
//            cout << "eigen cal finished" << endl;
            lambda_all.push_back(sorted_eigVal);
            lambda3.push_back(sorted_eigVal(2, 0));
//            cout << "eigen cal result saved" << endl;
//            index_with_eigVal.push_back(sorted_eigVal);
        }
        //非极大值抑制
        idx_lambda3 = sort_indexes(lambda3);
//        cout << "non_maximum_suppression begin" << endl;
        non_maximum_suppression();
//        cout << "non_maximum_suppression finished" << endl;
//        cout << "filter_by_param begin" << endl;
        feature_point_index_ = filter_by_param();
//        cout << "filter_by_param finished" << endl;
        return feature_point_index_;
    }
};
#endif //POINT_CLOUD_PROCESS_SL_ISS_H
