//
// Created by pyh on 2022/8/18.
//
#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include "result_set.h"
#include "octree.h"
#ifndef POINT_CLOUD_PROCESS_SL_FPFH_H
#define POINT_CLOUD_PROCESS_SL_FPFH_H
class FPFH_descriptor{
private:
        Eigen::MatrixXd* const point_cloud_;                //输入的点云数据
        int B_;                                             //每个直方图的bin数

        std::vector<RadiusNNResultSet> all_rnn_result_;     //保存每个点的近邻点信息
        std::vector<size_t> feature_index_;                 //特征点对应的索引
        std::vector<size_t> descriptor;                     //描述子
        Eigen::MatrixXd normal;                             //点云法向量
        Eigen::MatrixXd FPFH_all;                           //按列存储所有特征点的FPFH

    //计算SPFH
    Eigen::Matrix<double, Eigen::Dynamic, 1> SFPH(size_t query_point_idx){
        cout << "SFPH start" << endl;
        //得到query point的法向量
        Eigen::Matrix<double, 1, 3> normal_query;
        normal_query = normal.row(query_point_idx);

        //将query point之外的近邻点存入其中
        Eigen::Matrix<double, Eigen::Dynamic, 3> query_neighbors;
        query_neighbors.resize(all_rnn_result_[query_point_idx].size()-1, 3);
        for (int i = 1; i < all_rnn_result_[query_point_idx].size(); ++i) {
            query_neighbors.row(i-1) = point_cloud_->row(all_rnn_result_[query_point_idx].dist_index()[i].index);
        }

        //首先构建reference frame
        //计算每个点的（p2-p1）/|| p2-p1 ||
        Eigen::Matrix<double, Eigen::Dynamic, 3> diff;
        diff.resize(all_rnn_result_[query_point_idx].size()-1, 3);
        diff = query_neighbors.rowwise() - point_cloud_->row(query_point_idx);
        diff = diff.array().colwise() / diff.rowwise().norm().array();
        cout << "diff caled" << endl;
        //u = n1
        Eigen::Matrix<double, 1, 3> u = normal_query;
        //v = u X diff
        Eigen::Matrix<double, Eigen::Dynamic, 3> v;
        v.resize(all_rnn_result_[query_point_idx].size()-1, 3);
        v = diff.rowwise().cross(u);
        //w = u X v
        Eigen::Matrix<double, Eigen::Dynamic, 3> w;
        w.resize(all_rnn_result_[query_point_idx].size()-1, 3);
        w = v.rowwise().cross(u);
        cout << "axis caled" << endl;
        //将query point之外的近邻点的法向量n2存入其中
        Eigen::Matrix<double, Eigen::Dynamic, 3> query_neighbors_normal;
        query_neighbors_normal.resize(all_rnn_result_[query_point_idx].size()-1, 3);
        for (int i = 1; i < all_rnn_result_[query_point_idx].size(); ++i) {
            query_neighbors_normal.row(i-1) = normal.row(all_rnn_result_[query_point_idx].dist_index()[i].index);
        }

        //计算alpha = v 点乘 n2
        Eigen::Matrix<double, Eigen::Dynamic, 1> alpha;
        alpha.resize(all_rnn_result_[query_point_idx].size()-1, 1);
        alpha = (v.array() * query_neighbors_normal.array()).rowwise().sum();
        cout << "alpha caled" << endl;
        //计算phi = u 点乘 diff
        Eigen::Matrix<double, Eigen::Dynamic, 1> phi;
        phi.resize(all_rnn_result_[query_point_idx].size()-1, 1);
        phi = (diff.array().rowwise() * u.array()).rowwise().sum();
        cout << "phi caled" << endl;
        //计算theta = arctan( w 点乘 n2， u 点乘 n2)
        Eigen::Matrix<double, Eigen::Dynamic, 1> theta;
        theta.resize(all_rnn_result_[query_point_idx].size()-1, 1);
        theta = atan((w.array() * query_neighbors_normal.array()).rowwise().sum() / (query_neighbors_normal.array().rowwise() * u.array()).rowwise().sum());
        cout << "theta caled" << endl;
        //计算直方图并拼接为描述子spfh
        Eigen::Matrix<double, Eigen::Dynamic, 1> spfh;
        spfh.resize(B_*3, 1);
        Eigen::Matrix<double, Eigen::Dynamic, 1> histogram_alpha;
        histogram_alpha.resize(B_, 1);
        histogram_alpha = CalHistgram(alpha, -1.0, 1.0);
        Eigen::Matrix<double, Eigen::Dynamic, 1> histogram_phi;
        histogram_phi.resize(B_, 1);
        histogram_phi = CalHistgram(phi, -1.0, 1.0);
        Eigen::Matrix<double, Eigen::Dynamic, 1> histogram_theta;
        histogram_theta.resize(B_, 1);
        histogram_theta = CalHistgram(theta, -3.14, 3.14);
        spfh << histogram_alpha,
                histogram_phi,
                histogram_theta;
        cout << "spfh caled" << endl;
        return spfh;
    }




    //计算直方图
    Eigen::Matrix<double, Eigen::Dynamic, 1> CalHistgram(const Eigen::Matrix<double, Eigen::Dynamic, 1> mat, double range_min, double range_max) {
        cout << "CalHistgram" << endl;
        //要输出的直方图
        Eigen::Matrix<double, Eigen::Dynamic, 1> histogram;
        histogram.resize(B_, 1);
        histogram.setZero();
        //数据步长
        double range  = range_max - range_min;
        double step = range / B_;
        //数据起始点
        double begin_value = range_min;
        //遍历mat中的每一个数据，判断它处于哪个区间
        for (int i = 0; i < mat.rows(); ++i) {
            int k = 0;
            while (begin_value < range_max){
                if (mat(i,0) >= begin_value && mat(i,0) < begin_value+step){
                    histogram(k, 0)++;
                    break;
                }
                k++;
                begin_value = begin_value + step;
            }
        }
        cout << "CalHistgram finished" << endl;
        return histogram;
    }

public:
    //构造函数
    FPFH_descriptor(Eigen::MatrixXd* const point_cloud, int B, vector<std::size_t> feature_index):point_cloud_(point_cloud){
        B_ = B;
        feature_index_ = feature_index;
    }

    //计算法向量，存储近邻点集
    void cmp_normal(Eigen::MatrixXd* const data, double search_radius){
        normal.resize(data->rows(), data->cols());
        normal.setOnes();
        // -----------------------------------创建Octree-------------------------------------
        auto *OcTree = new octant;
        octree_construction(OcTree, *data, 1, 0.000001);
        cout << "octree constructed" << endl;

        // ----------------------------------计算法向量-----------------------------------------
        for (int i = 0; i < data->rows(); ++i) {
            //搜索点
            Eigen::Matrix<double, 1, Eigen::Dynamic> cur_point = data->row(i);
            //结果数据集
            RadiusNNResultSet RNN_result_set(search_radius);
            //没有被拜访过的点p，找到它半径r内点所有邻居
            octree_radius_search(OcTree, *data, RNN_result_set, cur_point);
            cout << "search finished" << endl;
            //将当前点的搜索结果保存起来
            all_rnn_result_.push_back(RNN_result_set);
            //如果当前点的近邻点不足3个，则认为它无法算出准确的法向量，将其法向量值设为0，其bad_index值设为true
            if ( RNN_result_set.size() < 3 ){
                normal.row(i).setZero();
            }
            else{
                //取出近邻点的坐标，存入矩阵中
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> neighbors;
                neighbors.resize(RNN_result_set.size(), data->cols());
                for (int j = 0; j < RNN_result_set.size(); ++j) {
                    neighbors.row(j) = data->row(RNN_result_set.dist_index()[j].index);
                }
                //计算近邻点们的中心点
                Eigen::Matrix<double, 1, Eigen::Dynamic> neighbors_means = neighbors.colwise().mean();
                //去中心化
                Eigen::MatrixXd neighbors_normalized = neighbors.rowwise() - neighbors_means;
                //PCA-计算特征向量
                Eigen::MatrixXd H = neighbors_normalized.transpose() * neighbors_normalized;
                Eigen::EigenSolver<Eigen::MatrixXd> es(H);
                Eigen::MatrixXd Vec = es.pseudoEigenvectors();
                Eigen::Matrix<double, Eigen::Dynamic, 1> Val = es.eigenvalues().real();
                //找到最小特征值的索引并获得相应的特征向量。
                Eigen::MatrixXd::Index minRow, minCol;
                Val.minCoeff(&minRow,&minCol);
                //保存第i个点的法向量
                normal.row(i) = Vec.col(minRow).transpose();
            }

        }
    }


    //计算FPFH
    void describe(){
        Eigen::Matrix<double, 1, 3> key_point;
        FPFH_all.resize(3*B_, feature_index_.size());
        //遍历所有的特征点 key points
        for (int i = 0; i < feature_index_.size(); ++i) {
            key_point = point_cloud_->row(feature_index_[i]);
            //建立一个矩阵，用于存储key point的近邻点的坐标
            Eigen::Matrix<double, Eigen::Dynamic, 3> keypoint_neighbors;
            keypoint_neighbors.resize(all_rnn_result_[feature_index_[i]].size()-1, 3);

            //写入坐标，跳过关键点自身
            for (int j = 1; j < all_rnn_result_[feature_index_[i]].size(); ++j) {
                keypoint_neighbors.row(j-1) = point_cloud_->row(all_rnn_result_[feature_index_[i]].dist_index()[j].index);
            }
            cout << "position written" << endl;

            //计算权重
            Eigen::MatrixXd w;
            w = ( (keypoint_neighbors.rowwise() - key_point).rowwise().norm() ).cwiseInverse().asDiagonal();
            cout << "w caled" << endl;

            //计算出自身之外的近邻点的SPFH
            Eigen::MatrixXd neighbors_SPFH;
            neighbors_SPFH.resize(3*B_, all_rnn_result_[feature_index_[i]].size()-1);
            for (int j = 1; j < all_rnn_result_[feature_index_[i]].size(); ++j) {
                    neighbors_SPFH.col(j-1) = SFPH(all_rnn_result_[feature_index_[i]].dist_index()[j].index);
            }
            cout << "neighbors SPFH caled" << endl;
            //计算自身——当前关键点的SPFH
            Eigen::MatrixXd keypoint_SPFH;
            keypoint_SPFH.resize(3*B_, 1);
            keypoint_SPFH = SFPH(feature_index_[i]);
            cout << "keypoint SPFH caled" << endl;
            //计算最终的FPFH
            Eigen::MatrixXd final_FPFH;
            final_FPFH.resize(3*B_, 1);
            final_FPFH = keypoint_SPFH + (neighbors_SPFH * w).rowwise().sum() * (1/(all_rnn_result_[feature_index_[i]].size()-1));
            cout << "final FPFH caled" << endl;
            //归一化
            final_FPFH = final_FPFH * (1 / final_FPFH.norm());
            cout << "normalized FPFH caled" << endl;

            //保存final_FPFH
            FPFH_all.col(i) = final_FPFH;
            cout << "FPFH_all caled" << endl;
        }
    }

    //获取FPFH描述子
    Eigen::MatrixXd get_descriptors(){
        return FPFH_all;
    }
};
#endif //POINT_CLOUD_PROCESS_SL_FPFH_H
