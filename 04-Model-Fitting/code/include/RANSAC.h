//
// Created by pyh on 2022/8/12.
//
//平面RANSAC
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include "octree.h"
#include "result_set.h"
#ifndef POINT_CLOUD_PROCESS_SL_RANSAC_H
#define POINT_CLOUD_PROCESS_SL_RANSAC_H
class ransac_p{
public:
    ransac_p(const Eigen::MatrixXd * data, double distance_threshold = 0.5, int num_iter = 50){
        distance_threshold_ = distance_threshold;
        num_iter_ = num_iter;
        inlier_cloud_.resize(1, data->cols());
        segmented_cloud_ = *data;
    }

    void cmp_normal(const Eigen::MatrixXd * data, double search_radius){
        normal.resize(data->rows(), data->cols());
        normal.setOnes();
        bad_index.resize(data->rows());
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
            //如果当前点的近邻点不足3个，则认为它无法算出准确的法向量，将其法向量值设为0，其bad_index值设为true
            if ( RNN_result_set.size() < 3 ){
                normal.row(i).setZero();
                bad_index[i] = true;
            }
            else{
                //取出近邻点的坐标，存入矩阵中
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> neighbors;
                neighbors.resize(RNN_result_set.size(), data->cols());
                for (int j = 0; j < RNN_result_set.size(); ++j) {
                    neighbors.row(j) = data->row(RNN_result_set.dist_index_list[j].index);
                }
                //计算近邻点们的中心点
                Eigen::Matrix<double, 1, Eigen::Dynamic> neighbors_means = neighbors.colwise().mean();
                //归一化
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

    void fit(const Eigen::MatrixXd * data) {
        //srand不得调用>=2次
        srand((int)time(0));
        for (int i = 0; i < num_iter_; ++i) {
            ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/ransac-point.txt");
            //随机选取三个点
            vector<Eigen::Matrix<double, 1, 3> > p;
            while (p.size() < 3) {
                int index = rand() % data->rows();
                outfile << index << "   ";
                if (bad_index[index] == false) {
                    p.push_back(data->row(index));
                }
            }
            outfile << endl;
            //通过叉乘较大来确保三个点不在同一直线上,如果叉乘很小，三点几乎一直线，则用随机的另外一个点，随机替换掉一个点。
            while (((p[0] - p[1]).cross(p[0] - p[2])).norm() < 0.01) {
                p[rand() % 3] = data->row(rand() % data->rows());
            }

            outfile << p[0] << endl;
            outfile << p[1] << endl;
            outfile << p[2] << endl;
            outfile << endl;
            outfile << endl;
            //选点结束

            //开始拟合平面ax+by+cz+d=0
            a = (p[0] - p[1]).cross(p[0] - p[2])(0, 0);
            b = (p[0] - p[1]).cross(p[0] - p[2])(0, 1);
            c = (p[0] - p[1]).cross(p[0] - p[2])(0, 2);
            //d=0 - (ax+by+cz)
            d = 0 - (a * p[0](0, 0) + b * p[0](0, 1) + c * p[0](0, 2));

            //遍历剩余的点，区分inlier和outlier
            inlier_idx.clear();
            inlier.clear();
            Eigen::Matrix<double, 1, Eigen::Dynamic> point;
            point.resize(1, data->cols());
            for (int idx = 0; idx < data->rows(); ++idx) {
                point = data->row(idx);
                double point_distance =
                        abs(a * point(0, 0) + b * point(0, 1) + c * point(0, 2) + d) / sqrt(a * a + b * b + c * c);
                if (point_distance < distance_threshold_ && !bad_index[idx]) {
                    inlier.push_back(point);
                    inlier_idx.push_back(idx);
                }
            }
            //这一轮运算中得到的inlier数大于原来的数量，则替换掉原来的inlier_cloud平面
            if (inlier.size() > final_inlier_idx.size()){
                final_inlier_idx.clear();
//                inlier_cloud_.resize(inlier.size(), data->cols());
                for (int j = 0; j < inlier.size(); ++j) {
                    final_inlier_idx.push_back(inlier_idx[j]);
//                    inlier_cloud_.row(j) = inlier[j];
//                    RemoveRow(segmented_cloud_, inlier_idx[j]);
                }
            }
            //本次拟合中的内点比例
            cout << "num of inlier" << inlier.size() << endl;
            cout << "num of inlier_idx" << inlier_idx.size() << endl;
            cout << "num of final_inlier_idx" << final_inlier_idx.size() << endl;
//            cout << "本次拟合中的内点比例: " << inlier.size()/data->rows() << endl;
        }
    }

    vector<size_t> get_inlier_idx(){
        return final_inlier_idx;
    }
private:
    double distance_threshold_;                              //距离阈值tau，根据检验选择
    int ransac_n_ = 3;                                       //RANSAC参数点数目，对于平面来说，平面模型:ax+by+cz+d=0
    int num_iter_;                                           //RANSAC迭代次数
    Eigen::MatrixXd inlier_cloud_;                           //内点的点云
    Eigen::MatrixXd segmented_cloud_;                        //内点之外的点云
    Eigen::MatrixXd normal;                                  //点云法向量
    vector<bool> bad_index;                                  //判断一个点搜索半径范围内的近邻点数是否超过3个，没超过则表示是一个坏点。
    double a,b,c,d;                                          //平面模型:ax+by+cz+d=0
    double inline_precent_;                                   //一轮拟合中inlier占整体的百分比，用于提前结束拟合。
    vector<Eigen::MatrixXd> inlier;                          //inlier包含的点的坐标
    vector<std::size_t> inlier_idx;                          //inlier包含的点在data中的索引
    vector<std::size_t> final_inlier_idx;                    //有最多inlier，inlier包含的点在data中的索引
    void RemoveRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove) {
        unsigned int numRows = matrix.rows() - 1;
        unsigned int numCols = matrix.cols();

        if( rowToRemove < numRows ) {
            matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) =
                    matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);
        }

        matrix.conservativeResize(numRows,numCols);
    }
};
#endif //POINT_CLOUD_PROCESS_SL_RANSAC_H
