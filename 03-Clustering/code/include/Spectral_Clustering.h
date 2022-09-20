//
// Created by pyh on 2022/8/7.
//
#include "Kmeans-K.h"
#include "octree.h"
#include "result_set.h"
#include "numeric"
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <fstream>
#ifndef POINT_CLOUD_PROCESS_SL_SPECTRAL_CLUSTERING_H
#define POINT_CLOUD_PROCESS_SL_SPECTRAL_CLUSTERING_H
class spectral_clustering{
private:
    int n_clusters_;                                            //聚类中心数量，可以自己选定也可以通过SVD结果选定
    int max_iter_;                                              //K-Means最大迭代次数
    double epsilon_;                                            //K-Means容差阈值

    size_t n_points_;                                           //数据点点数
//    std::vector<Eigen::Matrix<double, 1, 3> > eigen_Kvec;        //拉普拉斯矩阵最小的K个特征向量
    Eigen::MatrixXd eigen_Kvec;                                 //拉普拉斯矩阵最小的K个特征向量

    Eigen::MatrixXd W_;                                         //图的相似性矩阵
    Eigen::MatrixXd D_;                                         //图的度量矩阵（Degree Matrix）
    Eigen::MatrixXd L_;                                         //图的未归一化的拉普拉斯矩阵
    Eigen::MatrixXd L_n;                                        //图的归一化的拉普拉斯矩阵

    //计算相似矩阵W
    void cal_weight_mat(const Eigen::MatrixXd * data, int n_neighbors, double sigma){
        //如果一共有n个数据点，那么先构建相似矩阵W为一个n*n的全为0的矩阵
        W_.resize(data->rows(), data->rows());
        W_ = Eigen::MatrixXd::Zero(data->rows(), data->rows());
        // -----------------------------------创建Octree-------------------------------------
        auto *OcTree = new octant;
        octree_construction(OcTree, *data, 1, 0.000001);
        //---------------构建搜索点并展开搜索，用搜索结果构建相似度矩阵------------------------------
        for (int i = 0; i < data->rows(); ++i) {
            //搜索点
            Eigen::Matrix<double,1,3> search_point = data->row(i);
            //结果数据集
            KnnResultSet result_set(n_neighbors);
            //查找
            octree_knn_search(OcTree, *data, result_set, search_point);
            //修改第i个点和它最近邻之间的关系即edge，为避免节点和自己出现edge，wii=0
            //由于本Octree中的第0项是节点自身，故从第1项开始
            for (int j = 1; j < n_neighbors; ++j) {
                W_(i,result_set.dist_index_list[j].index) = exp( (-0.5) * pow(result_set.dist_index_list[j].distance, 2) * (1 / pow(sigma, 2)) );
            }
        }
    }

    //计算度量矩阵
    void cal_degree_matrix(){
        D_ = W_.rowwise().sum().asDiagonal();
    }

public:
    spectral_clustering(const Eigen::MatrixXd * data, int n_clusters, int n_neighbors = 50, double epsilon = 1e-4, double sigma = 10, int max_iter = 50){
        n_clusters_ = n_clusters;                               //初始化聚类数
        max_iter_   = max_iter;                                 //初始化K-Means最大迭代次数
        epsilon_    = epsilon;                                  //初始化K-Means容差阈值
        n_points_   = data->rows();                             //初始化点数
        cal_weight_mat(data, n_neighbors,sigma);                      //计算相似矩阵W_
        cal_degree_matrix();                                    //计算度量矩阵D_
        L_          = D_ - W_;                                  //计算未归一化的拉普拉斯矩阵
        L_n         = D_.inverse() * L_;                        //计算归一化的拉普拉斯矩阵

    }


    //计算特征值
    void fit(){
        ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/eigen.txt");
        //求L_n的特征值和特征向量
        Eigen::EigenSolver<Eigen::MatrixXd> es(L_n);
        Eigen::MatrixXd eigenVec = es.pseudoEigenvectors();
        Eigen::Matrix<double, Eigen::Dynamic, 1> eigenVal = es.eigenvalues().real();
        outfile << "eigenVec\n" << eigenVec << endl;
        outfile << "eigenval\n" << eigenVal << endl;
        //将特征值放入vector中，得到特征值升序排列后的原索引
        std::vector<double> eigenval_;
        for (int i = 0; i < eigenVal.rows(); ++i) {
            eigenval_.push_back(eigenVal(i,0));
        }
        std::vector<std::size_t> idx_sorted_eigenVal(eigenval_.size());
        std::iota(std::begin(idx_sorted_eigenVal), std::end(idx_sorted_eigenVal), 0);
        std::sort( idx_sorted_eigenVal.begin(), idx_sorted_eigenVal.end(), [&eigenval_](size_t i1, size_t i2) { return eigenval_[i1] < eigenval_[i2];} );

        //选取对应最小的k个特征值的特征向量,构成新的矩阵
        eigen_Kvec.resize(eigenVec.rows(), n_clusters_);
        for (int i = 0; i < n_clusters_; ++i) {
            //原eigenVec是中的特征向量是按列排布的。
            eigen_Kvec.col(i) = eigenVec.col(idx_sorted_eigenVal[i]);
        }

    }

    //对最小的k个特征值的特征向量的矩阵进行K-means，得到类
    Eigen::MatrixXd km_on_vec(){
        // -----------------------------------构建K-means-------------------------------------
        K_means_K k_means(n_clusters_,epsilon_,max_iter_);
        Eigen::MatrixXd min_distance_index = k_means.fit(eigen_Kvec);
        return min_distance_index;
    }

    void print_info(){
        ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/03-Clustering/doc/data.txt", ios_base::app);
        outfile << "W_\n" << W_;
        outfile << "D_\n" << D_;
        outfile << "L_\n" << L_;
        outfile << "eigen_Kvec\n" << eigen_Kvec;
    }
};
#endif //POINT_CLOUD_PROCESS_SL_SPECTRAL_CLUSTERING_H
