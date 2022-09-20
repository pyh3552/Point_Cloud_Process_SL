//
// Created by pyh on 2022/8/6.
//
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
using namespace std;
#ifndef POINT_CLOUD_PROCESS_SL_GMM_H
#define POINT_CLOUD_PROCESS_SL_GMM_H
Eigen::Matrix<double, Eigen::Dynamic, 1> multivariate_normal_distribution(Eigen::MatrixXd data, Eigen::Matrix<double, 1, 3> Mu, Eigen::Matrix3d Var){
    Eigen::MatrixXd gaussian;
    gaussian.resize(data.rows(), 1);
    for (int i = 0; i < data.rows(); ++i) {
        gaussian(i,0) = (
        ( 1 / (pow(2*3.1415926535897, 0.5*data.cols()) * pow( Var.determinant() , 0.5)) )
        *
        exp( (-0.5) *  (data.row(i)-Mu) * Var.inverse() * (data.row(i)-Mu).transpose()   )
        );
    }
    //返回的是各个数据点在第k个聚类中的概率
//    cout << "gaussian:\n" << gaussian << endl;
    return gaussian;
}
class GMM{
public:
    double n_clusters_;             //聚类个数
    int max_iter_;                  //最大迭代次数
    Eigen::MatrixXd mu;             //每个类的均值
    //用动态矩阵出现内存泄漏，未解决。改变维度需要手动在此修改。
    vector<Eigen::Matrix<double, 3, 3> > var;    //每个类的协方差
    vector<double> pi;              //每个类的权重
    Eigen::MatrixXd W;              //p(Z_nk)，每个点属于一个类的概率
    Eigen::MatrixXd data_;          //数据点
    double n_points;                //点数
    int D;                          //维度数
    vector<double> loglh;           //损失函数MLE的数值
    bool over = false;
    GMM(Eigen::MatrixXd data, double n_clusters, int max_iter=50){
        n_clusters_ = n_clusters;
        max_iter_ = max_iter;
        n_points = data.rows();
        D = data.cols();
        data_ = data;
        mu.resize(n_clusters, D);
        //随机选取k个点
        srand(time(0));
        for (int i = 0; i < n_clusters; ++i) {
            //初始化均值
//            std::cout << "初始化均值1" << endl;
            mu.row(i) = data.row(rand()% size_t(n_points));
//            std::cout << "初始化均值2" << endl;
//            cout << mu.row(0) <<endl;
//            cout << mu.row(1) <<endl;
            //初始化协方差
//            std::cout << "初始化协方差1" << endl;
//            std::cout << D << std::endl;
//            std::cout << "初始化协方差1.5" << endl;
//            cout << Eigen::Matrix3d::Identity(3,3) << endl;
//            cout << (10 * Eigen::Matrix3d::Identity(3,3)) << endl;
            var.push_back(10 * Eigen::Matrix3d::Identity(3,3));
//            std::cout << "初始化协方差2" << endl;
            //初始化pi，初始时刻认为每个高斯分布的权值相等且和为1
//            std::cout << "初始化pi1" << endl;
            pi.push_back(1/n_clusters);
//            std::cout << "初始化pi2" << endl;
        }
        //初始化p(Z_nk)，初始时刻认为属于每个聚类的概率相等
        W.resize(n_points,n_clusters);
        W = Eigen::MatrixXd::Ones(n_points,n_clusters) / n_clusters;

    }

    //更新p(Z_nk)
    void update_W(vector<size_t> &result){
        //计算每个点属于每个高斯分布的概率
        Eigen::MatrixXd pdfs;
        pdfs.resize(n_points, n_clusters_);
        pdfs = Eigen::MatrixXd::Zero(n_points, n_clusters_);
        //计算每个点属于第i个聚类的可能性。\pi_k * \mathcal{N}(x_n|\mu_k,\Sigma_k)
        for (int i = 0; i < n_clusters_; ++i) {
//            cout << "before update" << "W[" << i <<"] = \n" << W.col(i) << endl;
            //点由第i个高斯模型生成（pi[i]）且生成成功的概率（*高斯函数的数值）
            pdfs.col(i) = pi[i] * multivariate_normal_distribution(data_, mu.row(i), var[i]);
        }
        for (int i = 0; i < n_clusters_; ++i) {
            //归一化，需要保证每个点属于所有高斯分布的概率之和为1
            //W(a, b)表示第a个点属于第b个类的概率
//            cout << pdfs.col(i).array() / pdfs.rowwise().sum().array();
//            if (((pdfs.col(i).array() / pdfs.rowwise().sum().array()).minCoeff()) < LONG_MIN){
//                over = true;
//                break;
//            }
            W.col(i) = pdfs.col(i).array() / pdfs.rowwise().sum().array();
//            cout << double (floor(W.maxCoeff())) << endl;
//            cout << double (ceil(W.maxCoeff())) << endl;
//            cout << W.minCoeff() << endl;
//            cout <<( W.minCoeff() < (1e-10) )<< endl;
//            if (( W.minCoeff() < (1e-10) )){
//            if (W.maxCoeff() ==1){
//                over = true;
//            }
//            cout << "over = " << over << endl;
//            cout << "after update" << "W[" << i <<"] = " << W.col(i) << endl;
        }
        int row_idx, col_idx;
        for (int i = 0; i < n_points; ++i) {
            W.row(i).maxCoeff(&row_idx, &col_idx);
            result.push_back(col_idx);
        }
    }


    //更新mu
    void update_mu(){
//        cout << "before update " << "mu=" << mu << endl;
        mu = Eigen::MatrixXd::Zero(n_clusters_, data_.cols());
        for (int k = 0; k < n_clusters_; ++k) {

            //计算N_k
            //W.colwise().sum()(0,k);
            //将属于该类的所有点的坐标加权求和
            for (int j = 0; j < data_.rows(); ++j) {
                //\sum_{n=1}^N\gamma(z_{nk}) x_n
                mu.row(k) += W(j,k) * data_.row(j);
            }
            //求平均
            mu.row(k) = mu.row(k)/W.colwise().sum()(0,k);
//            cout << "after update" << "mu[" << k <<"] = " << mu.row(k) << endl;
        }
    }

    //更新var
    void update_var(){
        for (int k = 0; k < n_clusters_; ++k) {
//            cout << "before update" << " var[" << k <<"] = \n" << var[k] << endl;
            //清空原来数据，为累加做准备
            var[k] = Eigen::MatrixXd::Zero(data_.cols(), data_.cols());
            //计算N_k
            //W.colwise().sum()(0,k);
            //将属于该类的所有点的协方差矩阵加权求和
            for (int j = 0; j < data_.rows(); ++j) {
                //\sum_{n=1}^N\gamma(z_{nk}) x_n
//                cout << "data_.row(" << j <<") = " << data_.row(j) << endl;
//                cout << "mu.row(" << k <<") = " << mu.row(k) << endl;
//                cout << "(data_.row(" << j << ") - mu.row(" << k << " ))" << (data_.row(j) - mu.row(k)) << endl;
                var[k] += W(j,k) * (data_.row(j) - mu.row(k)).transpose() * (data_.row(j) - mu.row(k));
//                cout << var[k] << endl;
            }
            //求平均
            var[k] = var[k]/W.colwise().sum()(0,k);
//            cout << "after update" << "var[" << k <<"] = \n" << var[k] << endl;
        }
    }

    //更新pi
    void update_pi(){
        for (int k = 0; k < n_clusters_; ++k) {
//            cout << "before update" << "pi[" << k <<"] = " << pi[k] << endl;
//            cout << "W.colwise().sum()(0,k)/n_points:  " << W.colwise().sum()(0,k)/n_points << endl;
            pi[k] = W.colwise().sum()(0,k)/n_points;
//            cout << "after update" << "pi[" << k <<"] = " << pi[k] << endl;
        }
    }

    //计算损失函数.MLE
    double logLH(){
        //计算每个点属于每个高斯分布的概率
        Eigen::MatrixXd pdfs;
        pdfs.resize(n_points, n_clusters_);
//        cout << "resize successfully" << endl;
        pdfs = Eigen::MatrixXd::Zero(n_points, n_clusters_);
//        cout << "initialized" << endl;
        //遍历每个高斯分布
        for (int k = 0; k < n_clusters_; ++k) {
            //点由第i个高斯模型生成（pi[i]）且生成成功的概率（*高斯函数的数值）
            pdfs.col(k) = pi[k] * multivariate_normal_distribution(data_, mu.row(k), var[k]);
        }
//        cout << "pdfs caled" << endl;
        //先对点求和取对数再对聚类求和
        double mle = 0;
        for (int i = 0; i < n_points; ++i) {
            mle += log(pdfs.rowwise().sum()(i,0));
        }
        return mle;
    }

    void fit(vector<size_t> &result){
//        迭代次数
        int num_iter = 0;
        //保存当前的损失函数；
        loglh.push_back(logLH());
        while ( num_iter < max_iter_ && over == false){
//            cout << "num_iter = " << num_iter << endl;
            update_W(result);
            update_pi();
            update_mu();
            update_var();

            loglh.push_back(logLH());
            num_iter++;

        }
    }

    void predict(vector<size_t> &result){
//计算每个点属于每个高斯分布的概率
        Eigen::MatrixXd pdfs;
        pdfs.resize(n_points, n_clusters_);
        pdfs = Eigen::MatrixXd::Zero(n_points, n_clusters_);
        Eigen::MatrixXd W__;              //p(Z_nk)，每个点属于某一个类的概率
        //初始化p(Z_nk)，初始时刻认为属于每个聚类的概率相等
        W__.resize(n_points,n_clusters_);
        W__ = Eigen::MatrixXd::Ones(n_points,n_clusters_) / n_clusters_;
        //计算每个点属于第i个聚类的可能性。\pi_k * \mathcal{N}(x_n|\mu_k,\Sigma_k)
        for (int i = 0; i < n_clusters_; ++i) {
            //点由第i个高斯模型生成（pi[i]）且生成成功的概率（*高斯函数的数值）
            pdfs.col(i) = pi[i] * multivariate_normal_distribution(data_, mu.row(i), var[i]);
        }
        for (int i = 0; i < n_clusters_; ++i) {
            //归一化，需要保证每个点属于所有高斯分布的概率之和为1
            //W(a, b)表示第a个点属于第b个类的概率
            W__.col(i) = pdfs.col(i).array() / pdfs.rowwise().sum().array();
        }

        int row_idx, col_idx;
        for (int i = 0; i < n_points; ++i) {
            W__.row(i).maxCoeff(&row_idx, &col_idx);
            result.push_back(col_idx);
        }
//        cout << "W__" << W__ << endl;
    }



};
#endif //POINT_CLOUD_PROCESS_SL_GMM_H
