//
// Created by pyh on 2022/8/21.
//
#include <ctime>
#include <cstdlib>
#include "kdtree.h"
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#ifndef POINT_CLOUD_PROCESS_SL_RANSAC_REGISTRATION_H
#define POINT_CLOUD_PROCESS_SL_RANSAC_REGISTRATION_H
class RANSAC_build{
    friend class RANSAC_ICP;
public:
    RANSAC_build(Eigen::MatrixXd* source_points, Eigen::MatrixXd* target_points):source_point_cloud_(source_points),target_point_cloud_(target_points){
        max_works_ = 16;
        num_samples_ = 4;
        max_correspondence_distance_ = 0.0015;
        max_iter_ = 500;
        max_validation_ = 500;
        max_edge_length_ratio_ = 0.9;
    }

    void set_max_works(int max_works){
        max_works_ = max_works;
    }

    void set_num_samples(int num_samples){
        num_samples_ = num_samples;
    }

    void set_max_correspondence_distance(double max_correspondence_distance){
        max_correspondence_distance_ = max_correspondence_distance;
    }

    void set_max_iter(int max_iter){
        max_iter_ = max_iter;
    }

    void set_max_validation(int max_validation){
        max_validation_ = max_validation;
    }

    void set_max_edge_length_ratio_(double max_edge_length_ratio){
        max_edge_length_ratio_ = max_edge_length_ratio;
    }

    void set_normal_angle_threshold(double normal_angle_threshold){
        normal_angle_threshold_ = normal_angle_threshold;
    }
private:
    Eigen::MatrixXd* const source_point_cloud_;                //输入的源点云数据
    Eigen::MatrixXd* const target_point_cloud_;                //输入的目标点云数据
    int max_works_;                                             //
    int num_samples_;                                           //
    double max_correspondence_distance_;                        //
    int max_iter_;                                              //最大迭代次数
    int max_validation_;                                        //
    double max_edge_length_ratio_;                              //
    double normal_angle_threshold_;                             //

};

class RANSAC_ICP{

public:
    RANSAC_ICP(RANSAC_build builder){
        max_works_icp = builder.max_works_;
        num_samples_icp = builder.num_samples_;
        max_correspondence_distance_icp = builder.max_correspondence_distance_;
        max_iter_icp = builder.max_iter_;
        max_validation_icp = builder.max_validation_;
        max_edge_length_ratio_icp = builder.max_edge_length_ratio_;
        normal_angle_threshold_icp = builder.normal_angle_threshold_;

    }

    void set_source_points(Eigen::MatrixXd* pcd_source){
        pcd_source_ = pcd_source;
    }

    void set_target_points(Eigen::MatrixXd pcd_target){
        pcd_target_ = pcd_target;
    }

    void set_source_fpfh(Eigen::MatrixXd source_fpfh){
        source_fpfh_ = source_fpfh;
    }

    void set_target_fpfh(Eigen::MatrixXd target_fpfh){
        target_fpfh_ = target_fpfh;
    }

    void ransac_match(){
        cout << "in ransac_match" << endl;
        //获取对应的特征空间匹配对
        // 第一列为源点云中的特征点的索引，第二列为目标点云中和它有最相近描述子的点的索引
        Eigen::Matrix<size_t, Eigen::Dynamic, 2> matches = get_potential_matches();
        cout << "matches gotten" << endl;
        //在目标点云中建立kdtree
        auto * kdtree_pcd_target_ = new kdtree;
        build_kd_tree(kdtree_pcd_target_, mat2vecinvec(pcd_target_), 0);
        //RANSAC
        //随机选取三个点对
        Eigen::Matrix<size_t, 3, 2> pairs_chosen;
        //srand不得调用>=2次
        srand((int)time(0));
        //
        Eigen::MatrixXd input_source = *pcd_source_;
        //开始迭代
        for (int i = 0; i < max_iter_icp; ++i) {
            //提取出点对所对应的坐标
            Eigen::MatrixXd source_chosen_pos;
            source_chosen_pos.resize(3, pcd_source_->cols());
            Eigen::MatrixXd target_chosen_pos;
            target_chosen_pos.resize(3, pcd_target_.cols());
            for (int i = 0; i < 3; ++i) {
                pairs_chosen.row(i) = matches.row(rand() % matches.rows());
                source_chosen_pos.row(i) = pcd_source_->row(pairs_chosen(i, 0));
                target_chosen_pos.row(i) = pcd_target_.row(pairs_chosen(i, 1));
            }
            cout << "points selected" << endl;
            //通过Procrustes Transformation解出变换矩阵
            Eigen::MatrixXd Trans;
            Trans = solve_icp(source_chosen_pos, target_chosen_pos);
            cout << "Transformation caled" << endl;
            //计算内点的数量
            check_fitness(Trans, input_source, kdtree_pcd_target_);
            cout << "check fitness" << endl;
        }

    }

    Eigen::MatrixXd get_Trans_final(){
//        cout << Trans_final << endl;
        return Trans_final;
    }



private:
    int max_works_icp;                                             //
    int num_samples_icp;                                           //
    double max_correspondence_distance_icp;                        //
    int max_iter_icp;                                              //最大迭代次数
    int max_validation_icp;                                        //
    double max_edge_length_ratio_icp;                              //
    double normal_angle_threshold_icp;                             //

    Eigen::MatrixXd* pcd_source_;
    Eigen::MatrixXd pcd_target_;
    Eigen::MatrixXd source_fpfh_;
    Eigen::MatrixXd target_fpfh_;


    vector<std::size_t> inlier_idx;                          //inlier包含的点在data中的索引
    vector<std::size_t> final_inlier_idx;                    //有最多inlier，inlier包含的点在data中的索引

    Eigen::MatrixXd Trans_final;                             //拥有最多内点的变换

    //将矩阵转化为嵌套的vector，以后可以修改kdtree来省略这一步
    vector<vector<double> > mat2vecinvec(Eigen::MatrixXd mat){
        vector<vector<double> > mat_in_vec;
        mat_in_vec.resize(mat.rows());
        for (int row = 0; row < mat.rows(); ++row) {
            for (int col = 0; col < mat.cols(); ++col) {
                mat_in_vec[row].push_back(mat(row,col));
            }
        }
        return mat_in_vec;
    }


    Eigen::Matrix<size_t, Eigen::Dynamic, 2> get_potential_matches(){
        //用于存储特征空间中配对的源点云点和目标点云点的索引，
        // 第一列为源点云中的点的索引，第二列为目标点云中和它有最相近描述子的点的索引
        Eigen::Matrix<size_t, Eigen::Dynamic, 2> matches;
        matches.resize(source_fpfh_.rows(), 2);
        cout << "matches constructed" << endl;
        //在高维空间构建对应的搜索树——在目标点云的特征空间中构建搜索树
        auto* kdTree = new kdtree;
        build_kd_tree(kdTree, mat2vecinvec(target_fpfh_), 0);
        cout << "tree of target_fpfh_ constructed" << endl;
        //创建搜索结果集
        kd_KnnResultSet resultset_fpfh(1);
        cout << "resulet set of tree of target_fpfh_ constructed" << endl;
        //以源点云中的每一个点的描述子作为搜索点输入，寻找目标点云的特征空间中和它最相近的一个点
        for (int i = 0; i < source_fpfh_.rows(); ++i) {
            //查找第i个点对应的描述子，在目标点云中最近的点。
            kdtree_knn_search(kdTree, target_fpfh_, resultset_fpfh, source_fpfh_.row(i));
            matches(i,0) = i;
            matches(i,1) = resultset_fpfh.dist_index_list[0].index;
        }
        cout << "matches caled" << endl;
        return matches;
    }






    //求解Procrustes Transformation
    Eigen::MatrixXd solve_icp(Eigen::MatrixXd source, Eigen::MatrixXd target){
        //对source进行去中心化
        Eigen::MatrixXd source_uncentered;
        source_uncentered.resize(source.rows(), source.cols());
        source_uncentered = source.rowwise() - source.colwise().mean();
        cout << "source_uncentered caled" << endl;
        //对target进行去中心化
        Eigen::MatrixXd target_uncentered;
        target_uncentered.resize(target.rows(), target.cols());
        target_uncentered = target.rowwise() - target.colwise().mean();
        cout << "target_uncentered caled" << endl;
        //对B'A^T进行奇异值分解
        Eigen::MatrixXd H;
        H.resize(target.rows(), source.rows());
        H = target_uncentered.transpose() * source_uncentered;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullV| Eigen::ComputeFullU);
        Eigen::MatrixXd U;
        Eigen::MatrixXd V;
        Eigen::MatrixXd Sigma;
        U = svd.matrixU();
        V = svd.matrixV();
        Sigma = svd.singularValues();
        cout << "SVD caled" << endl;
        //得到旋转矩阵和平移向量
        Eigen::MatrixXd mat_rotation;//理应是一个三行三列的矩阵
        mat_rotation = U * V.transpose();
        Eigen::MatrixXd vec_movement;//理应是一个三行一列的向量
        vec_movement = (target.transpose() - mat_rotation * source.transpose()).rowwise().mean();
        cout << "R, t caled" << endl;
        //将旋转矩阵和平移向量拼接后返回
        Eigen::MatrixXd transform;
        transform.resize(mat_rotation.rows() + vec_movement.transpose().rows(), mat_rotation.cols());

        transform.block(0, 0, mat_rotation.rows(), mat_rotation.cols()) = mat_rotation;

        transform.block(mat_rotation.rows(), 0, vec_movement.cols(), vec_movement.rows()) = vec_movement.transpose();
        ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/09-Registration/doc/R.txt", ios_base::app);
        outfile << "mat_rotation:\n" << mat_rotation << endl;
        cout << transform << endl;
        return transform;
    }

    //对源点云使用估计出来的变换矩阵，对变换后的源点云中的每个点，搜索在目标点云中的最近邻，计算距离。由距离判断是否属于inlier。
    void check_fitness(Eigen::MatrixXd Trans, Eigen::MatrixXd source, kdtree* kdtree_pcd_target_){
        inlier_idx.clear();
        final_inlier_idx.clear();
        Eigen::Matrix<double, 1, 3> t;
        t = Trans.block(3,0,1,3);
        Eigen::MatrixXd source_transformed = ( Trans.block(0,0,3,3) * source.transpose() ).transpose().rowwise() + t;
        cout << "source_transformed:\n" << source_transformed << endl;
        //创建搜索结果集
        kd_KnnResultSet resultset(1);
        //以变换后的源点云中的每个点的坐标输入，寻找目标点云的坐标中和它最相近的一个点
        for (int i = 0; i < pcd_source_->rows(); ++i) {
            cout << "source_transformed.row(i) : " << source_transformed.row(i) << endl;
            cout << "pcd_target_ row(0) : " << pcd_target_.row(0) << endl;
            //查找变换后的源点云中的第i个点，在目标点云中最近的点。
            kdtree_knn_search(kdtree_pcd_target_, pcd_target_, resultset, source_transformed.row(i));
            //如果两者距离小于阈值，则认为变换后的源点云中的该点为inlier
            cout << "resultset.dist_index_list[0].distance : " << resultset.dist_index_list[0].distance << endl;
            cout << "resultset.dist_index_list[0].index : " << resultset.dist_index_list[0].index << endl;
            if (resultset.dist_index_list[0].distance < max_correspondence_distance_icp){
                //保存inlier的索引
                inlier_idx.push_back(i);
            }
        }
        cout << "num of inlier" << inlier_idx.size() << endl;
        //如果当前计算出来的inlier数量大于final_inlier_idx，则替换其中内容。
        if (inlier_idx.size() > final_inlier_idx.size()){
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            Trans_final = Trans;
            final_inlier_idx.clear();
            for (int i = 0; i < inlier_idx.size(); ++i) {
                final_inlier_idx.push_back(inlier_idx[i]);
            }
        }
    }
};
#endif //POINT_CLOUD_PROCESS_SL_RANSAC_REGISTRATION_H
