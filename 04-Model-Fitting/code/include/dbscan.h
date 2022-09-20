//
// Created by pyh on 2022/8/9.
//
#include "octree.h"
#include "result_set.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
#include <set>
#include <numeric>
#include <ctime>
#include <algorithm>
#include <fstream>
#ifndef POINT_CLOUD_PROCESS_SL_DBSCAN_H
#define POINT_CLOUD_PROCESS_SL_DBSCAN_H
class dbscan{
public:
    dbscan(const Eigen::MatrixXd * data, double radius_threshold = 1, int min_samples = 4){
        cout << "radius_threshold_" << endl;
        radius_threshold_ = radius_threshold;
        cout << "min_samples_" << endl;
        min_samples_ = min_samples;
        cout << "n_points_" << endl;
        n_points_ = data->rows();
        //将所有点都列为未访问点，将他们的索引存入unvisited
        cout << "unvisited" << endl;
        std::vector<size_t> tmp(n_points_);
        cout << "unvisited" << endl;
        std::iota(std::begin(tmp), std::end(tmp), 0);
        cout << "unvisited" << endl;
        unvisited.insert(tmp.begin(), tmp.end());
        //初始化矩阵，用于存储每个点属于哪个类
        cout << "cluster_index" << endl;
        cluster_index.resize(data->rows(), 1);
        cout << "cluster_index" << endl;
        cluster_index = Eigen::MatrixXd::Zero(data->rows(), 1);

    }

    void fit_core(const Eigen::MatrixXd * data) {
        ofstream outfile("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/data.txt");
        cout << "ofstream finished" << endl;
        // -----------------------------------创建Octree-------------------------------------
        auto *OcTree = new octant;
        octree_construction(OcTree, *data, 1, 0.000001);
        cout << "octree finished" << endl;
        //---------------构建搜索点并展开搜索，用搜索结果构建core_set------------------------------
        cout << "search start" << endl;
        for (int i = 0; i < data->rows(); ++i) {
            //搜索点
            Eigen::Matrix<double, 1, 3> search_point = data->row(i);
            //结果数据集
            RadiusNNResultSet RNN_result_set(radius_threshold_);
            //没有被拜访过的点p，找到它半径r内点所有邻居
            octree_radius_search(OcTree, *data, RNN_result_set, search_point);
            cout << "search finished" << endl;
            //转换近邻点索引的数据类型为set
            outfile << "neighbour of point[" << i << "] :\n";
            std::set<size_t> rnn_set;
            outfile << "size : " << RNN_result_set.size() << endl;
            for (int j = 0; j < RNN_result_set.size(); ++j) {
                outfile << endl;
                outfile << RNN_result_set.dist_index_list[j].index << "  ";
                rnn_set.insert(RNN_result_set.dist_index_list[j].index);
                cout << "bug???" << endl;
            }
            cout << "bug2???" << endl;
            all_RNN.push_back(rnn_set);
            cout << "bug3???" << endl;
            //判断邻居数量>min_samples？
            if ((RNN_result_set.size() - 1) > min_samples_) {
                //邻居数量>min_samples则将当前点作为core，将它的索引存入core_set
                core_set.insert(i);
            }
//            outfile << endl;
//            outfile << "core_set:" << *core_set.rbegin() << endl;
        }
        cout << "fit-core finished" << endl;
    }

    void fit_cluster(){
//        ofstream outcluster("/home/pyh/文档/GitHub/Point_Cloud_Process_SL/04-Model-Fitting/doc/cluster.txt");
        //聚类
        while (core_set.size() > 0){
            cout << "Loop 1" << endl;
            unvisited_old = unvisited;
            //从核心点点集中随机选取一个核心点core
            srand(time(0));
            std::set<std::size_t>::iterator it_core_set = core_set.begin();
            advance(it_core_set,  rand()%(core_set.size()));
            size_t cur_core = * it_core_set;
            //将当前选中的核心点加入visited，并从unvisited中删除以表示已经访问过了
            //将当前
            for_search.insert(cur_core);
            visited.insert(cur_core);
            unvisited.erase(cur_core);
            //开启一轮聚类
            while (for_search.size() > 0){
                cout << "Loop 2" << endl;
                size_t new_core = * for_search.begin();
//                outcluster << "new core:  " << new_core << endl;
//                set<std::size_t>::iterator it_nei = all_RNN[new_core].begin();
//                outcluster << "neighbour of new core :\n";
//                for( ; it_nei != all_RNN[new_core].end(); ++it_nei) {
//                    outcluster << *it_nei << "   ";
//                }
//                outcluster << endl;
                //如果当前搜索点是核心点
                if (core_set.find(new_core) != core_set.end()){
                    //当前核心对象的近邻点和unvisited的交集S
//                    std::set<std::size_t> S;
//                    set_intersection(unvisited.begin(), unvisited.end(), all_RNN[new_core].begin(), all_RNN[new_core].end(),
//                                     inserter(S, S.begin()));
//                    outcluster << "S:\n";
//                    set<std::size_t>::iterator it_S = S.begin();
//                    for( ; it_S != S.end(); ++it_S) {
//                        outcluster << *it_S << "   ";
//                    }
//                    outcluster << endl;
                    //用该new_core的未搜索过的近邻点替换for_search，用于再做检测
                    for_search.clear();                     //新的用于搜索的set是当前核心点的近邻点中未搜索过的点的合集
                    //用set_difference排除new_core的近邻点中已经搜索过的点
                    set_difference(all_RNN[new_core].begin(), all_RNN[new_core].end(), visited.begin(), visited.end(), insert_iterator<set<std::size_t> >(for_search,for_search.begin()));



//                    set_union(visited.begin(),visited.end(),all_RNN[new_core].begin(), all_RNN[new_core].end(),
//                              inserter(visited, visited.begin()));
//                    outcluster << "for search:\n";
//                    set<std::size_t>::iterator it_for_search = for_search.begin();
//                    for( ; it_for_search != for_search.end(); ++it_for_search) {
//                        outcluster << *it_for_search << "   ";
//                    }
//                    outcluster << endl;

                    //从unvisited中剔除本次访问了的点
                    unvisited.erase(new_core);
//                    std::set<std::size_t> unvisited_tmp;
//                    set_difference(unvisited.begin(), unvisited.end(), S.begin(), S.end(), insert_iterator<set<std::size_t> >(unvisited_tmp,unvisited_tmp.begin()));
//                    unvisited.clear();
//                    unvisited = unvisited_tmp;
//                    outcluster << "new unvisited:\n";
//                    set<std::size_t>::iterator it_new_unvisited = unvisited.begin();
//                    for( ; it_new_unvisited != unvisited.end(); ++it_new_unvisited) {
//                        outcluster << *it_new_unvisited << "   ";
//                    }
//                    outcluster << endl;
                }
                else{
                    //将不是core point的点从for_search中去除
                    for_search.erase(new_core);
                }

                //维护visited，将本次检查的点放入visited
                visited.insert(new_core);
                //当前new_core完成检测，删除。下一个就是它的邻域点了。
//                outcluster << "visited:\n";
//                set<std::size_t>::iterator it_visited = visited.begin();
//                for( ; it_visited != visited.end(); ++it_visited) {
//                    outcluster << *it_visited << "   ";
//                }
//                outcluster << endl;
//                core_set.erase(new_core);
            }

            //unvisited_old中剔除剩下的还没访问的点，就是这一轮聚类中完成聚类的点
            std::set<std::size_t> cluster;
            set_difference(unvisited_old.begin(), unvisited_old.end(), unvisited.begin(), unvisited.end(), insert_iterator<set<std::size_t> >(cluster, cluster.begin()));
            //从所有核心点的集合中去掉已经被这个聚类纳入的核心点!!!!!
            std::set<std::size_t> core_set_tmp;
            set_difference(core_set.begin(), core_set.end(), cluster.begin(), cluster.end(), insert_iterator<set<std::size_t> >(core_set_tmp,core_set_tmp.begin()));
            core_set.clear();
            core_set = core_set_tmp;
            //存储每个点属于哪个类
            std::set<std::size_t>::iterator it_cluster = cluster.begin();
            for ( ; it_cluster != cluster.end(); ++it_cluster){
                cluster_index(*it_cluster,0) = k;
            }
            //完成一轮聚类，聚类数+1
            k = k + 1;
        }
        //聚类结束，仍未被拜访的点视作噪声点
        std::set<std::size_t> noise_cluster;
        noise_cluster = unvisited;
        //噪声点归类为-1
        std::set<std::size_t>::iterator it_noise = noise_cluster.begin();
        for ( ; it_noise != noise_cluster.end(); ++it_noise){
            cluster_index(*it_noise,0) = -1;
        }
    }

    Eigen::MatrixXd get_cluster_index(){
        return cluster_index;
    }

    int get_cluster_num(){
        return k;
    }
private:
    double radius_threshold_;           //搜索半径
    int min_samples_;                   //最小邻域点个数
    size_t n_points_;                   //数据点数
    int k = 0;                          //聚类数
    Eigen::MatrixXd cluster_index;      //矩阵，用于存储每个点属于哪个类
    std::set<size_t> core_set;          //存储核心点的索引
    std::set<size_t> unvisited;         //存储尚未访问的点的索引
    std::set<size_t> visited;           //存储尚未访问的点的索引
    std::set<size_t> unvisited_old;         //存储尚未访问的点的索引
    std::set<size_t> for_search;            //存储本轮聚类探索的未搜索过点的索引
    std::vector<std::set<size_t> > all_RNN; //存储每个点的近邻点
};
#endif //POINT_CLOUD_PROCESS_SL_DBSCAN_H
