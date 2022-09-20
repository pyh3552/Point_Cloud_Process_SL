//
// Created by pyh on 2022/7/31.
//
#include "kdtree.h"
#include <iostream>
#include <vector>
using namespace std;

template <typename T>
void dis_vec(vector<T> vec){
    cout << "( ";
    for(int ix = 0; ix<vec.size() - 1; ++ix){
        cout << vec[ix] << ", ";
    }
    cout << vec[vec.size() - 1] << " )" << endl;
}


int main() {
    Eigen::Matrix4d mat;
    mat << 1, 2, 0, 1,
           2, 3, 6, 2,
           2, 6, 9, 0,
           100, 120, 900, 10;

    //赋值
    vector<vector<double> > train(4, vector<double>(4, 0));
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned j = 0; j < 4; ++j)
            train[i][j] = mat(i, j);
    //建树
    auto *kdTree = new kdtree;
    build_kd_tree(kdTree, train, 0);

    print_kd_tree(kdTree, 0);
    //目标点
    Eigen::Matrix<double,4,1> q;
    q << 1,2,3,1;
    KnnResultSet resultset(3);
//    KnnResultSet *r;
//    *r = resultset;
    //查找
    kdtree_knn_search(kdTree, mat, resultset, q);
    resultset.display();

    return 0;
}