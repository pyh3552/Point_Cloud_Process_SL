//
// Created by pyh on 2022/8/22.
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
int main(){
    int data[6][4] = {{0,0,0,1},{1,0,0,1},{0,1,0,1},{0,0,1,1},{1,0,1,1},{0,1,1,1}};
//    int data[6][2] = {{2,3},{5,4},{9,6},{4,7},{8,1},{7,2}};
    vector<vector<int> > data_vec;
//    Eigen::VectorXd q;
//    q.resize(4);
//    q[0] = 1;
//    q[1] = 2;
//    q[2] = 3;
//    q[3] = 1;

    //赋值
    vector<vector<double> > train(6, vector<double>(4, 0));
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 4; ++j)
            train[i][j] = data[i][j];
    //建树
    auto* kdTree = new kdtree;
    build_kd_tree(kdTree, train, 0);

    print_kd_tree(kdTree, 0);
    //目标点
    vector<double> goal;
//    goal.push_back(3);
//    goal.push_back(4.5);
    goal.push_back(100);
    goal.push_back(101);
    goal.push_back(102);
    goal.push_back(103);
    //查找
    vector<double> nearest_neighbor = search_nearest_neighbor(goal, kdTree);
    auto beg = nearest_neighbor.begin();
    cout << "The nearest neighbor is: ";
    while(beg != nearest_neighbor.end()) cout << *beg++ << ",";
    cout << endl;

    for(int i = 0; i < 6; ++i){
        cout << "data[" << i << "] = ";
        dis_vec(train[i]);
        cout << " distance: " << measure_distance(train[i] , goal, 0) << endl;
    }
    return 0;
}
