//
// Created by pyh on 2022/8/2.
//
#include "octree.h"
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
using namespace std;
int main(){
    Eigen::Matrix<double, 9, 3> mx_in;
    mx_in << 1,0,0,
             1,2,3,
             0,2,0,
             0,0,3,
             2,3,4,
             4,5,6,
             6,7,8,
             7,8,9,
             8,9,10;
    cout << mx_in << endl;
    // -----------------------------------创建Octree-------------------------------------
    auto *OcTree = new octant;
    octree_construction(OcTree, mx_in, 1, 0.000001);
    OcTree->display();
    cout << endl;
    for (int i = 0; i < 8; ++i) {
        cout << "OcTree->children[" << i << "]" << endl;
        OcTree->children[i]->display();
        cout << endl;
    }
    for (int i = 0; i < 8; ++i) {
        cout << "OcTree->children[0]->children[" << i << "]" << endl;
        OcTree->children[0]->children[i]->display();
        cout << endl;
    }
    for (int i = 0; i < 8; ++i) {
        cout << "OcTree->children[0]->children[7]->children[" << i << "]" << endl;
        OcTree->children[0]->children[7]->children[i]->display();
        cout << endl;
    }

    //-------------------------------------目标点-----------------------------------------------------
    Eigen::Matrix<double,1,3> search_point = mx_in.row(1);
    cout << "search_point: " << search_point << endl;
    RadiusNNResultSet radius_result_set(4);
    octree_radius_search(OcTree, mx_in, radius_result_set, search_point);
    cout << "count = " << radius_result_set.count << endl;
    for (int i = 0; i < radius_result_set.count; ++i) {

        radius_result_set.dist_index_list[i].dis_DistIndex();

    }
    return 0;
}
