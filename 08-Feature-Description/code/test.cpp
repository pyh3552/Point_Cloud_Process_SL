//
// Created by pyh on 2022/8/18.
//
//#include "ISS.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
using namespace std;

int main(){
//    Eigen::Matrix3d a;
//    a << 1,2,3,1,1,1,2,2,2;
//    Eigen::Matrix<double, 1, 3> b;
//    b << 1,2,3;
//    cout << a << endl;
//    cout << a.rowwise().norm() << endl;
//    cout << a.array().colwise() / a.rowwise().norm().array() << endl;
//    cout << a.array().rowwise() * b.array() << endl;
//    Eigen::Matrix<double, 1, 3> c;
//    c = a.colwise().mean();
//    cout << a.rowwise() - c << endl;

//    Eigen::Matrix<double, 4, 3> Trans;
//    Trans << 1,2,3,
//             5,6,7,
//             2,3,4,
//             4,5,6;
//    Eigen::Matrix<double, 2, 3> source;
//    source << 1,0,0,1,2,3;
//    cout << Trans.block(0,0,3,3) << endl;
//    cout << Trans.block(0,0,3,3) * source.transpose() << endl;
//    cout << ( Trans.block(0,0,3,3) * source.transpose() ).transpose() << endl;
//    cout << Trans.block(3,0,1,3) << endl;
//    Eigen::Matrix<double, 1, 3> t;
//    t = Trans.block(3,0,1,3);
//    cout << ( Trans.block(0,0,3,3) * source.transpose() ).transpose().rowwise() + t;

    Eigen::Vector3d a;
    Eigen::Matrix<double, 1, 3> b;
    b << 1,2,3;
    cout << b << endl;
    a = b;
    cout << a << endl;
    return 0;
}