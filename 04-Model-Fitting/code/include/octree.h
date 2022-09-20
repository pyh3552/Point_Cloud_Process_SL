//
// Created by pyh on 2022/8/1.
//
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
#include "result_set.h"
using namespace std;
#ifndef POINT_CLOUD_PROCESS_SL_OCTREE_H
#define POINT_CLOUD_PROCESS_SL_OCTREE_H
//节点，构成Octree的基本元素
class octant{
public:
    octant(){
//        children[0] = nullptr;
//        children[1] = nullptr;
//        children[2] = nullptr;
//        children[3] = nullptr;
//        children[4] = nullptr;
//        children[5] = nullptr;
//        children[6] = nullptr;
//        children[7] = nullptr;
        center[0,0] = NULL;
        center[0,1] = NULL;
        center[0,2] = NULL;
    }
    //打印该Octant的内容
    void display(){
        std::cout << "Center : " << center.row(0) << std::endl;
        std::cout << "Extent : " << extent << std::endl;
        std::cout << "is_leaf : " << is_leaf << std::endl;
        std::cout << "point_indices : " << std::endl;
        for (int i = 0; i < point_indices.size(); ++i) {
            std::cout << point_indices[i] << "  ";
        }
        cout << endl;
    }

    //指向子节点的指针
    std::vector<octant *> children{nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};
    //正方体中心点
    Eigen::Matrix<double , 1, 3> center;
    //正方体边长的一半 0.5*length
    double extent;
    //Octant中点的索引
    std::vector<size_t> point_indices;
    //是否是一个leaf
    bool is_leaf = true;

};
//# 功能：遍历octree
//# 输入：
//#     root: 构建好的octree
//#     depth: 当前深度
//#     max_depth：最大深度
void traverse_octree(octant* root, std::vector<size_t> depth, std::vector<size_t> max_depth){
    depth[0] += 1;
    if (max_depth[0] < depth[0]){
        max_depth[0] = depth[0];
    }
    //根节点未被初始化，八叉树为空
    if (root->center[0,0] == NULL && root->center[0,1] == NULL && root->center[0,2] == NULL){
        return;
    }
    //当前Octant是一个leaf
    else if(root->is_leaf){
        for (int i = 0; i < root->point_indices.size(); ++i) {
            std::cout << root->point_indices[i] << std::endl;
        }
    }
    //Octant不是一个leaf时
    else {
        int j = 0;
        //找出有几个children
        for (int i = 0; i < root->children.size(); ++i) {
            if (root->children[i] != nullptr) {
                j++;
            }
        }
        for (int k = 0; k < j; ++k) {
            traverse_octree(root->children[k], depth, max_depth);
        }
    }
    //只有打印完一个leaf后才会执行这个语句，将层数-1
    depth[0] -= 1;
}


//# 功能：通过递归的方式构建octree
//# 输入：
//#     root：根节点
//#     db：原始数据
//#     center: 中心
//#     extent: 当前分割区间
//#     point_indices: 点的key
//#     leaf_size: scale：每个叶子节点包含的最多数据点个数
//#     min_extent:   最小分割区间

void octree_recursive_build(octant* root, Eigen::MatrixXd db, int leaf_size, double min_extent){

    //如果Octant里面没有任何点
    if (root->point_indices.size() == 0){
//        cout << "no point when build" << endl;
        return;
    }

    //构建当前节点, 若当前节点未被初始化，就调用构造函数
    //感觉对我没用,因为children的内容就是octant指针
//    if (root->center[0,0] == NULL && root->center[0,1] == NULL && root->center[0,2] == NULL){
//        *root = octant();
//    }

    //决定是否要继续划分当前节点
    if (root->point_indices.size() <= leaf_size || root->extent <= min_extent){
        //如果Octant中的点数≤leaf size或者Octant的边长一半≤min extent，那就是leaf了，不用再划分
        root->is_leaf = true;
    }
    else{
        root->is_leaf = false;
        //用于存放每个子octant里面有哪些点，这些点的索引
        std::vector<std::vector<size_t> > children_point_indices (8);
        //遍历当前节点下的每一个数据点，确定其所属的空间网格
        for (int point_idx = 0; point_idx < root->point_indices.size(); ++point_idx) {
            Eigen::Matrix<double, 1, 3> cur_point = db.row(root->point_indices[point_idx]);
            //一个Octant能分出8份。8的二进制数是1000。通过四位2进制数来表征八个子节点。
            //二进制的第1,2,3三位是当前点是否在x,y,z轴上超过中心点的开关量，顺利让morton_code表达了1-7
            int morton_code = 0;
            if (cur_point[0,0] > root->center[0,0]){
                //按位或运算
                morton_code = morton_code | 1;
            }
            if (cur_point[0,1] > root->center[0,1]){
                //按位或运算
                morton_code = morton_code | 2;
            }
            if (cur_point[0,2] > root->center[0,2]){
                //按位或运算
                morton_code = morton_code | 4;
            }
            //
            children_point_indices[morton_code].push_back(root->point_indices[point_idx]);
        }

        // 遍历每一个空间网格，继续递归构建下一层
        double factor[2] = {-0.5, 0.5};
        for (int i = 0; i < 8; ++i) {
            root->children[i] =  new octant;
            root->children[i]->center[0] = root->center[0] + factor[(i & 1) > 0] * root->extent;
            root->children[i]->center[1] = root->center[1] + factor[(i & 2) > 0] * root->extent;
            root->children[i]->center[2] = root->center[2] + factor[(i & 4) > 0] * root->extent;
            root->children[i]->extent    = root->extent * 0.5;
            root->children[i]->point_indices = children_point_indices[i];
//            root->children[i];
            octree_recursive_build(root->children[i], db, leaf_size, min_extent);
//            double child_center_x = root->center[0] + factor[(i & 1) > 0] * root->extent;
//            double child_center_y = root->center[1] + factor[(i & 2) > 0] * root->extent;
//            double child_center_z = root->center[2] + factor[(i & 4) > 0] * root->extent;
        }
    }

}

//# 功能：判断当前query区间是否在octant内
//# 输入：
//#     query: 索引信息
//#     radius：索引半径
//#     octant：octree
//# 输出：
//#     判断结果，即True/False
bool inside(Eigen::Matrix<double, 1, 3> query, double radius, octant * oc){

    //搜索点到octant中心点的偏移向量,可以视为以center为坐标原点的坐标系中query的坐标
    Eigen::Matrix<double, 1, 3> query_offset = query - oc->center;
    //将offset中的值取绝对值
    Eigen::Array<double, 1, 3> query_offset_abs = query_offset.array().abs();
    //以radius为半径，query为圆心的球体（全正）。从query出发，沿x,y,z方向分别移动一个radius的距离
    Eigen::Array<double, 1, 3> possible_space = query_offset_abs + radius;
    //三个终点的相应的坐标值均小于extent时，表示这个球体被包含在了octant内
    if(possible_space[0,0] < oc->extent && possible_space[0,1] < oc->extent && possible_space[0,2] < oc->extent){
        cout << "inside" << endl;
    }

    return (possible_space[0,0] < oc->extent && possible_space[0,1] < oc->extent && possible_space[0,2] < oc->extent);
}

//# 功能：判断当前query区间是否和octant有重叠部分
//# 输入：
//#     query: 索引信息
//#     radius：索引半径
//#     octant：octree
//# 输出：
//#     判断结果，即True/False
bool overlaps(Eigen::Matrix<double, 1, 3> query, double radius, octant * oc){
    //搜索点到octant中心点的偏移向量,可以视为以center为坐标原点的坐标系中query的坐标
    Eigen::Matrix<double, 1, 3> query_offset = query - oc->center;
    //将offset中的值取绝对值
    Eigen::Array<double, 1, 3> query_offset_abs = query_offset.array().abs();

    //以center为坐标原点的坐标系中query的坐标,在任意一个方向上大于半径和extent之和，就表明query球体完全在正方体之外
    double max_dist = radius + oc->extent;
    if (query_offset_abs[0,0] > max_dist || query_offset_abs[0,1] > max_dist || query_offset_abs[0,2] > max_dist)
    {
        cout << "query球体完全在正方体之外" << endl;
        return false;
    }


    //上面的if已经排除了不接触了情况，剩下的情况都是球和立方体有接触的
    //只要有任意两个坐标轴上的值，被限定在octant的extent中，那么球和立方体必然有（侧）面接触，上面的if已经排除了第三根轴太大的情况。
    if ( ( (int (query_offset_abs[0,0] < oc->extent))   +   (int (query_offset_abs[0,1] < oc->extent))   +   (int (query_offset_abs[0,2] < oc->extent)) ) >= 2){
        cout << "面接触" << endl;
        return true;
    }

    //考虑球体接触octant的角或者边
    //由于query球体在octant内部的情况已经考虑了。排除有任意两个坐标轴上的值，被限定在octant的extent中的情况。
    //我们只需要考虑query点在正方体外的情况
    //1.对于和棱相接触的情况，棱是哪个坐标轴方向的，就将球体和正方体投影到与该轴垂直的平面上进行考虑
    //  通过相减获得了球心（此时为圆心）到正方体棱（此时为正方形的角点）的距离在两个方向上的投影。
    //2.而对于和角接触的情况，则不用做投影。具体在下面的情况中，就是不会取到0。
    double x_diff = ( (query_offset_abs[0,0] - oc->extent) > 0 ) ? (query_offset_abs[0,0] - oc->extent) : 0;
    double y_diff = ( (query_offset_abs[0,1] - oc->extent) > 0 ) ? (query_offset_abs[0,1] - oc->extent) : 0;
    double z_diff = ( (query_offset_abs[0,2] - oc->extent) > 0 ) ? (query_offset_abs[0,2] - oc->extent) : 0;
    //只要圆心和角点的距离小于半径，那么球体和正方体就是相交的
    if (x_diff * x_diff + y_diff * y_diff + z_diff * z_diff < radius * radius){
        cout << "角或者棱" << endl;
    }
    return x_diff * x_diff + y_diff * y_diff + z_diff * z_diff < radius * radius;
}

//# 功能：判断当前query是否包含octant
//# 输入：
//#     query: 索引信息
//#     radius：索引半径
//#     octant：octree
//# 输出：
//#     判断结果，即True/False
bool contains(Eigen::Matrix<double, 1, 3> query, double radius, octant * oc){
    //搜索点到octant中心点的偏移向量,可以视为以center为坐标原点的坐标系中query的坐标
    Eigen::Matrix<double, 1, 3> query_offset = query - oc->center;
    //将offset中的值取绝对值
    Eigen::Array<double, 1, 3> query_offset_abs = query_offset.array().abs();
    //偏移向量+一个center到角点的对角线向量，得到的向量的长度，就是query到正方体的最远距离。
    Eigen::Array<double, 1, 3> query_offset_to_farthest_corner = query_offset_abs + oc->extent;
    //最远距离比半径小，则正方体完全在球体内部
    return query_offset_to_farthest_corner.matrix().norm() < radius;
}

//# 功能：在octree中查找信息
//# 输入：
//#    root: octree
//#    db：原始数据
//#    result_set: 索引结果
//#    query：索引信息
bool octree_radius_search_fast(octant * root, Eigen::MatrixXd db, RadiusNNResultSet &result_set, Eigen::Matrix<double, 1, 3> query){
    //如果Octant里面没有任何点
    if (root->point_indices.size() == 0){
        return false;
    }

    // case1 如果当前的搜索点及搜索半径所构成的球面包含当前的网格，则选取当前网格中的所有数据点
    if (contains(query, result_set.worst_distance, root)){
        //比较当前网格中的所有点
//        Eigen::MatrixXd leaf_points;
//        Eigen::MatrixXd query_expand;
        std::vector<double> diff;
//        for (int i = 0; i < root->point_indices.size(); ++i) {
//            leaf_points.row(i) = db.row(root->point_indices[i]);
//            query_expand.row(i) = query;
//            diff.push_back( ( query_expand - leaf_points).row(i).norm() );
//        }
        for (int i = 0; i < root->point_indices.size(); ++i) {
            //得到搜索点到该octant中数据点的距离
            diff.push_back( ( query - db.row(root->point_indices[i])).norm() );
            result_set.add_point(diff[i], root->point_indices[i]);
        }
        return false;
    }

    //case2 如果当前节点是叶子节点Leaf
    if (root->is_leaf && (root->point_indices.size() > 0)){
        //比较leaf中的内容
        std::vector<double> diff;
        for (int i = 0; i < root->point_indices.size(); ++i) {
            //得到搜索点到该octant中数据点的距离
            diff.push_back( ( query - db.row(root->point_indices[i])).norm() );
            result_set.add_point(diff[i], root->point_indices[i]);
        }
        //判断是否需要提前停止，提前停止的条件为：当前搜索点及搜索半径所构成的球面完全包含于当前的网格中
        return inside(query, result_set.worst_distance, root);
    }

    //case3 循环遍历当前网格的8个子网格，并递归搜索
    for (int i = 0; i < root->children.size(); ++i) {
        if (root->children[i] == nullptr){
            continue;
        }
        //如果搜索点的球体和正方体不相交
        if (false == overlaps(query, result_set.worst_distance, root->children[i])){
            continue;
        }
        //递归搜索子节点
        if (octree_radius_search_fast(root->children[i], db, result_set, query)){
            return true;
        }
    }
    //每一层都能判断到是否要提前终止搜索
    return inside(query, result_set.worst_distance, root);
}


//# 功能：在octree中查找radius范围内的近邻
//# 输入：
//#     root: octree
//#     db: 原始数据
//#     result_set: 搜索结果
//#     query: 搜索信息
bool octree_radius_search(octant * root, Eigen::MatrixXd db, RadiusNNResultSet &result_set, Eigen::Matrix<double, 1, 3> query){
    //如果Octant里面没有任何点
    if (root->point_indices.size() == 0){
        cout << "no point" << endl;
        return false;
    }

    //如果当前节点是叶子节点Leaf
    if (root->is_leaf && (root->point_indices.size() > 0)){
//        cout << "It is a leaf." << endl;
        //比较leaf中的内容
        std::vector<double> diff;
        for (int i = 0; i < root->point_indices.size(); ++i) {
            //得到搜索点到该octant中数据点的距离
            diff.push_back( ( query - db.row(root->point_indices[i])).norm() );
            result_set.add_point(diff[i], root->point_indices[i]);
        }
        //判断是否需要提前停止，提前停止的条件为：当前搜索点及搜索半径所构成的球面完全包含于当前的网格中
        return inside(query, result_set.worst_distance, root);
    }

    //首先搜索当前数据点所在的子网格
    //一个Octant能分出8份。8的二进制数是1000。通过四位2进制数来表征八个子节点。
    //二进制的第1,2,3三位是当前点是否在x,y,z轴上超过中心点的开关量，顺利让morton_code表达了1-7
    int morton_code = 0;
    if (query[0,0] > root->center[0,0]){
        //按位或运算
        morton_code = morton_code | 1;
    }
    if (query[0,1] > root->center[0,1]){
        //按位或运算
        morton_code = morton_code | 2;
    }
    if (query[0,2] > root->center[0,2]){
        //按位或运算
        morton_code = morton_code | 4;
    }
    //可以一路递归得到搜索点所在的leaf
    if (octree_radius_search(root->children[morton_code], db, result_set, query)){
        return true;
    }

    //递归搜索剩下的子网格
    for (int i = 0; i < 8; ++i) {
        if (i == morton_code || root->children[i] == nullptr){
            cout << "i == morton_code || root->children[i] == nullptr" << endl;
            continue;
        }
        //如果搜索点的球体和正方体不相交
        if (false == overlaps(query, result_set.worst_distance, root->children[i])){
//            root->children[i]->display();
            cout << "no overlaps" << endl;
            continue;
        }
        //递归搜索子节点
        if (octree_radius_search(root->children[i], db, result_set, query)){
            return true;
        }
    }
    //每一层都能判断到是否要提前终止搜索，提前停止的条件为：当前搜索点及搜索半径所构成的球面完全包含于当前网格中
    return inside(query, result_set.worst_distance, root);
}

//# 功能：在octree中查找最近的k个近邻
//# 输入：
//#     root: octree
//#     db: 原始数据
//#     result_set: 搜索结果
//#     query: 搜索信息
bool octree_knn_search(octant * root, Eigen::MatrixXd db, KnnResultSet &result_set, Eigen::Matrix<double, 1, 3> query){
    //如果Octant里面没有任何点
    if (root->point_indices.size() == 0){
        return false;
    }

    //如果当前节点是叶子节点Leaf
    if (root->is_leaf && (root->point_indices.size() > 0)){
        //比较leaf中的内容
        std::vector<double> diff;
        for (int i = 0; i < root->point_indices.size(); ++i) {
            //得到搜索点到该octant中数据点的距离
            diff.push_back( ( query - db.row(root->point_indices[i])).norm() );
            result_set.add_point(diff[i], root->point_indices[i]);
        }
        //判断是否需要提前停止，提前停止的条件为：当前搜索点及搜索半径所构成的球面完全包含于当前的网格中
        cout << "inside(query, result_set.worstDist(), root) = " << inside(query, result_set.worstDist(), root) << endl;
        return inside(query, result_set.worstDist(), root);
    }

    //首先搜索当前数据点所在的子网格
    //一个Octant能分出8份。8的二进制数是1000。通过四位2进制数来表征八个子节点。
    //二进制的第1,2,3三位是当前点是否在x,y,z轴上超过中心点的开关量，顺利让morton_code表达了1-7
    int morton_code = 0;
    if (query[0,0] > root->center[0,0]){
        //按位或运算
        morton_code = morton_code | 1;
    }
    if (query[0,1] > root->center[0,1]){
        //按位或运算
        morton_code = morton_code | 2;
    }
    if (query[0,2] > root->center[0,2]){
        //按位或运算
        morton_code = morton_code | 4;
    }
    //可以一路递归得到搜索点所在的leaf
    if (octree_knn_search(root->children[morton_code], db, result_set, query)){
        return true;
    }

    //递归搜索剩下的子网格
    for (int i = 0; i < root->children.size(); ++i) {
        if (i == morton_code || root->children[i] == nullptr){
            continue;
        }
        //如果搜索点的球体和正方体不相交
        if (false == overlaps(query, result_set.worstDist(), root->children[i])){
            continue;
        }
        //递归搜索子节点
        if (octree_knn_search(root->children[i], db, result_set, query)){
            return true;
        }
    }
    //每一层都能判断到是否要提前终止搜索，提前停止的条件为：当前搜索点及搜索半径所构成的球面完全包含于当前网格中
    return inside(query, result_set.worstDist(), root);
}






//# 功能：构建octree，即通过调用octree_recursive_build函数实现对外接口
//# 输入：
//#    dp_np: 原始数据
//#    leaf_size：scale
//#    min_extent：最小划分区间
void octree_construction(octant* root, Eigen::MatrixXd db, int leaf_size, double min_extent){
    size_t N = db.rows();
    int dim  = db.cols();
    //确定最初正方体的边界
    Eigen::Vector3d db_max = db.colwise().maxCoeff();
    Eigen::Vector3d db_min = db.colwise().minCoeff();
    //最初正方体的棱长的一半
    double db_extent = 0.5 * (db_max - db_min).maxCoeff();
    cout << "最初正方体的棱长的一半: " << db_extent;
    //确定最初正方体的中心点
    Eigen::Matrix<double, 1, 3> db_center = db.colwise().mean();
    root->center = db_center;
    root->extent = db_extent;
    //将点云中的点按序放入索引中
    for (int i = 0; i < N; ++i) {
        root->point_indices.push_back(i);
    }
    octree_recursive_build(root, db, leaf_size, min_extent);
}


#endif //POINT_CLOUD_PROCESS_SL_OCTREE_H
