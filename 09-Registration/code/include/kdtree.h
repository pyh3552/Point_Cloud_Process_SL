//
// Created by pyh on 2022/7/30.
//
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
using namespace std;

#ifndef POINT_CLOUD_PROCESS_SL_KDTREE_H
#define POINT_CLOUD_PROCESS_SL_KDTREE_H

class kdtree{
public:
    int index = 0;
    //当前节点的内容
    vector<double> node;
    //父节点，左右子节点的指针
    kdtree* parent;
    kdtree* left_child;
    kdtree* right_child;
    //分割轴方向
    int axis;
    //数据点在原来矩阵中的索引
    int point_indices;
    //默认构造函数
    kdtree() : parent(nullptr), left_child(nullptr), right_child(nullptr), axis(-1){}

    //判断节点是否为空,若为空则返回True
    bool is_empty() const{
        return node.empty();
    }

    //判断当前的节点是否是一个Leaf节点;即本身不为空但没有任何子节点
    bool is_leaf() const{
        return (!node.empty()) && right_child == nullptr && left_child == nullptr;
    }

    //判断当前节点是否是树的根节点
    bool is_root() const{
        return (!is_empty()) && parent == nullptr;
    }

    //判断当前节点是不是其父节点的左子节点
    bool is_left() const{
        return parent->left_child->node == node;
    }

    //判断当前节点是不是其父节点的右子节点
    bool is_right() const{
        return parent->right_child->node == node;
    }

};
//数据结构：保存点的索引和距离
class kd_DistIndex{
public:
    double distance;
    size_t index;
    kd_DistIndex(double dist,size_t idx){
        distance = dist;
        index = idx;
    }
    //拷贝构造函数
    kd_DistIndex( const DistIndex &rhs ){
//        cout << "拷贝构造函数" << endl;
        distance = rhs.distance;
        index = rhs.index;
    }
    bool lt(DistIndex self, DistIndex other){
        return self.distance < other.distance;
    }
};
//KNN搜索结果数据集，输入：capacity——需要搜索的点的个数
class kd_KnnResultSet{
public:
    //capacity——需要搜索的点的个数
    size_t capacity;
    int count;
    double worst_dist;
    vector<DistIndex> dist_index_list;
    int comparison_counter;
    //构造函数
    kd_KnnResultSet(size_t cap):capacity(cap),count(0),worst_dist(1e10),comparison_counter(0){
        for(int i = 0; i < cap; ++i){
            dist_index_list.push_back(DistIndex(worst_dist, 0));
        }
    }

    int size() const{ return count;}
    bool full() const{ return count==capacity;}
    double worstDist() const { return worst_dist;}

    //插入点
    void add_point(double dist, size_t index){
        comparison_counter += 1;
        if( dist > worst_dist){
            cout << "大于worst distance。" << endl;
            return;
        }
        if (count < capacity){
            count += 1;
//            cout << "capacity" << capacity << endl;
//            cout << "count" << count << endl;
        }
        int i = count - 1;
        while(i > 0){
            if (dist_index_list[i-1].distance > dist){//从后往前，已有的距离大于传入的距离的话
                dist_index_list[i] = dist_index_list[i-1];//把已有距离向后复制一位
                i -= 1;
            }
            else{
                break;
            }
        }

        //插入符合条件的DistIndex
        dist_index_list[i].distance = dist;
        dist_index_list[i].index = index;
        //worst distance就是dist-index列表中最后一个的距离
        worst_dist = dist_index_list[capacity-1].distance;
    }

    void display(){
        cout << "当前的目标点的近邻点的索引和距离为：" << endl;
        for (int i = 0; i < dist_index_list.size(); ++i) {
            cout << "索引： " << dist_index_list[i].index;
            cout << "    距离： " << dist_index_list[i].distance << endl;
        }
    }
};
//转置一个矩阵
template <typename T>
vector<vector<T> > transpose(vector<vector<T> > Matrix){
    unsigned row = Matrix.size();
    unsigned col = Matrix[0].size();

    vector<vector<T> > Trans(col, vector<T>(row, 0));   //声明了一个vector，一共col项，每一项都是一个vector，该vector中共row项，且全是0
    for (int i = 0; i < col; ++i){
        for(int j = 0; j < row; ++j){
            Trans[i][j] = Matrix[j][i];
        }
    }

    return Trans;
}

//查找数组的中位数
template <typename T>
T find_middle_value(vector<T> vec)
{
    sort(vec.begin(), vec.end());
    auto pos = vec.size() / 2;
    return vec[pos];
}

//构建KDtree
void build_kd_tree(kdtree* tree, vector<vector<double> > data, unsigned depth){
    //样本数量
    unsigned samples_num = data.size();

    //终止条件
    if (samples_num == 0)//没有数据
    {
//        cerr << "No Data!" << endl;
        return;
    }
    if (samples_num == 1)//Leaf的情况
    {
        tree->node = data[0];
//        tree->point_indices = i;
        return;
    }

    //样本的维度
    unsigned dimension = data[0].size();
    vector<vector<double> > trans_data = transpose(data);

    //选择要切分的坐标轴/属性,depth是什么？？？
    unsigned split_attribute = depth % dimension;
    //在点云的情况下，tans_data里面一共有三个vector，各自存储了x,y,z方向的所有数值
    vector<double> split_attribute_values = trans_data[split_attribute];

    //选择切分值
    double splitValue = find_middle_value(split_attribute_values);

    //根据选定的切分属性和切分值，将数据集分为两个子集
    vector<vector<double> > subset1;
    vector<vector<double> > subset2;
    for (unsigned i = 0; i < samples_num; ++i){
        //如果第i个点的当前切分坐标轴的值等于切分值，且node为空，就将第i个点的坐标放入节点中;将分割轴属性赋给该点的axis变量
        if (split_attribute_values[i] == splitValue && tree->node.empty()){
            tree->node = data[i];
            tree->axis = split_attribute;
//            tree->point_indices =

        }
        else{
            if (split_attribute_values[i] < splitValue){
                subset1.push_back(data[i]);
            }
            else{
                subset2.push_back(data[i]);
            }
        }
    }

    //子集递归调用build_kd_tree函数
    //申请一段新的内存，并让left_child指向它
    tree->left_child = new kdtree;
    //当前的tree即节点就成了左子节点的父节点
    tree->left_child->parent = tree;
    //申请一段新的内存，并让right_child指向它
    tree->right_child = new kdtree;
    //当前的tree即节点就成了右子节点的父节点
    tree->right_child->parent = tree;
    //目前看来depth用于让三个轴轮流成为切分轴。
    build_kd_tree(tree->left_child, subset1, depth + 1);
    build_kd_tree(tree->right_child, subset2, depth + 1);
}

//逐层打印kd树
void print_kd_tree(kdtree *tree, unsigned depth){
    for (unsigned i = 0; i < depth; i++){
        cout << "\t";
    }
    //c++11 ":" 按序循环,打印node中的数值，在点云的情况中，就是打印三个坐标。
    for (double j : tree->node){
        cout << j << ",";
    }
    cout << " axis = " << tree->axis << " point_indices = " << tree->point_indices;
    cout << endl;

    //如果左子节点和右子节点都是空指针，则当前节点是一个leaf
    if (tree->left_child == nullptr && tree->right_child == nullptr){
        return;
    }
    //若不是Leaf，就先打印它的左子节点，再打印它的右子节点。总体呈现方法是root， leaf， right顺序
    else{
        if (tree->left_child != nullptr){
            for(unsigned i = 0; i < depth + 1; i++){
                cout << "\t";
            }
            cout << "left:";
            print_kd_tree(tree->left_child, depth + 1);
        }
        cout << endl;

        if (tree->right_child != nullptr){
            for(unsigned i = 0; i < depth + 1; i++){
                cout << "\t";
            }
            cout << "right:";
            print_kd_tree(tree->right_child, depth + 1);
        }
        cout << endl;
    }
}

//计算空间中两个点的距离
double measure_distance(vector<double> point1, vector<double> point2, unsigned method){
    if (point1.size() != point2.size()){
        cerr << "Dimensions don't match!!";
        return -1;
    }
    switch (method){
        //欧氏距离
        case 0:{
            double res = 0;
            //确保i不会溢出
            for (vector<double>::size_type i = 0; i < point1.size(); i++){
                //(x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2
                res += pow((point1[i] - point2[i]), 2);
            }
            return sqrt(res);//开根号得到距离
        }
        //曼哈顿距离
        case 1:{
            double res = 0;
            for (vector<double>::size_type i = 0; i < point1.size(); i++){
                //|x1 - x2| + |y1 - y2| + |z1 - z2|
                res += abs(point1[i] - point2[i]);
            }
            return res;
        }
        default:
        {
            cerr << "Invalid method!!" << endl;
            return -1;
        }
    }
}







//knn搜索
//输入：
//  tree：KD树
//  pointset：原始数据
//  result_set:搜索结果
//  query：要搜索近邻的目标对象
bool kdtree_knn_search(kdtree *tree, Eigen::MatrixXd pointset, kd_KnnResultSet &resultset, Eigen::VectorXd query){
//    cout << "递归了一次" << endl;
    if (tree->is_empty())
        return false;
    //如果当前节点是一个Leaf,那么比较该Leaf中的每一个数据点
//    int i = 0;
    if (!tree->is_empty() ){
//        k : tree->node;
        Eigen::VectorXd leaf_points;
        leaf_points.resize(pointset.cols());
        for (int i = 0; i < leaf_points.rows(); ++i) {
            leaf_points[i] = tree->node[i];
        }
//        cout << "leaf_points: " << leaf_points;
        double dist = (query - leaf_points).norm();
        bool equal = false;
        for (int i = 0; i < pointset.rows(); ++i) {
            for (int j = 0; j < pointset.cols(); ++j) {
                if(pointset(i, j) == leaf_points(j)){
                    equal = true;
                }
                else{
                    equal = false;
                    break;
                }
            }
            if (equal == true){
                tree->point_indices = i;
                break;
            }
        }
        resultset.add_point(dist, tree->point_indices);
//        i++;
//        return false;
    }

    //判断先从分割轴的哪一侧开始搜索,对于当前tree而言，如果query在它分割轴的左边
    if(tree->axis != -1) {
        if (query[tree->axis] <= tree->node[tree->axis] && !tree->left_child->is_empty()) {
            kdtree_knn_search(tree->left_child, pointset, resultset, query);
            //判断是否需要搜索另外一侧
            if (abs(query[tree->axis] - tree->node[tree->axis]) < resultset.worstDist() && !tree->right_child->is_empty()) {
                kdtree_knn_search(tree->right_child, pointset, resultset, query);
            }
        }
        else if(query[tree->axis] > tree->node[tree->axis] && !tree->right_child->is_empty()) {
            kdtree_knn_search(tree->right_child, pointset, resultset, query);
            //判断是否需要搜索另外一侧
            if (abs(query[tree->axis] - tree->node[tree->axis]) < resultset.worstDist() && !tree->left_child->is_empty()) {
                kdtree_knn_search(tree->left_child, pointset, resultset, query);
            }
        }
    }
    return false;
}




//在kdtree中搜索当前点的最近邻
//输入：当前点，已经构造好的kd树
//输出：目标点的最近邻
vector<double> search_nearest_neighbor(vector<double> goal_point, kdtree *tree){
    /*第一步：在kd树中找出包含目标点的叶子结点Leaf：
     * 从根结点出发，递归的向下访问kd树，
     * 若当前目标点的当前维度的坐标小于切分点的坐标，
     * 则移动到左子结点，否则移动到右子结点，
     * 直到子结点为叶结点为止,以此叶子结点Leaf为“当前最近点”——该点自己
    */
    //数据的维度数量，点云为三维
    unsigned k = tree->node.size();
    //将当前维度初始化为0，从第一维度开始
    unsigned d = 0;
    kdtree* current_tree = tree;
    vector<double> current_nearest = current_tree->node;//根节点
    //直到搜索到一个Leaf为止，一直循环
    while(!current_tree->is_leaf()){
        unsigned index = d % k;//结果必然为0，1，2，...，k之一，用于计算当前维度/深度
        if (current_tree->right_child->is_empty() || goal_point[index] < current_nearest[index]){
        //当目前的节点没有右子节点或者目标点在该当前维度上的数值较当前节点小的时候，
        //就应当将树移到左子节点上去看
            current_tree = current_tree->left_child;
        }
        else{
        //相反，就应当将树移到右子节点上去看
            current_tree = current_tree->right_child;
        }
        //换一个坐标轴
        d++;
    }
    current_nearest = current_tree->node;

    /*第二步：递归地向上回退， 在每个结点进行如下操作：
    (a)如果该结点保存的实例比当前最近点距离目标点更近，则以该例点为“当前最近点”
    (b)当前最近点一定存在于某结点一个子结点对应的区域，检查该子结点的父结点的另
    一子结点对应区域是否有更近的点（即检查另一子结点对应的区域是否与以目标点为球
    心、以目标点与“当前最近点”间的距离为半径的球体相交）；如果相交，可能在另一
    个子结点对应的区域内存在距目标点更近的点，移动到另一个子结点，接着递归进行最
    近邻搜索；如果不相交，向上回退*/

    //计算当前最近邻和目标点的距离（欧氏），获得worst distance
    double current_distance = measure_distance(goal_point, current_nearest, 0);

    //如果当前kd树的根节点是其父节点的左子节点，则搜索其父节点的右子节点所代表的区域
    //如果当前kd树的根节点是其父节点的右子节点，则搜索其父节点的左子节点所代表的区域
    kdtree* search_district;
    if (current_tree->is_left()){
        if (current_tree->parent->right_child == nullptr)//若其父节点没有右子节点
            search_district = current_tree;
        else
            search_district = current_tree->parent->right_child;
    }
    else{
        if (current_tree->parent->left_child == nullptr)//若其父节点没有左子节点
            search_district = current_tree;
        else
            search_district = current_tree->parent->left_child;
    }

    //如果搜索区域对应的node不是整个kd树的root，就继续回退搜索,搜索到整个kdtree的根节点为止
    while(search_district->parent!= nullptr){
        //目标点到分界线的垂直距离。搜索区域与目标点的最近距离????轴的选定看不懂，感觉应该是d + data-dimenion - 1
        double district_distance = abs( goal_point[search_district->axis] - search_district->parent->node[search_district->axis] );
        //如果“搜索区域与目标点的最近距离”比“当前最近邻与目标点的距离”worst distance短，表明搜索区域内可能存在距离目标点更近的点
        if (district_distance < current_distance){
            double parent_distance = measure_distance(goal_point, search_district->parent->node, 0);
            if (parent_distance < current_distance){//如果到分界线点的直线距离小于worst distance，最近邻就换成分界线点
                current_distance = parent_distance;
                current_tree = search_district->parent;
                current_nearest = current_tree->node;
            }
            if (!search_district->is_empty()){//搜索区域是否满足要求小于worst distance
                double root_distance = measure_distance(goal_point, search_district->node, 0);
                if (root_distance < current_distance){
                    current_distance = root_distance;
                    current_tree = search_district;
                    current_nearest = current_tree->node;
                }
            }
            if (search_district->left_child != nullptr){//如果搜索区域有左子节点，判断是否小于worst distance
                double left_distance = measure_distance(goal_point, search_district->left_child->node, 0);
                if (left_distance < current_distance){
                    current_distance = left_distance;
                    current_tree = search_district;
                    current_nearest = current_tree->node;
                }
            }
            if (search_district->right_child != nullptr){//如果搜索区域有右子节点，判断是否小于worst distance
                double right_distance = measure_distance(goal_point, search_district->right_child->node, 0);
                if (right_distance < current_distance){
                    current_distance = right_distance;
                    current_tree = search_district;
                    current_nearest = current_tree->node;
                }
            }
        }//end if

        if (search_district->parent->parent != nullptr )//若搜索区域的父节点有父节点
        {
            //那么如果搜索区域的父节点是搜索区域父父节点的左子节点，就搜索父父节点的右子节点，反之则搜索父父节点的左子节点
            search_district = search_district->parent->is_leaf()?
                              search_district->parent->parent->right_child:
                              search_district->parent->parent->left_child;
        }
        else{
            search_district = search_district->parent;
        }
        d++;
    }
    return current_nearest;
}
#endif //POINT_CLOUD_PROCESS_SL_KDTREE_H
