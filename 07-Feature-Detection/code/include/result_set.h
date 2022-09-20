//
// Created by pyh on 2022/8/2.
//
#include <iostream>
#ifndef POINT_CLOUD_PROCESS_SL_RESULT_SET_H
#define POINT_CLOUD_PROCESS_SL_RESULT_SET_H
//数据结构：保存点的索引和距离
class DistIndex{
public:
    double distance;
    size_t index;
    DistIndex(double dist,size_t idx){
        distance = dist;
        index = idx;
    }
    //拷贝构造函数
    DistIndex( const DistIndex &rhs ){
//        cout << "拷贝构造函数" << endl;
        distance = rhs.distance;
        index = rhs.index;
    }
    bool lt(DistIndex self, DistIndex other){
        return self.distance < other.distance;
    }
    void dis_DistIndex(){
        std::cout << "index of neighbour = " << index << std::endl;
        std::cout << "distance = " << distance << std::endl;
    }

};



//KNN搜索结果数据集，输入：capacity——需要搜索的点的个数
class KnnResultSet{
public:
    //capacity——需要搜索的点的个数
    size_t capacity;
    int count;
    double worst_dist;
    std::vector<DistIndex> dist_index_list;
    int comparison_counter;
    //构造函数
    KnnResultSet(size_t cap):capacity(cap),count(0),worst_dist(1e10),comparison_counter(0){
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
            std::cout << "大于worst distance。" << std::endl;
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
        std::cout << "当前的目标点的近邻点的索引和距离为：" << std::endl;
        for (int i = 0; i < dist_index_list.size(); ++i) {
            std::cout << "索引： " << dist_index_list[i].index;
            std::cout << "    距离： " << dist_index_list[i].distance << std::endl;
        }
    }
};


//RadiusNN搜索结果数据集，输入：Radius——需要搜索的半径
class RadiusNNResultSet{
public:
    RadiusNNResultSet(double radius) : radius(radius),count(0),worst_distance(radius),comparison_counter(0){}


    std::vector<DistIndex> dist_index() const{
        return dist_index_list;
    }

    int size() const{
        return count;
    }

    double worstDist() const{
        return radius;
    }
    double get_worstDist() const{
        return worst_distance;
    }

    void add_point(double dist, size_t index){
//        std::cout << "call add_point" << std::endl;
        comparison_counter += 1;
        if (dist > radius){
            return;
        }
        count += 1;
        dist_index_list.push_back(DistIndex(dist, index));
    }

    void display(){
        std::cout << "当前的目标点的近邻点的索引和距离为：" << std::endl;
        for (int i = 0; i < dist_index_list.size(); ++i) {
            std::cout << "索引： " << dist_index_list[i].index;
            std::cout << "    距离： " << dist_index_list[i].distance << std::endl;
        }
    }

private:
    double radius;
    int count;
    double worst_distance;
    int comparison_counter;
    std::vector<DistIndex> dist_index_list;
};
#endif //POINT_CLOUD_PROCESS_SL_RESULT_SET_H
