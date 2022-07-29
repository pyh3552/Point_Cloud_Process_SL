#include "pca_normal.h"
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
bool pca_normal::setInputCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud) {
    m_input = input_cloud;
    return true;
}

void pca_normal::cal_nor(pcl::PointCloud<pcl::PointNormal> &m_output) {
    //如果既设定了搜索半径又设定了k，则报错。
    if (_search_radius != 0.0)
    {
        if (_k != 0)
        {
            printf("Both radius and K defined!,Set one of them to zero first and then re-run compute ().\n");
            exit(10);

        }
    }

    if ( _k != 0) {
        //建立kdtree对象
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(m_input);
        std::vector<int> neighbor_index_k;//领域索引
        std::vector<float> neighbor_square_distance_k;//领域距离大小

        PointT search_point;

        for(size_t i = 0; i < m_input->size(); ++i){
            search_point = m_input->points[i];
            // 创建点云提取对象
            pcl::PointCloud<pcl::PointXYZ>::Ptr extract(new pcl::PointCloud<pcl::PointXYZ>);
            // 创建矩阵对象，将近邻点挑出来变为一个矩阵
            Eigen::MatrixXd ecloudmat;
            if (kdtree.nearestKSearch(search_point, _k, neighbor_index_k, neighbor_square_distance_k) > 0)//寻找领域为K的值
            {
//                extract->setInputCloud(m_input);
                pcl::copyPointCloud(*m_input, neighbor_index_k, *extract);
                //将近邻点挑出来变为一个矩阵
                PointConversionEigen(extract,ecloudmat);
//                Eigen::MatrixXd cloudmat_trans = cloudmat.transpose();

//                Eigen::Matrix<double, 1, 3> ecloudmat_mean = Eigen::VectorwiseOp<Eigen::MatrixXd, Eigen::Vertical>::mean();
                //求出邻域的中心点
                Eigen::RowVector3d ecloudmat_mean = ecloudmat.colwise().mean();
                //均值化为0
                ecloudmat.rowwise() -= ecloudmat_mean;

                Eigen::Matrix3d H = ecloudmat.transpose() * ecloudmat;
                //求H的特征值和特征向量
//                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(H);
                Eigen::EigenSolver<Eigen::Matrix3d> es(H);
                Eigen::Matrix3d Vec = es.pseudoEigenvectors();
                Eigen::Matrix<double, 3, 1> Val = es.eigenvalues().real();
//                Eigen::Matrix3d vec = eig.eigenvectors();
//                Eigen::Matrix3d val = eig.eigenvalues();
                //找到最小特征值的索引并获得相应的特征向量。
                Eigen::MatrixXd::Index minRow, minCol;
                Val.minCoeff(&minRow,&minCol);
                Vec.col(minRow);

                pcl::PointNormal current_point = {search_point.x, search_point.y, search_point.z};
                current_point.normal_x = Vec(0, minRow);
                current_point.normal_y = Vec(1, minRow);
                current_point.normal_z = Vec(2, minRow);
                m_output.push_back(current_point);
            }
        }
    }
}


//功能：点云转换为矩阵
void pca_normal::PointConversionEigen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::MatrixXd &cloudMat)
{
    cloudMat.resize(cloud->points.size(), 3);    //定义点云的行数，3列
    for (int itr = 0; itr < cloud->points.size(); itr++)
    {
        cloudMat(itr, 0) = cloud->points[itr].x;
        cloudMat(itr, 1) = cloud->points[itr].y;
        cloudMat(itr, 2) = cloud->points[itr].z;
    }
}