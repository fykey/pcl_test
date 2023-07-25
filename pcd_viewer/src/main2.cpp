#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

//#include <boost/thread/thread.hpp>
#include <thread>
#include <chrono>
using namespace std;

// ビューワー起動時の一回だけ呼ばれる
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    // 背景色の設定 
    viewer.setBackgroundColor(0,0,0); //white
    cout << "viewerOneOff" << std::endl;
}

// ビューワー起動中の毎フレーム実行される
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    cout << "viewerPsycho__1111" << std::endl;
}


int main()
{   // set Type

    using CloudType2 = pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;


    CloudType2::Ptr rgb_cloud(new CloudType2);
    // 作成したPointCloudを読み込む
    // pcl::io::loadPCDFile("colored_cloud.pcd", *rgb_cloud);
    pcl::io::loadPCDFile("main2.pcd", *rgb_cloud);

    /*
    // Approximate Voxel Gridを使う場合
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> avg;
    avg.setInputCloud(rgb_cloud);
    avg.setLeafSize(0.0001f, 0.0001f, 0.0001f);
    avg.filter(*cloud_filtered);
    */

    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
    voxelGrid.setInputCloud(rgb_cloud);
    float leaf_size_ = 0.001;
    // set the leaf size (x, y, z)
    voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    // apply the filter to dereferenced cloudVoxel
    voxelGrid.filter(*cloud_filtered);

    ne.setInputCloud (cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.01);
    std::cout<<"test"<<std::endl;
    ne.compute (*cloud_normals);
std::cout<<"test"<<std::endl;
    pcl::visualization::PCLVisualizer viewer("PointCloudViewer");
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_filtered,cloud_normals, 100, 0.05, "normals");
std::cout<<"test"<<std::endl;
    viewer.addPointCloud(cloud_filtered);
    
    // ビューワー視聴用ループ
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
        std::cout<<"test"<<std::endl;
    }
    return 0;
    
}  