#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/impl/io.hpp>

using namespace std;

// ビューワー起動時の一回だけ呼ばれる
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    // 背景色の設定 
    viewer.setBackgroundColor(1, 1, 1); //white
    cout << "viewerOneOff" << std::endl;
}

// ビューワー起動中の毎フレーム実行される
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    cout << "viewerPsycho" << std::endl;
}


int main()
{   // set Type
    using CloudType = pcl::PointCloud<pcl::PointXYZ>;
    using CloudType2 = pcl::PointCloud<pcl::PointXYZRGB>;

    CloudType::Ptr original_cloud(new CloudType);
    CloudType2::Ptr rgb_cloud(new CloudType2);

    // 作成したPointCloudを読み込む
    pcl::io::loadPCDFile("output_via_frames.pcd", *original_cloud);

    // copy p_cloud data into rgb_cloud
    copyPointCloud(*original_cloud, *rgb_cloud);

    // add RGB data to rgb_cloud
    for (int h = 0; h < rgb_cloud->height; h++) {
            for (int w = 0; w < rgb_cloud->width; w++) {
                    pcl::PointXYZRGB &point = rgb_cloud->points[w + h * rgb_cloud->width];
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
            }
    }

    // ビューワーの作成
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    viewer.showCloud(rgb_cloud);
    
    // ビューワー起動時の一回だけ呼ばれる関数をセット
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    
    // ビューワー起動中の毎フレーム実行される関数をセット
    viewer.runOnVisualizationThread(viewerPsycho);
    
    // ビューワー視聴用ループ
    while (!viewer.wasStopped())
    {
    
    }
    return 0;
}  