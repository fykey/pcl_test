#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>  // 追加
#include <thread>
#include <chrono>
#include <X11/Xlib.h>  // 追加

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path-to-pcd-file>" << std::endl;
        return EXIT_FAILURE;
    }

     XInitThreads();

    // 引数からPCDファイルのパスを取得
    std::string pcd_file_path = argv[1];

    // PCL PointCloudオブジェクトを作成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // PCDファイルの読み込み
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", pcd_file_path.c_str());
        return EXIT_FAILURE;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcd_file_path << std::endl;

    // Voxel Grid Downsamplingの設定
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);  // ボクセルのサイズを設定 (1cm^3のボクセル)
    sor.filter(*cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points" << std::endl;

    // PCL Visualizerオブジェクトを作成
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // ポイントクラウドを表示
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, rgb, "sample cloud");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // ビューワーのメインループ
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        printf("test");
    }

    return EXIT_SUCCESS;
}
