#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/thread/mutex.hpp>
#include <pcl/filters/extract_indices.h>
#include <time.h>

#include <thread>
#include <chrono>
using namespace std;


static double global_angular_threshold;
static double global_distance_threshold;

static void setCondifionFunctionParameter(
      const double angular_threshold,
      const double distance_threshold)
    {
      global_angular_threshold = angular_threshold;
      global_distance_threshold = distance_threshold;
    }


float max_curvature_ = 0.1;
float cluster_tolerance_ = 0.1;
int min_size_ = 100;

// クラスタリング時の閾値。
float angular_threshold_ = 0.04;
float distance_threshold_ = 0.002;


static bool regionGrowingFunction(const pcl::PointXYZRGBNormal& a,
                                      const pcl::PointXYZRGBNormal& b,
                                      float distance)
    {
      if (distance > global_distance_threshold) {
        return false;
      }
      else {
        Eigen::Vector3f a_normal(a.normal_x, a.normal_y, a.normal_z);
        Eigen::Vector3f b_normal(b.normal_x, b.normal_y, b.normal_z);
        double dot = std::abs(a_normal.dot(b_normal));
        double angle;
        if (dot > 1.0) {
          angle = acos(1.0);
        }
        else if (dot < -1.0) {
          angle = acos(-1.0);
        }
        else {
          angle = acos(dot);
        }
        //ROS_INFO("angle: %f", angle);
        if (angle > global_angular_threshold) {
          return false;
        }
        else {
          return true;
        }
      }
    }





int main()
{   
  // 作成したPointCloudを読み込む
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile("main2.pcd", *rgb_cloud);

  //フィルター処理開始
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  /*
  // Approximate Voxel Gridを使う場合
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> avg;
  avg.setInputCloud(rgb_cloud);
  avg.setLeafSize(0.0001f, 0.0001f, 0.0001f);
  avg.filter(*cloud_filtered);
  */

  //Voxel Gridを利用する場合
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setInputCloud(rgb_cloud);
  float leaf_size_ = 0.004;     // サンプリング度合いを決める変数。大きくするとよりサンプリングする。
  voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxelGrid.filter(*cloud_filtered);

    //法線推定処理開始
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.01);
  std::cout << "calc" << std::endl;
  ne.compute (*cloud_normals);

    //法線とXYZRGB点群を連結
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr all_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud_filtered, *cloud_normals, *all_cloud);


  //点群のうち、一定曲率以下のもののみ抽出する
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  for (size_t i = 0; i < all_cloud->points.size(); i++) {
    if (!std::isnan(all_cloud->points[i].x) &&
        !std::isnan(all_cloud->points[i].y) &&
        !std::isnan(all_cloud->points[i].z) &&
        !std::isnan(all_cloud->points[i].normal_x) &&
        !std::isnan(all_cloud->points[i].normal_y) &&
        !std::isnan(all_cloud->points[i].normal_z) &&
        !std::isnan(all_cloud->points[i].curvature)) {
      if (all_cloud->points[i].curvature < max_curvature_) {
        indices->indices.push_back(i);
      }
    }
  }
    std::cout << "sampling (curv)" << std::endl;
  pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
  std::vector<pcl::PointIndices> clusters;
    
  cec.setInputCloud (all_cloud);
  cec.setIndices(indices);
  cec.setClusterTolerance(cluster_tolerance_);
  cec.setMinClusterSize(min_size_);
  setCondifionFunctionParameter(angular_threshold_, distance_threshold_);
  cec.setConditionFunction(&regionGrowingFunction);
  std::cout << "clustering start" << std::endl;
  
  clock_t start = clock();
  cec.segment (clusters);
  clock_t end = clock();
      std::cout << "clustering end" << std::endl;
      std::cout << "time = " << (double)(end-start) / CLOCKS_PER_SEC << " sec" << std::endl;

  //クラスタごとに分割し、segmented_clusterに格納
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> ei;
  ei.setInputCloud(all_cloud);
  ei.setNegative(false);
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> segmented_cluster;
  for (size_t i=0; i<clusters.size();i++){

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
    *tmp_clustered_indices = clusters[i];
    ei.setIndices(tmp_clustered_indices);
    ei.filter(*tmp_clustered_points);
    segmented_cluster.push_back(tmp_clustered_points);
  }

    //セグメントごとに色分け
  double rgb[3] = {};
  const int channel = 3;	//RGB
  const double step = ceil(pow(segmented_cluster.size()+2, 1.0/(double)channel));	//exept (000),(111)
  const double max = 1.0;

  pcl::visualization::PCLVisualizer viewer {"Conditional Euclidian Clustering"};

    for(size_t i=0;i<segmented_cluster.size();i++){
      std::string name = "cluster_" + std::to_string(i);
      rgb[0] += 1/step;
      for(int j=0;j<channel-1;j++){
        if(rgb[j]>max){
          rgb[j] -= max + 1/step;
          rgb[j+1] += 1/step;
        }
      }

      //viewer処理
      
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "cloud");
      viewer.addPointCloud<pcl::PointXYZRGBNormal>(segmented_cluster[i], name);
        //viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(segmented_cluster[i], 1, 0.5, name);
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, name);
    }
std::cout << "cluster num = " << clusters.size() << std::endl;
  while (!viewer.wasStopped())
  {
    viewer.spin();
    //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
    
  }

  return 0;

}  