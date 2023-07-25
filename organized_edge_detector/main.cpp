#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <set>

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal N;
typedef pcl::Label L;

bool use_nan_boundary_ = false;
bool use_occluding_ = true;
bool use_occluded_ = true;
bool use_curvature_ = true;
bool use_rgb_ = false;

float depth_discontinuation_threshold_ = 0.01; // 仮の値を代入

std::vector<int> addIndices(const std::vector<int>& a,
                            const std::vector<int>& b)
{
   std::set<int> all(b.begin(), b.end());
   for(size_t i=0; i < a.size(); i++){
    all.insert(a[i]);
   }
   return std::vector<int>(all.begin(), all.end());
}

void estimateEdge(const pcl::PointCloud<PointT>::Ptr &input,
                  const pcl::PointCloud<N>::Ptr &normal,
                  pcl::PointCloud<L>::Ptr &output,
                  std::vector<pcl::PointIndices> &label_indices)
{
  pcl::OrganizedEdgeFromRGBNormals<PointT, N, L> oed;
  oed.setDepthDisconThreshold(depth_discontinuation_threshold_);
  oed.setMaxSearchNeighbors(100);
  int flags = 0;
  if (use_nan_boundary_) {
    flags |= oed.EDGELABEL_NAN_BOUNDARY;
  }
  if (use_occluding_) {
    flags |= oed.EDGELABEL_OCCLUDING;
  }
  if (use_occluded_) {
    flags |= oed.EDGELABEL_OCCLUDED;
  }
  if (use_curvature_) {
    flags |= oed.EDGELABEL_HIGH_CURVATURE;
  }
  if (use_rgb_) {
    flags |= oed.EDGELABEL_RGB_CANNY;
  }
  oed.setEdgeType(flags);
  oed.setInputNormals(normal);
  oed.setInputCloud(input);
  oed.compute(*output, label_indices);

}

void estimateStraightEdges(){

}

void estimateNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                    pcl::PointCloud<pcl::Normal>::Ptr normal)
{
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.01);
    ne.setNormalSmoothingSize(5.0);
    ne.setInputCloud(cloud);
    ne.compute(*normal);
}

int main(int argc, char ** argv){
    if (argc != 2){ // 引数の数が 1 ではなく 2 であることを確認します
        pcl::console::print_error ("Syntax is: %s <pcd-file> \n", argv[0]);
        return (1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Label>::Ptr label(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;

    pcl::io::loadPCDFile<PointT> (argv[1], *cloud);

    // 法線ベクトルの推定
    estimateNormal(cloud, normal);
    std::cout << "finished" << std::endl;

    estimateEdge(cloud, normal, label, label_indices);

    ////////////////////////////////////////////////////////
    // build indices includes all the indices
    ////////////////////////////////////////////////////////
    std::vector<int> tmp1 = addIndices(label_indices[0].indices,
                                       label_indices[1].indices);
    std::vector<int> tmp2 = addIndices(tmp1, label_indices[2].indices);
    std::vector<int> tmp3 = addIndices(tmp2, label_indices[3].indices);
    std::vector<int> all = addIndices(tmp3, label_indices[4].indices);
    std::cout << label_indices[4].indices << std::endl;
    /*
     if (use_straightline_detection_) {
      std::vector<std::vector<int> > straightline_indices;
      estimateStraightEdges(cloud, all, msg->header, straightline_indices);
     }
    */

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normal, 100, 0.05, "normals");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud"); // 点群を追加
    //viewer->addPointCloud<pcl::PointXYZ>(label, "labels");
    // 点群の色を設定（オレンジ色）
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 255, 165, 0);
   //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, single_color, "cloud");

    // 可視化の際に点のサイズを設定
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    
    
    while (!viewer->wasStopped())
    {
        viewer->spin(); // 100ms待機してイベントを処理します
    }

    return 0;
}