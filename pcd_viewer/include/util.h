#pragma once


#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
// #include <pcl/common/impl/io.hpp>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/lccp_segmentation.h>
#include <iostream>


namespace util
{

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);

// librealsense の points と video_frame(color) から PCLの３次元点群を生成する
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);

// OpenCVの color_image / depth_image / と realsenseのintrinsics からPCLの３次元点群を生成する
pcl::PointCloud<pcl::PointXYZRGB>::Ptr frames_to_pcl(cv::Mat color_image, cv::Mat depth_image_float, rs2_intrinsics intrinsics, float depth_scale);


// ---------- postprocess for depth

void cutoff_out_of_depth_range(cv::InputArray src, cv::OutputArray dst, float range_min, float range_max);
void median_filter(cv::InputArray src, cv::OutputArray dst, int kernel_size);
void cutoff_by_depth_gradient(cv::InputArray src, cv::OutputArray dst, float gradient_threshold, int edge_dilation);
void bilateral_filter(cv::InputArray src, cv::OutputArray dst, int diameter, double sigma_depth, double sigma_space);
void crop_area(cv::InputArray src, cv::OutputArray dst, int min_x, int min_y, int max_x, int max_y);


}
