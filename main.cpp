#include <iostream>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/lccp_segmentation.h>

#include <util.h>


#define WIDTH  640
#define HEIGHT 480
#define FPS    30


int main(int argc, char* argv[])
{
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
    config.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
    rs2::pipeline pipe;
    pipe.start(config);

    rs2::colorizer color_map;
    rs2::align align(RS2_STREAM_COLOR);

    for (int i = 0; i < 3; i++)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        cv::waitKey(10);
    }

    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        auto aligned_frames = align.process(frames);
        rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::video_frame depth_frame = aligned_frames.get_depth_frame();

        float depth_scale = ((rs2::depth_frame)depth_frame).get_units();

        auto profile = depth_frame.get_profile();
        auto stream_profile = profile.as<rs2::video_stream_profile>();
        auto intrinsics = stream_profile.get_intrinsics();

        rs2::pointcloud pc;
        pc.map_to(color_frame);
        rs2::points points = pc.calculate(depth_frame);

        // cv::Mat color_image(cv::Size(WIDTH, HEIGHT), CV_64FC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat color_image(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(WIDTH, HEIGHT), CV_16U , (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // デプスをメートル単位のfloat値に変換する
        cv::Mat depth_image_float;
        depth_image.convertTo ( depth_image_float, CV_32F, depth_scale);

        // 遠方の点群が邪魔なので、最大Z=0.6で切ってしまう
        util::cutoff_out_of_depth_range ( depth_image_float, depth_image_float, 0.2, 0.6 );

        cv::imshow("color", color_image);
        cv::imshow("depth", depth_image_float);

        int key = cv::waitKey(10);

        if (key == 27 || key == 'q' || key == 'Q')  // ESCキーまたは q/Qキー で終了
        {
            cv::destroyAllWindows;

            // ３次元点群を生成して保存 --> CloudCompareで3D可視化可能 
            // {
            //     auto pcl_points = util::points_to_pcl(points, color_frame);
            //     pcl::io::savePCDFile("output_via_points.pcd", * pcl_points);
            // }
            {
                auto pcl_points = util::frames_to_pcl(color_image, depth_image_float, intrinsics, depth_scale);
                pcl::io::savePCDFile("output_via_frames.pcd", * pcl_points);
            }

            break;
        }

    }
    pipe.stop();

    return EXIT_SUCCESS;
}
