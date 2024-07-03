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

int main(int argc, char* argv[])
{
    // 引数の解析
    std::string depth_image_path, color_image_path;
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--depth" && i + 1 < argc)
        {
            depth_image_path = argv[++i];
        }
        else if (std::string(argv[i]) == "--color" && i + 1 < argc)
        {
            color_image_path = argv[++i];
        }
    }

    if (depth_image_path.empty() || color_image_path.empty())
    {
        std::cerr << "Usage: " << argv[0] << " --depth depth.jpg --color color.jpg" << std::endl;
        return EXIT_FAILURE;
    }

    // カラー画像とデプス画像の読み込み
    cv::Mat color_image = cv::imread(color_image_path, cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);

    if (color_image.empty() || depth_image.empty())
    {
        std::cerr << "Failed to read input images." << std::endl;
        return EXIT_FAILURE;
    }

    // 深度画像をメートル単位のfloat値に変換する（深度スケールは0.001（例））
    float depth_scale = 0.001f;  // これは適宜調整してください
    cv::Mat depth_image_float;
    depth_image.convertTo(depth_image_float, CV_32F, depth_scale);

    // 遠方の点群が邪魔なので、最大Z=0.6で切ってしまう
    util::cutoff_out_of_depth_range(depth_image_float, depth_image_float, 0.2, 0.6);

    cv::imshow("color", color_image);
    cv::imshow("depth", depth_image_float);

    int key = cv::waitKey(0);

    if (key == 27 || key == 'q' || key == 'Q')  // ESCキーまたは q/Qキー で終了
    {
        cv::destroyAllWindows();

        // カメラの内部パラメータを設定（例として、Realsense D435のパラメータ）
        rs2_intrinsics intrinsics;
        intrinsics.width = WIDTH;
        intrinsics.height = HEIGHT;
        intrinsics.fx = 960.0f;  // 例: 焦点距離（適宜調整してください）
        intrinsics.fy = 960.0f;  // 例: 焦点距離（適宜調整してください）
        intrinsics.ppx = 640.0f;  // 例: 主点のx座標（適宜調整してください）
        intrinsics.ppy = 400.0f;  // 例: 主点のy座標（適宜調整してください）
        intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;
        intrinsics.coeffs[0] = 0.1f;  // 例: 歪み係数（適宜調整してください）
        intrinsics.coeffs[1] = 0.0f;
        intrinsics.coeffs[2] = 0.0f;
        intrinsics.coeffs[3] = 0.0f;
        intrinsics.coeffs[4] = 0.0f;

        // 3次元点群を生成して保存
        auto pcl_points = util::frames_to_pcl(color_image, depth_image_float, intrinsics, depth_scale);
        pcl::io::savePCDFile("output_via_frames.pcd", *pcl_points);
    }

    return EXIT_SUCCESS;
}
