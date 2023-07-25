#include <util.h>


namespace util
{

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();

    auto Texture_Coord = points.get_texture_coordinates();

    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        auto& p = cloud->points[i];

        p.x = ptr[i].x;
        p.y = ptr[i].y;
        p.z = ptr[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        p.r = get<2>(RGB_Color); // Reference tuple<2>
        p.g = get<1>(RGB_Color); // Reference tuple<1>
        p.b = get<0>(RGB_Color); // Reference tuple<0>
    }

    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr frames_to_pcl(cv::Mat color_image, cv::Mat depth_image_float, rs2_intrinsics intrinsics, float depth_scale)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // PointCloudの大きさを決定
    p_cloud->width  = color_image.cols;
    p_cloud->height = color_image.rows;
    p_cloud->points.resize(p_cloud->width * p_cloud->height);

    // 直接、値を入力してPointCloudを作成
    for (int h = 0; h < p_cloud->height; h++)
    {
        for (int w = 0; w < p_cloud->width; w++)
        {
            pcl::PointXYZRGB &point = p_cloud->points[w + h * p_cloud->width];
            point.z = depth_image_float.at<float>(h, w);
            point.x = (w - intrinsics.ppx) / intrinsics.fx * point.z;
            point.y = (h - intrinsics.ppy) / intrinsics.fy * point.z;
            cv::Vec3b c = color_image.at<cv::Vec3b>(h, w);
            point.r = c[2];
            point.g = c[1];
            point.b = c[0];
        }
    }

    return p_cloud;
}


// -------- postprocess for depth

void cutoff_out_of_depth_range(cv::InputArray src, cv::OutputArray dst, float range_min, float range_max)
{
    CV_Assert( src.type() == CV_32FC1 );

    cv::Mat _src = src.getMat();
    dst.create(src.size(), src.type());
    cv::Mat _dst = dst.getMat();

    _src.copyTo(_dst);

    _dst.setTo(0.0, _dst > range_max);
    _dst.setTo(0.0, _dst < range_min);
}


void median_filter(cv::InputArray src, cv::OutputArray dst, int kernel_size)
{
    CV_Assert( src.type() == CV_32FC1 );

    cv::Mat _src = src.getMat();
    dst.create(src.size(), src.type());
    cv::Mat _dst = dst.getMat();

    if (kernel_size == 3 || kernel_size == 5)
    {
        cv::medianBlur(_src, _dst, kernel_size);
    }
    else
    {
        std::cout << "Error kernel size of median blur (only support 3 or 5)!" << std::endl;
    }
}


void cutoff_by_depth_gradient(cv::InputArray src, cv::OutputArray dst, float gradient_threshold, int edge_dilation)
{
    CV_Assert( src.type() == CV_32FC1 );

    cv::Mat _src = src.getMat();
    dst.create(src.size(), src.type());
    cv::Mat _dst = dst.getMat();

    cv::Mat sobel_x, sobel_y, edge, thresh, dilated;

    cv::Sobel(_src, sobel_x, CV_32F, 1, 0, 3);
    cv::Sobel(_src, sobel_y, CV_32F, 0, 1, 3);
    cv::Mat sobel_x_x;
    cv::multiply(sobel_x, sobel_x, sobel_x_x);
    cv::Mat sobel_y_y;
    cv::multiply(sobel_y, sobel_y, sobel_y_y);
    // cv::sqrt(sobel_x * sobel_x + sobel_y * sobel_y, edge);
    cv::sqrt(sobel_x_x + sobel_y_y, edge);
    cv::threshold(edge, thresh, gradient_threshold, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(edge_dilation, edge_dilation));
    cv::dilate(thresh, dilated, kernel);

    // [TODO] adjust with pytoroboeye

    _src.copyTo(_dst);
    _dst.setTo(0.0, dilated > 0);
}


void bilateral_filter(cv::InputArray src, cv::OutputArray dst, int diameter, double sigma_depth, double sigma_space)
{
    CV_Assert( src.type() == CV_32FC1 );

    cv::Mat _src = src.getMat();
    dst.create(src.size(), src.type());
    cv::Mat _dst = dst.getMat();

    cv::bilateralFilter(_src, _dst, diameter, sigma_depth, sigma_space);
}


void crop_area(cv::InputArray src, cv::OutputArray dst, int min_x, int min_y, int max_x, int max_y)
{
    CV_Assert( src.type() == CV_32FC1 );

    cv::Mat _src = src.getMat();
    dst.create(src.size(), src.type());
    cv::Mat _dst = dst.getMat();

    _src.copyTo(_dst);

    for (int i = 0; i < _dst.rows; ++i)
    {
        if (i < min_y || i > max_y)
        {
            _dst.row(i).setTo(0.0);
        }
    }
    for (int i = 0; i < _dst.cols; ++i)
    {
        if (i < min_x || i > max_x)
        {
            _dst.col(i).setTo(0.0);
        }
    }
}

} // namespace util