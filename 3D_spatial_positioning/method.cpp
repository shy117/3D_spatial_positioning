#include "method.h"

Method::Method()
{

}

//BGR->RGB
imgLR Method::bgr2rgb(imgLR img)
{
    imgLR img_rgb;

    cv::cvtColor(img.imgL,img_rgb.imgL,cv::COLOR_BGR2RGB);
    cv::cvtColor(img.imgR,img_rgb.imgR,cv::COLOR_BGR2RGB);
    // std::cout << "BGR->RGB成功" << std::endl;
    return img_rgb;
}

// BGR->gray
imgLR Method::bgr2gray(imgLR img)
{
    imgLR img_gray,img_z;
    //if (img1.dims() == 3) {
    cv::cvtColor(img.imgL,img_gray.imgL,cv::COLOR_BGR2GRAY);
    cv::cvtColor(img.imgR,img_gray.imgR,cv::COLOR_BGR2GRAY);
    // std::cout << "RGB->灰度图" << std::endl;
    return img_gray;
}

// 获取映射变换矩阵、重投影矩阵
void Method::getRectifyTransform(int width, int height, const StereoConfig& config, cv::Mat *left_map1, cv::Mat *left_map2, cv::Mat *right_map1, cv::Mat *right_map2, cv::Mat *Q) {
    // 初始化校正映射
    cv::Mat R1, R2, P1, P2, roi1, roi2;
    cv::Size size(width, height);
    cv::Mat flag;

    // 计算校正变换计算校正变换
    cv::stereoRectify(config.cam_matrix_left, config.distortion_l, config.cam_matrix_right, config.distortion_r,
                        size, config.R, config.T, R1, R2, P1, P2, *Q, cv::CALIB_ZERO_DISPARITY, 0);

    // 初始化未失真和校正映射
    cv::initUndistortRectifyMap(config.cam_matrix_left, config.distortion_l, R1, P1, size, CV_32FC1, *left_map1, *left_map2);
    cv::initUndistortRectifyMap(config.cam_matrix_right, config.distortion_r, R2, P2, size, CV_32FC1, *right_map1, *right_map2);

    std::cout << "获取畸变校正和立体校正的映射变换矩阵、重投影矩阵" << std::endl;
    std::cout << "Q:" << *Q << std::endl;
}

// 图像矫正
imgLR Method::rectifyImage(cv::Mat image1, cv::Mat image2, cv::Mat left_map1, cv::Mat left_map2, cv::Mat right_map1, cv::Mat right_map2)
{
    imgLR rectifyed_img;
    cv::remap(image1, rectifyed_img.imgL, left_map1, left_map2, cv::INTER_AREA);
    cv::remap(image2, rectifyed_img.imgR, right_map1, right_map2, cv::INTER_AREA);
    // std::cout << "图像矫正" << std::endl;
    return rectifyed_img;
}

// 绘制平行线
cv::Mat Method::draw_line(cv::Mat image1, cv::Mat image2) {
    cv::Mat result;
    // 检查图像是否成功加载且大小相同
    if (!image1.empty() && !image2.empty() && image1.size() == image2.size()) {
        // 使用 hconcat 进行横向拼接
        cv::hconcat(std::vector<cv::Mat>{image1, image2}, result);
        std::cout << "横向拼接" << std::endl;
    } else {
        // 处理图像加载失败或大小不匹配的情况
        std::cout << "Error: Images are not loaded correctly or their sizes do not match." << std::endl;
    }
    // 遍历并绘制等间距的平行线
    int line_interval = 50; // 直线间隔
    for (int k = 0; k < image1.rows / line_interval; ++k) {
        cv::Point start(0, line_interval * (k + 1));
        cv::Point end(2 * image1.cols - 1, line_interval * (k + 1)); // 注意：OpenCV的坐标是从0开始的，所以宽度减1
        cv::line(result, start, end, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    std::cout << "平行线绘制" << std::endl;
    return result;
}

// SGBM立体匹配
void Method::stereoMatchSGBM(const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat *disp, cv::Mat *filteredImg, cv::Mat * filter_disp, cv::Mat *filt_Color) {
    int min_disp = 0;
    int num_disp = 128 - min_disp;
    int blockSize = 3;
    int img_channels = 1;
    int P1 = 8 * img_channels * blockSize * blockSize;
    int P2 = 32 * img_channels * blockSize * blockSize;
    int disp12MaxDiff = 1;
    int preFilterCap = 1;
    int uniquenessRatio = 10;
    int speckleWindowSize = 100;
    int speckleRange = 1;
    int mode = cv::StereoSGBM::MODE_HH;

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create( min_disp, num_disp, blockSize,
                                                             P1, P2, disp12MaxDiff, preFilterCap,
                                                             uniquenessRatio, speckleWindowSize, speckleRange,
                                                             mode);
    cv::Ptr<cv::StereoMatcher> stereoR = cv::ximgproc::createRightMatcher(stereo);
    // 用于过滤后的图像
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(stereo);
    wls_filter->setLambda(8000.0);
    wls_filter->setSigmaColor(2.0);

    cv::Mat grayL = left_image, grayR = right_image;

    cv::Mat dispL,dispR;
    stereo->compute(grayL, grayR, *disp);
    disp->convertTo(dispL, CV_16S);

    stereoR->compute(grayR, grayL, dispR);
    dispR.convertTo(dispR, CV_16S);

    // 使用 WLS filter
    wls_filter->filter(dispL, grayL, *filteredImg, dispR);

    filteredImg->convertTo(*filteredImg, CV_8U, 255.0 / (num_disp * 16.0));

    filteredImg->convertTo(*filter_disp, CV_32F, 1.0 / 16.0);
    *filter_disp = (*filter_disp - min_disp) / num_disp;

    // 应用颜色映射
    cv::applyColorMap(*filteredImg, *filt_Color, cv::COLORMAP_OCEAN);
}


cv::Mat Method::hw3ToN3(const cv::Mat& points) {
    int height = points.rows;
    int width = points.cols;
    // std::cout << "height:" <<height << " width:" << width << std::endl;
    cv::Mat points_1 = points.reshape(1, height * width);
    cv::Mat points_2 = points(cv::Rect(0, 0, width, height)).reshape(1, height * width);
    cv::Mat points_3 = points(cv::Rect(0, 0, width, height)).reshape(1, height * width);

    cv::Mat points_;
    cv::hconcat(points_1, points_2, points_);
    cv::hconcat(points_, points_3, points_);

    return points_;
}

// 深度图转换点云图
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Method::DepthColor2Cloud(const cv::Mat& points_3d, const cv::Mat& colors) {
    int size = points_3d.total();

    cv::Mat points_ = hw3ToN3(points_3d);
    cv::Mat colors_float = hw3ToN3(colors);
    cv::Mat colors_int;
    colors_float.convertTo(colors_int, CV_32S);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    // 将颜色信息转换并复制到点云数组
    for (int i = 0; i < size; ++i) {
        pcl::PointXYZRGBA point;
        point.x = points_.at<float>(i, 0);
        point.y = -points_.at<float>(i, 1);
        point.z = points_.at<float>(i, 2);

        int color_val = static_cast<int>(colors_int.at<int>(i, 0)) |
                       (static_cast<int>(colors_int.at<int>(i, 1)) << 8) |
                       (static_cast<int>(colors_int.at<int>(i, 2)) << 16);
        point.rgb = *reinterpret_cast<float*>(&color_val);

        // 删除不合适的点并添加到点云中
        if (point.z > 0.0f && point.z <= 15000.0f  &&
            point.x <= 10000.0f && point.x >= -10000.0f &&
            point.y <= 10000.0f && point.y >= -10000.0f) {
            //point.z = 100.0f;
            pointcloud->push_back(point);
        } 
    }

    return pointcloud;
}

// 显示点云图
void Method::view_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointcloud)
{
    try {
        //默认保存
        std::string filename = "output.pcd";
        pcl::io::savePCDFile(filename, *pointcloud);

        // 创建一个PCL可视化对象
        pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

        // 添加点云到可视化窗口
        viewer.addPointCloud<pcl::PointXYZRGBA>(pointcloud, "PointCloud");

        // 设置点云渲染属性
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloud");

        // 开始交互式显示
        while (!viewer.wasStopped()) {
            viewer.spinOnce(100); // 100ms一次刷新
        }
    } catch (const std::exception& e) {
        std::cerr << "Error during visualization: " << e.what() << std::endl;
    }
}

// 保存点云图
void Method::save_cloud(std::string filename, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointcloud)
{
    // 尝试保存点云数据到.pcd文件
    if (pcl::io::savePCDFile(filename, *pointcloud) == -1)
    {
        PCL_ERROR("Failed to save point cloud data to disk.\n");
        return;
    }
    std::cout << "Saved point cloud to " << filename.c_str() << std::endl;
}

// 打开点云图
void Method::view_cloud_test(QString fileName)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // 读取.pcd文件
//    QString fileName = QFileDialog::getOpenFileName(nullptr, "Open PointCloud", ".", "Open PCD files(*.pcd)");

    if (fileName.isEmpty()) {
        return; // 用户取消了文件选择
    }
    std::string file_name = fileName.toStdString();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_name, *cloud) == -1) {
        PCL_ERROR("Could not read file %s\n", file_name.c_str());
        return;
    }

    // 创建一个PCL可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));

    // 添加点云到可视化窗口
    viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "cloud");

    // 设置点云渲染属性
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // 开始交互式显示
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100, true); // 100ms一次刷新
    }
}
