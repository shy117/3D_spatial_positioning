#ifndef METHOD_H
#define METHOD_H

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <iostream>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <algorithm>

#include <vector>
#include "stereoconfig.h"
#include "cloudviewerthread.h"

#include <QString>
#include <QFileDialog>
#include <QThread>


using namespace pcl;
using namespace pcl::visualization;
using namespace std;
using namespace cv;

struct imgLR{
    cv::Mat imgL;
    cv::Mat imgR;
};

struct CallbackData {
    cv::Mat points_3d;
    cv::Mat filter_disp;
};

class Method
{
public:
    Method();

    imgLR bgr2rgb(imgLR);
    imgLR bgr2gray(imgLR);
    void getRectifyTransform(int width, int height, const StereoConfig& config, cv::Mat *left_map1, cv::Mat *left_map2, cv::Mat *right_map1, cv::Mat *right_map2, cv::Mat *Q);
    cv::Mat draw_line(cv::Mat image1, cv::Mat image2);
    imgLR rectifyImage(cv::Mat image1, cv::Mat image2, cv::Mat left_map1, cv::Mat left_map2, cv::Mat right_map1, cv::Mat right_map2);
    void stereoMatchSGBM(const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat *disp, cv::Mat *filteredImg, cv::Mat * filter_disp, cv::Mat *filt_Color);
    cv::Mat hw3ToN3(const cv::Mat& points);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr DepthColor2Cloud(const cv::Mat& points_3d, const cv::Mat& colors);
    void view_cloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointcloud);
    void save_cloud(std::string filename, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointcloud);
    void view_cloud_test(QString fileName);

    //CloudViewerThread(QObject *parent = nullptr) : QThread(parent) {}
};

#endif // METHOD_H
