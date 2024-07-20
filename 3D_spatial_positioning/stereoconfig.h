#ifndef STEREOCONFIG_H
#define STEREOCONFIG_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class StereoConfig {
public:
    cv::Mat cam_matrix_left;// 左相机内参
    cv::Mat cam_matrix_right;// 右相机内参

    // 左右相机畸变系数:[k1, k2, p1, p2, k3]
    cv::Mat distortion_l;
    cv::Mat distortion_r;

    cv::Mat R;// 旋转矩阵

    cv::Mat T;// 平移矩阵

    double focal_length;// 焦距

    double baseline;// 基线距离

    StereoConfig() {
        // 设置左相机内参
        cam_matrix_left = (cv::Mat_<double>(3, 3) << 501.8107, -2.4181, 333.7716,
                                                     0, 499.4659, 246.0701,
                                                     0, 0, 1);
        // 设置右相机内参
        cam_matrix_right = (cv::Mat_<double>(3, 3) << 502.0400, 0.7958, 322.3981,
                                                      0, 500.5333, 245.6888,
                                                      0, 0, 1);

        // 设置左右相机畸变系数（k1,k2,p1,p2,k3）
        distortion_l = (cv::Mat_<double>(1, 5) << -0.0716, 0.2363, 0.0018, -0.0014, -0.3173);
        distortion_r = (cv::Mat_<double>(1, 5) << -0.0811, 0.2108, 0.0015, -0.0031, -0.2103);

        // 设置旋转矩阵
        R = (cv::Mat_<double>(3, 3) << 1.0, -0.000232, -0.0044,
                                       0.00026, 1.0, 0.0063,
                                       0.0044, -0.0073, 1.0);

        // 设置平移矩阵
        T = (cv::Mat_<double>(3, 1) << -59.8929, -0.00011, -1.8621);

        // 设置焦距
        focal_length = 500.9625;

        // 设置基线距离
        baseline = 59.8929;
    }
};

//class StereoConfig {
//public:
//    cv::Mat cam_matrix_left;// 左相机内参
//    cv::Mat cam_matrix_right;// 右相机内参

//    // 左右相机畸变系数:[k1, k2, p1, p2, k3]
//    cv::Mat distortion_l;
//    cv::Mat distortion_r;

//    cv::Mat R;// 旋转矩阵

//    cv::Mat T;// 平移矩阵

//    double focal_length;// 焦距

//    double baseline;// 基线距离

//    StereoConfig() {
//        // 设置左相机内参
//        cam_matrix_left = (cv::Mat_<double>(3, 3) << 500.6135, -2.5167, 334.7617,
//                                                     0, 498.6875, 244.9146,
//                                                     0, 0, 1);
//        // 设置右相机内参
//        cam_matrix_right = (cv::Mat_<double>(3, 3) << 501.1568, 1.0569, 321.7995,
//                                                      0, 499.9198, 245.3022,
//                                                      0, 0, 1);

//        // 设置左右相机畸变系数（k1,k2,p1,p2,k3）
//        distortion_l = (cv::Mat_<double>(1, 5) << -0.0451, 0.0678, 0.0011, -0.001, 0);
//        distortion_r = (cv::Mat_<double>(1, 5) << -0.0699, 0.1173, 0.0011, -0.0033, 0);

//        // 设置旋转矩阵
//        R = (cv::Mat_<double>(3, 3) << 1.0, -0.000322, -0.007161,
//                                       0.000378, 1.0, 0.007858,
//                                       0.007158, -0.007861, 1.0);

//        // 设置平移矩阵
//        T = (cv::Mat_<double>(3, 1) << -59.7825, -0.0100, -1.7786);

//        // 设置焦距
//        focal_length = 500.0944;

//        // 设置基线距离
//        baseline = 59.7825;
//    }
//};

#endif // STEREOCONFIG_H
