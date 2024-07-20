#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWidgets/QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QString>
#include <QFileDialog>
#include <QMouseEvent>
#include <QMessageBox>
#include <QThread>


#include <opencv2/opencv.hpp>
#include "stereoconfig.h"
#include "method.h"
#include "cloudviewerthread.h"

#include "ui_mainwindow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
    void nextFrame();
    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_action_2_triggered();

    //void on_action_triggered();

    void complete_method();

    void on_actionb_triggered();

    void mousePressEvent(QMouseEvent *event);
    //void mouse_callback(int event, int x, int y, int flags, void* param);

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_action_3_triggered();

    void on_action_4_triggered();

    void on_action_5_triggered();

    void on_pushButton_9_clicked();

    void on_action_6_triggered();

    void on_action_7_triggered();

    void on_action_8_triggered();

    void on_pushButton_10_clicked();

private:
    QImage image_scaling(cv::Mat);

    void updateImages();

    void nonStaticMouseCallback(int event, int x, int y, int flags, void* );

    static void staticMouseCallback(int event, int x, int y, int flags, void* param) {
        auto* self = static_cast<MainWindow*>(param);
        self->nonStaticMouseCallback(event, x, y, flags, nullptr);
    }

    void Measure_distance(int x,int y);

    std::string getTime();

    void open_Images();

    void open_camera();

    void save_Images();

    void save_depth_Images();

    void make_cloud();

    void clearData();


private:
    Ui::MainWindow *ui;
    cv::Mat frame,frame_rgb;
    cv::VideoCapture capture;
    double rate; //FPS
    QTimer *timer;

    // QLabel的当前尺寸
    int labelWidth = 320,labelHeight = 240;
    int img_width = 640,img_height = 480;
    Method SVMethod;
    imgLR imgBGR,imgRGB,imgGRAY;
    imgLR image;// 矫正后图像

    StereoConfig config;// 读取相机内参和外参
    cv::Mat points_3d;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud;
    cv::Mat filt_Color_bgr,img_bgr;
    cv::Mat disp,filteredImg,filter_disp,filt_Color;
    imgLR img_rectified,img_rectified_gray;
    cv::Mat left_map1, left_map2, right_map1, right_map2, Q;
    cv::Mat Left_nice_line;
    int flagCloud=0;

    //CloudViewerThread *thread;
};



#endif // MAINWINDOW_H
