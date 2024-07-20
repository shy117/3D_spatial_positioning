#include "mainwindow.h"

void mouse_callback(int event, int x, int y, int flags, void* param);

MainWindow::MainWindow(QWidget *parent)
   : QMainWindow(parent)
   , ui(new Ui::MainWindow)
{
   ui->setupUi(this);
   SVMethod.getRectifyTransform(img_width, img_height, config, &left_map1, &left_map2, &right_map1, &right_map2, &Q);
   //ui->textEdit->append("获取畸变校正和立体校正的映射变换矩阵、重投影矩阵");
   ui->label_5->setAttribute(Qt::WA_TransparentForMouseEvents, false);
   timer = new QTimer(this);

//   CloudViewerThread *cloudViewerThread = new CloudViewerThread(this);
//   cloudViewerThread->start(); // 这将在新线程中调用view_cloud_test()
}

MainWindow::~MainWindow()
{
    delete ui;
}

// 处理下一帧
void MainWindow::nextFrame()
{
   capture >> frame;
   if (!frame.empty()) {
       cv::Mat left_img(frame, cv::Rect(0, 0, frame.cols/2, frame.rows)); // 左侧图像
       cv::Mat right_img(frame, cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows)); // 右侧图像
       imgBGR.imgL = left_img.clone();
       imgBGR.imgR = right_img.clone();

       cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
       // 设置QLabel的Pixmap
       ui->stereoCamera->setPixmap(QPixmap::fromImage(image_scaling(frame_rgb)));

       // 调用完整方法
       // complete_method();
   }
}

// 更新处理结果
void MainWindow::updateImages()
{
    // 绘制线条
    Left_nice_line = img_rectified.imgL.clone();
    cv::line(Left_nice_line, cv::Point(128, 0), cv::Point(128, 480), cv::Scalar(0, 255, 0), 1);

    // 颜色转换适配label
    cv::cvtColor(filt_Color,filt_Color_bgr,cv::COLOR_BGR2RGB);
    cv::cvtColor(Left_nice_line,img_bgr,cv::COLOR_BGR2RGB);

    // label显示
    ui->label->setPixmap(QPixmap::fromImage(image_scaling(filt_Color_bgr)));
    ui->label_5->setPixmap(QPixmap::fromImage(image_scaling(img_bgr)));
}

// 按钮-打开双目相机
void MainWindow::on_pushButton_clicked()
{
    open_camera();
}

// 工具栏-打开双目相机
void MainWindow::on_action_3_triggered()
{
    open_camera();
}

// 打开双目相机
void MainWindow::open_camera()
{
    if (capture.isOpened())
    {
        capture.release(); // 如果已经打开摄像头，先关闭它
        if (timer != nullptr)
        {
            timer->stop();
            delete timer;
            timer = nullptr;
        }
    }
    if (!capture.open(2)) { // 打开默认摄像头
        QMessageBox::critical(this, "错误", "无法打开双目相机");
        return;
    }
    //capture.open(2); // 打开默认摄像头
    capture.set(cv::CAP_PROP_FRAME_WIDTH, img_width*2);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, img_height);

    if (capture.isOpened())
    {
        rate = capture.get(cv::CAP_PROP_FPS);
        capture >> frame;
        if (!frame.empty())
        {
            // 分割双目相机图像
            cv::Mat left_img(frame, cv::Rect(0, 0, frame.cols/2, frame.rows)); // 左侧图像
            cv::Mat right_img(frame, cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows)); // 右侧图像
            imgBGR.imgL = left_img.clone();
            imgBGR.imgR = right_img.clone();

            cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB); // 转换颜色空间为RGB

            // 设置QLabel的Pixmap
            ui->stereoCamera->setPixmap(QPixmap::fromImage(image_scaling(frame_rgb)));

            // 设置定时器以匹配FPS
            // 重新创建并初始化定时器

            timer = new QTimer(this);
            timer->setInterval(1000 / rate);
            connect(timer, &QTimer::timeout, this, &MainWindow::nextFrame); // 使用C++11语法连接信号和槽
            timer->start();

            std::cout << "打开相机" << std::endl;
        }
    }
}

// 按钮-打开图片
void MainWindow::on_pushButton_2_clicked()
{
    open_Images();
}

// 打开图片
void MainWindow::on_action_4_triggered()
{
    open_Images();
}

// 打开图片
void MainWindow::open_Images()
{
    if (!capture.isOpened())
    {
        cv::Mat result;
        // 读取图片
        //QString image_left = QFileDialog::getOpenFileName(nullptr, "Open image_left", ".", "Open image files(*.png)");
        //QString image_right = QFileDialog::getOpenFileName(nullptr, "Open image_right", ".", "Open image files(*.png)");

        //读取左图片
        QFileDialog filedialog;
        filedialog.setAcceptMode(QFileDialog::AcceptOpen);
        filedialog.setViewMode(QFileDialog::List);
        filedialog.setFileMode(QFileDialog::AnyFile);
        filedialog.setWindowTitle(tr("Open image_left"));
        filedialog.setDefaultSuffix("png");
        filedialog.setOption(QFileDialog::DontUseNativeDialog);
        if(filedialog.exec()==  QDialog::Accepted )
        {
            QStringList filePaths = filedialog.selectedFiles();
            QString image_left =filePaths[0];
            qDebug() << image_left;
            imgBGR.imgL = cv::imread(image_left.toStdString());
        }

        //读取右图片
        QFileDialog filedialog2;
        filedialog2.setAcceptMode(QFileDialog::AcceptOpen);
        filedialog2.setViewMode(QFileDialog::List);
        filedialog2.setFileMode(QFileDialog::AnyFile);
        filedialog2.setWindowTitle(tr("Open image_right"));
        filedialog2.setDefaultSuffix("png");
        filedialog2.setOption(QFileDialog::DontUseNativeDialog);
        if(filedialog2.exec()==  QDialog::Accepted )
        {
            QStringList filePaths = filedialog2.selectedFiles();
            QString image_right =filePaths[0];
            qDebug() << image_right;
            imgBGR.imgR = cv::imread(image_right.toStdString());
        }

        if (imgBGR.imgL.empty()||imgBGR.imgR.empty())
        {
            std::cerr << "图像没有正确加载" << std::endl;
            ui->textEdit->append("图像没有正确加载");
            QMessageBox::critical(this,"错误","图像没有正确加载");
            return ;
        }

        // 获取图片的长和宽
        img_width = imgBGR.imgL.cols;  // 图片的宽度
        img_height = imgBGR.imgL.rows; // 图片的高度

        std::cout << "width: " << img_width << " height: " << img_height << std::endl;
        QString strNum1 = QString::number(img_width);
        QString strNum2 = QString::number(img_height);
        // 连接所有字符串
        QString fullText = "width：" + strNum1 + "   height: " + strNum2 + " ";
        // 将完整的字符串添加到 QTextEdit
        ui->textEdit->append(fullText);

        imgRGB = SVMethod.bgr2rgb(imgBGR);


        cv::hconcat(std::vector<cv::Mat>{imgRGB.imgL, imgRGB.imgR}, result);
        // 设置QLabel的Pixmap
        ui->stereoCamera->setPixmap(QPixmap::fromImage(image_scaling(result)));

        // 调用完整方法
        // complete_method();
    }
    else
    {
        std::cerr << "双目相机已打开,无法添加图片" << std::endl;
        ui->textEdit->append("双目相机已打开,无法添加图片");
        QMessageBox::critical(this,"错误","双目相机已打开,无法添加图片");
    }
}

// 根据QLabel的尺寸缩放图像
QImage MainWindow::image_scaling(cv::Mat img)
{
   QImage img_scaled;
   // 根据QLabel的尺寸缩放图像
   if(img.channels() == 1)
   {
       img_scaled = QImage(img.data, img.cols, img.rows, img.step, QImage::Format_Grayscale8)
               .scaled(labelWidth, labelHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation);
   }
   else if(img.channels() == 3 && img.cols == img_width)
   {
       img_scaled = QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)
               .scaled(labelWidth, labelHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation);
   }
   else
   {
       img_scaled = QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888)
               .scaled(labelWidth*2, labelHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation);
   }

   return img_scaled;
}

void MainWindow::clearData()
{
    // 清空数据
    frame.release();
    frame_rgb.release();
    imgBGR.imgL.release();
    imgBGR.imgR.release();
    imgRGB.imgL.release();
    imgRGB.imgR.release();
    imgGRAY.imgL.release();
    imgGRAY.imgR.release();
    image.imgL.release();
    image.imgR.release();
    points_3d.release();
    filt_Color_bgr.release();
    img_bgr.release();
    disp.release();
    filteredImg.release();
    filter_disp.release();
    filt_Color.release();
    img_rectified.imgL.release();
    img_rectified.imgR.release();
    img_rectified_gray.imgL.release();
    img_rectified_gray.imgR.release();
    Left_nice_line.release();
}

// 关闭双目相机
void MainWindow::on_pushButton_3_clicked()
{
    // 释放摄像头资源
    if (capture.isOpened()) {
       capture.release();
    }

    // 停止定时器
    if (timer != nullptr) {
       timer->stop();
       delete timer;
       timer = nullptr;
    }

    // 清除QLabel内容
    ui->stereoCamera->clear();
    ui->label->clear();
    ui->label_5->clear();

    clearData();

}

// 清除显示
void MainWindow::on_pushButton_4_clicked()
{
   // 清除QLabel内容
   ui->stereoCamera->clear();
   ui->label->clear();
   ui->label_5->clear();

   clearData();

}

// 显示测距窗口
void MainWindow::on_pushButton_5_clicked()
{
    if(!image.imgL.empty())
    {
        cv::Mat result;
        // 绘制等间距平行线，检查立体校正的效果
        //cv::hconcat(std::vector<cv::Mat>{imgBGR.imgL, imgBGR.imgR}, result);
        //cv::Mat line = SVMethod.draw_line(img_rectified.imgL, img_rectified.imgR);
        //ui->textEdit->append("绘制等间距平行线，检查立体校正的效果");

        // 计算像素点的3D坐标（左相机坐标系下）
        cv::reprojectImageTo3D(disp, points_3d, Q);

        // 创建窗口并设置鼠标回调函数
        cv::namedWindow("Left_nice_line");
        cv::imshow("Left_nice_line", Left_nice_line);

        // 鼠标点击回调
        cv::setMouseCallback("Left_nice_line", staticMouseCallback, this);

        ui->textEdit->append("点击图片测距");
        // 结果显示
        // imshow显示
        //cv::imshow("result", result);
        //cv::imshow("line", line);
        cv::waitKey(0); // 等待按键
        // cv::imwrite("concatenated_image.jpg", result); // 如果需要保存图像，取消注释这行代码

    }
    else
    {
        ui->textEdit->append("信息不足,无法测距");
        std::cerr << "信息不足,无法测距" << std::endl;
        QMessageBox::critical(this,"错误","信息不足,无法测距");
    }
}

// 动态鼠标回调
void MainWindow::nonStaticMouseCallback(int event, int x, int y, int flags, void* ) {
   // 直接访问ui
   if (event == cv::EVENT_LBUTTONDBLCLK && (flags & cv::EVENT_FLAG_LBUTTON) != 0) {
       Measure_distance(x,y);
   }
}

// 完整处理流程
void MainWindow::complete_method()
{
    // 矫正图像;
    image.imgL = imgBGR.imgL.clone();
    image.imgR = imgBGR.imgR.clone();

    // 消除畸变
    undistort(imgBGR.imgL, image.imgL, config.cam_matrix_left, config.distortion_l);
    undistort(imgBGR.imgR, image.imgR, config.cam_matrix_right, config.distortion_r);
    //ui->textEdit->append("消除畸变");

    // 预处理灰度图
    imgGRAY = SVMethod.bgr2gray(image);
    //ui->textEdit->append("预处理灰度图");
    // 立体校正
    img_rectified = SVMethod.rectifyImage(image.imgL, image.imgR, left_map1, left_map2, right_map1, right_map2);
    img_rectified_gray = SVMethod.rectifyImage(imgGRAY.imgL, imgGRAY.imgR, left_map1, left_map2, right_map1, right_map2);

    // SGBM半局部块匹配算法
    SVMethod.stereoMatchSGBM(img_rectified_gray.imgL, img_rectified_gray.imgR, &disp, &filteredImg, &filter_disp, &filt_Color);
    //ui->textEdit->append("立体匹配");

    // 更新图像显示
    updateImages();
}

// 界面点击
void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && event->modifiers() == Qt::NoModifier) {
        QPoint pos = event->pos();
        QLabel *yourLabel = ui->label_5; // 假定这个label就是用来显示图像的

        // 获取QLabel的大小
        QSize labelSize = yourLabel->size();
        int width = labelSize.width();
        int height = labelSize.height();

        // 将label上的像素坐标转换为原图像的坐标
        int x = pos.x();
        int y = pos.y(); // 注意Y轴方向通常需要翻转
        //std::cout << "\n窗口坐标 x:" << x << " y:" << y << std::endl;
        //QString message = ("\n窗口坐标 x:%1  y:%2").arg(x).arg(y);
        //ui->textEdit->append(QStringLiteral("\n窗口坐标 x:%1  y:%2").arg(x).arg(y));

        int labelx = 400; // 测距图像坐标
        int labely = 460;

        x = x - labelx;
        y = y - labely;
        // 确保点击点在图像范围内
        if(!disp.empty() && !filter_disp.empty())
        {
            if (x >= 64 && x < width && y >= 0 && y < height)
            {
                std::cout << "label图像坐标 x:" << x << " y:" << y << std::endl;
                ui->textEdit->append(QStringLiteral("label图像坐标 x:%1  y:%2").arg(x).arg(y));
                Measure_distance(x*2, y*2);
            }
            else if(x >= 0 && x < 64 && y >= 0 && y < height)
            {
                std::cout << "测距请点击绿线右侧\n" << std::endl;
                ui->textEdit->append("测距请点击绿线右侧\n");
            }
        }
   }
   QMainWindow::mousePressEvent(event); // 传递事件给父类处理
}

// 定义计算函数
// 四参数方程
double calculate_4can(double X)
{
    // 相关系数 R2：0.999412443759929
    const double A = 401.63475319142;
    const double B = 1.29413400085038;
    const double C = 0.0100899174018329;
    const double D = 6.50851967007008;
    double Distance = (A - D) / (1 + std::pow((X / C), B)) + D;
    Distance = std::round(Distance * 100.0) / 100.0;
    return Distance;
}

// 五次多项式
double calculate_5ci(double X)
{
    // 相关系数 R2：0.996133751758688
    const double a = 0.208824816199579;
    const double b = -0.00576051793329097;
    const double c = 7.14251842370227E-05;
    const double d = -4.34997599365819E-07;
    const double e = 1.26392074842923E-09;
    const double f = -1.39859686304211E-12;
    double Distance = a + b * X + c * pow(X, 2) + d * pow(X, 3) + e * pow(X, 4) + f * pow(X, 5);
    Distance = std::round(Distance * 100.0) / 100.0;
    return Distance;
}


// 测距
void MainWindow::Measure_distance(int x,int y)
{
    cv::reprojectImageTo3D(disp, points_3d, Q);
    cv::Vec3f point_3d_vec = points_3d.at<cv::Vec3f>(y, x);

    double x_3d = static_cast<double>(point_3d_vec[0]);
    double y_3d = static_cast<double>(point_3d_vec[1]);
    double z_3d = static_cast<double>(point_3d_vec[2]);
    // 访问三维点和视差图
    double disp_value = filter_disp.at<float>(y, x);

    // 计算平均视差和距离（略去边界检查以简化示例）
    double average = 0;
    for (int u = -1; u < 2; ++u) {
       for (int v = -1; v < 2; ++v) {
           average += filter_disp.at<float>(y + u, x + v);
       }
    }
    average /= 9;

    // 四参数方程
    double Distance1 = calculate_4can(average);


    // double dis = sqrt((x_3d * x_3d) + (y_3d * y_3d) + (z_3d * z_3d)) / 100.0;
    std::cout << "像素坐标 x = " << x << ", y = " << y << std::endl;
    std::cout << "点 (" << x << ", " << y << ") 的三维坐标 (" << x_3d << ", " << -y_3d << ", " << z_3d << ") cm" << std::endl;
    std::cout << "滤波视差值：" << disp_value << std::endl;
    std::cout << "平均视差值：" << average << std::endl;
    std::cout << "以下为目标点距离左摄像头的相对距离：" << std::endl;
    std::cout << "四参数方程计算距离：" << Distance1 << " cm\n" << std::endl;

    // 构建要追加到文本编辑框的字符串
    QString message = QStringLiteral("像素点坐标 (%1, %2) \n"
                                     "三维空间坐标 (%3, %4, %5) cm\n"
                                     "滤波视差值：%6\n"
                                     "平均视差值：%7\n"
                                     "以下为目标点距离左摄像头的相对距离：\n"
                                     "四参数方程计算距离：%8 cm\n")
                                    .arg(x).arg(y)
                                    .arg(x_3d).arg(-y_3d).arg(z_3d)
                                    .arg(disp_value)
                                    .arg(average)
                                    .arg(Distance1);
    ui->textEdit->append(message);
}

// 保存双目相机图片
void MainWindow::on_action_5_triggered()
{
    save_Images();
}


// 按钮-保存双目相机图片
void MainWindow::on_pushButton_7_clicked()
{
    save_Images();
}

// 保存双目相机图片
void MainWindow::save_Images()
{
    if(!frame.empty())
    {
        // 获取当前时间
        std::string time_str = getTime();

        // 构建文件名
        std::string imgL = "imgs/imageL_" + time_str + ".png";
        std::string imgR = "imgs/imageR_" + time_str + ".png";
        std::cout << imgL << "\n" << imgR << std::endl;
        cv::imwrite(imgL, imgBGR.imgL);
        cv::imwrite(imgR, imgBGR.imgR);
        std::cout << "保存成功" << std::endl;
        ui->textEdit->append("双目相机图片保存成功");
        ui->textEdit->append("左图像："+QString::fromStdString(imgL));
        ui->textEdit->append("右图像："+QString::fromStdString(imgR));
    }
    else
    {
        std::cerr << "请先打开相机,再进行保存" << std::endl;
        ui->textEdit->append("请先打开相机,再进行保存");
        QMessageBox::critical(this,"错误","请先打开相机,再进行保存");
    }
}

// 工具栏-保存深度图
void MainWindow::on_actionb_triggered()
{
    save_depth_Images();
}

// 按钮-保存深度图
void MainWindow::on_pushButton_8_clicked()
{
    save_depth_Images();
}

// 保存深度图
void MainWindow::save_depth_Images()
{
    if(!filt_Color.empty())
    {
        // 获取当前时间
        std::string time_str = getTime();

        // 构建文件名
        std::string filename = "imgs/filt_Color_" + time_str + ".png";
        std::cout << filename << std::endl;
        cv::imwrite(filename, filt_Color);
        std::cout << "保存成功" << std::endl;
        ui->textEdit->append("保存深度图："+QString::fromStdString(filename));
    }
    else
    {
        std::cerr << "没有深度图,无法保存" << std::endl;
        ui->textEdit->append("没有深度图,无法保存");
        QMessageBox::critical(this,"错误","没有深度图,无法保存");

    }
}

// 获取当前时间
std::string MainWindow::getTime()
{
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    // 转换为本地时间
    std::tm* local_time = std::localtime(&time_t_now);

    // 格式化时间字符串
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y%m%d_%H%M%S");
    std::string time_str = oss.str();
    return time_str;
}


void MainWindow::on_pushButton_9_clicked()
{
    if (imgBGR.imgL.empty()||imgBGR.imgR.empty())
    {
        std::cerr << "图像没有正确加载" << std::endl;
        ui->textEdit->append("图像没有正确加载");
        QMessageBox::critical(this,"错误","图像没有正确加载");
        return ;
    }
    // 调用完整方法
    complete_method();
}


// 按钮-保存测距图
void MainWindow::on_pushButton_6_clicked()
{
    if(!Left_nice_line.empty())
    {
        // 获取当前时间
        std::string time_str = getTime();

        // 构建文件名
        std::string filename = "imgs/Left_nice_line" + time_str + ".png";
        std::cout << filename << std::endl;
        cv::imwrite(filename, Left_nice_line);
        std::cout << "保存成功" << std::endl;
        ui->textEdit->append("保存测距图："+QString::fromStdString(filename));

    }
    else
    {
        std::cerr << "没有测距图,无法保存" << std::endl;
        ui->textEdit->append("没有测距图,无法保存");
        QMessageBox::critical(this,"错误","没有测距图,无法保存");
    }
}

// 工具栏-生成点云图
void MainWindow::on_action_6_triggered()
{
    make_cloud();
}

// 生成点云图
void MainWindow::make_cloud()
{
    if(!disp.empty() && !img_rectified.imgL.empty())
    {
        if(flagCloud > 0)
        {
            QMessageBox::critical(this,"错误","点云图已生成");
            return;
        }
        // 计算像素点的3D坐标（左相机坐标系下）
        cv::reprojectImageTo3D(disp, points_3d, Q);
        // 构建点云--Point_XYZRGBA格式
        pointcloud = SVMethod.DepthColor2Cloud(points_3d, img_rectified.imgL);
        // 显示点云
        //SVMethod.view_cloud(pointcloud);

        //默认保存
        QString fileName;
        // 尝试保存点云数据到.pcd文件
        std::string time_str = getTime();
        // 构建文件名
        std::string filename = "imgs/pcd/PointCloud_" + time_str + ".pcd";
        pcl::io::savePCDFile(filename, *pointcloud);
        fileName = QString::fromStdString(filename);

        // 打开测试点云
        if (fileName.isEmpty()) {
            return; // 用户取消了文件选择
        }
        CloudViewerThread *thread = new CloudViewerThread(fileName, this);
        connect(thread, &CloudViewerThread::finished, thread, &QThread::quit);
        connect(thread, &CloudViewerThread::finished, thread, &QObject::deleteLater);
        thread->start();
        ui->textEdit->append("生成点云图");
        flagCloud++;
        //ui->action_6->setEnabled(false);
    }
    else
    {
        std::cerr << "信息不足,无法生成点云图" << std::endl;
        ui->textEdit->append("信息不足,无法生成点云图");
        QMessageBox::critical(this,"错误","信息不足,无法生成点云图");
    }
}


// 工具栏-打开点云图
//void MainWindow::on_action_triggered()
//{
//    // 打开测试点云
//    //SVMethod.view_cloud_test(fileName);

//    QString fileName;

//    QFileDialog filedialog;
//    filedialog.setAcceptMode(QFileDialog::AcceptOpen);
//    filedialog.setViewMode(QFileDialog::List);
//    filedialog.setFileMode(QFileDialog::AnyFile);
//    filedialog.setWindowTitle(tr("Open PointCloud"));
//    filedialog.setDefaultSuffix("pcd");
//    filedialog.setOption(QFileDialog::DontUseNativeDialog);
//    if(filedialog.exec()==  QDialog::Accepted )
//    {
//        QStringList filePaths = filedialog.selectedFiles();
//        fileName =filePaths[0];
//        qDebug() << fileName;
//    }

//    if (fileName.isEmpty())
//    {
//        return; // 用户取消了文件选择
//    }
//    // 打开测试点云
//    CloudViewerThread *thread = new CloudViewerThread(fileName, this);
//    connect(thread, &CloudViewerThread::finished, thread, &QObject::deleteLater);
//    connect(thread, &CloudViewerThread::finished, thread, &QThread::quit);
//    thread->start();
//    ui->textEdit->append("打开点云图");
//    ui->action->setEnabled(false);
//}

// 工具栏-保存点云图
void MainWindow::on_action_2_triggered()
{
    if(pointcloud != NULL)
    {
        // 尝试保存点云数据到.pcd文件
        std::string time_str = getTime();
        // 构建文件名
        std::string filename = "imgs/pcd/PointCloud_" + time_str + ".pcd";
        SVMethod.save_cloud(filename, pointcloud);
        std::cout << "点云图保存成功" << std::endl;
        ui->textEdit->append("保存点云图："+QString::fromStdString(filename));

    }
    else
    {
        std::cerr << "没有点云信息,无法保存" << std::endl;
        ui->textEdit->append("没有点云信息,无法保存");
        QMessageBox::critical(this,"错误","没有点云信息,无法保存");

    }

}


void MainWindow::on_action_7_triggered()
{
    ui->textEdit->clear();
    QString message = QStringLiteral("以下是系统介绍：\n"
                                     "基于双目摄像机的三维空间系统，进行相机标定、立体匹配、点云生成，实现三维空间定位与测距。\n"
                                     "有两种模式：相机模式和图片模式\n"
                                     "相机模式下，实时显示双目相机拍摄的场景，可进行拍照保存图片\n"
                                     "图片模式下，需要自己选者左右图像\n"
                                     "进行立体匹配后会显示深度图和测距图像\n"
                                     "点击测距图像进行测距\n"
                                     "扩展部分显示点云图像\n");

    ui->textEdit->append(message);
}


void MainWindow::on_action_8_triggered()
{
    ui->textEdit->clear();
    QString message = QStringLiteral("左相机内参：[[501.8107,    -2.4181,  333.7716],\n"
                                    "	    [                0, 499.4659,  246.0701],\n"
                                    "	    [                0,                 0,                 1]]\n\n"
                                    "右相机内参：[[ 502.0400,      0.7958, 322.3981],\n"
                                    "	    [                 0, 500.5333, 245.6888],\n"
                                    "	    [                 0,                 0,                 1]]\n\n"
                                    "左相机畸变系数（k1,k2,p1,p2,k3）：\n"
                                    "[-0.0716, 0.2363, 0.0018, -0.0014, -0.3173]\n\n"
                                    "右相机畸变系数（k1,k2,p1,p2,k3）：\n"
                                    "[-0.0811, 0.2108, 0.0015, -0.0031, -0.2103]\n\n"
                                    "旋转矩阵：[[          1.0, -0.000232,  -0.0044],\n"
                                    "	[ 0.00026,              1.0,   0.0063],\n"
                                    "	[    0.0044,     -0.0073,          1.0]]\n\n"
                                    "平移矩阵：[-59.8929, -0.00011, -1.8621]\n");

    ui->textEdit->append(message);
}


void MainWindow::on_pushButton_10_clicked()
{
    ui->textEdit->clear();
}

