#ifndef POINTCLOUDPREVIEWWIDGET_H
#define POINTCLOUDPREVIEWWIDGET_H

#include <QColor>
#include <QPoint>
#include <QVector>
#include <QVector3D>
#include <QWidget>

#include <opencv2/opencv.hpp>

class PointCloudPreviewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PointCloudPreviewWidget(QWidget *parent = nullptr);

    bool setCloud(const cv::Mat &points3d, const cv::Mat &colors, QString *errorMessage = nullptr);
    bool setCloudFromPly(const QString &fileName, QString *errorMessage = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    struct CloudPoint {
        QVector3D position;
        QColor color;
    };

    bool updateBoundsAndView(QString *errorMessage);

    QVector<CloudPoint> points;
    QVector3D center;
    float radius = 1.0f;
    float yaw = -25.0f;
    float pitch = -18.0f;
    float zoom = 1.0f;
    QPoint lastMousePosition;
};

#endif // POINTCLOUDPREVIEWWIDGET_H
