#ifndef CLOUDVIEWERTHREAD_H
#define CLOUDVIEWERTHREAD_H

#include <QThread>
#include <QMutex>
#include <QDialog>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QFileDialog>
#include<QDebug>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

class CloudViewerThread : public QThread
{
    Q_OBJECT

public:
    explicit CloudViewerThread(const QString &fileName, QObject *parent = nullptr);
    void stopViewer();

protected:
    void run() override;

signals:
    void finished();

private:
    QString m_fileName;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
    mutable QMutex m_mutex;
    bool m_shouldStop{false};

};

#endif // CLOUDVIEWERTHREAD_H
