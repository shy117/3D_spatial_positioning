#include "cloudviewerthread.h"

CloudViewerThread::CloudViewerThread(const QString &fileName, QObject *parent) :
    QThread(parent),
    m_fileName(fileName)
{
}


void CloudViewerThread::stopViewer()
{
    m_shouldStop = true;
    if (m_viewer)
    {
        m_viewer->close();
        //qDebug()<<"1111";
    }
    emit finished();
}

//void CloudViewerThread::run()
//{
//    if (m_viewer && !m_shouldStop)
//    {
//        m_viewer->removeAllPointClouds();
//        m_viewer->close();
//    }
//    m_viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer"));

//    QMutexLocker locker(&m_mutex);
//    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

//    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(m_fileName.toStdString(), *m_cloud) == -1)
//    {
//        PCL_ERROR("Could not read file %s\n", m_fileName.toStdString().c_str());
//        emit finished();
//        return;
//    }

//    m_viewer->addPointCloud<pcl::PointXYZRGBA>(m_cloud, "cloud");
//    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

//    while (!m_shouldStop && !m_viewer->wasStopped())
//    {
//        m_viewer->spinOnce(100, true); // 参数true表示不阻塞主线程
//        if (!m_viewer) // 新增：检查m_viewer是否已经销毁
//            break;
//    }
//    stopViewer();
//}

void CloudViewerThread::run()
{
    m_viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer"));

    QMutexLocker locker(&m_mutex);
    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(m_fileName.toStdString(), *m_cloud) == -1)
    {
        PCL_ERROR("Could not read file %s\n", m_fileName.toStdString().c_str());
        emit finished();
        return;
    }

    m_viewer->addPointCloud<pcl::PointXYZRGBA>(m_cloud, "cloud");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    while (!m_shouldStop && !m_viewer->wasStopped())
    {
        m_viewer->spinOnce(100, true);
    }

    // 清理工作放在循环结束后
    m_viewer->removeAllPointClouds(); // 先移除所有点云
    m_viewer->close();               // 关闭窗口
    m_viewer.reset();                // 重置智能指针，确保资源被正确释放

    emit finished();
}
