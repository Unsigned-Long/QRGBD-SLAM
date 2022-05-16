#include "pcl_visual.h"
#include "QThread"
#include <QDebug>

PCLVisual::PCLVisual(QObject *parent)
    : QObject{parent} {

    // pcl win

    this->allCloudPts = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloudIdx = 0;
    this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>("PointCloud");
    this->viewer->setSize(1000, 640);
    this->viewer->setBackgroundColor(0.9f, 0.9f, 0.9f);

    Eigen::Isometry3f origin(Eigen::Quaternionf::Identity());
    origin.pretranslate(Eigen::Vector3f::Zero());
    this->viewer->addCoordinateSystem(0.5, Eigen::Affine3f(origin.affine()), "origin");

    viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    viewer->spinOnce();
}

void PCLVisual::showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw) {
    if (this->_mode == PCL_VISUAL_MODE::RENDER_MODE) {
        this->renderMode(frameCloud, Tcw);
    } else if (this->_mode == PCL_VISUAL_MODE::INTERACTIVE_MODE) {
        while (!this->viewer->wasStopped() && this->_mode == PCL_VISUAL_MODE::INTERACTIVE_MODE) {
            this->viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        emit this->signalShowPtsFinished();
    }
}

void PCLVisual::renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw) {
    qDebug() << "PCL Visual thread: " << QThread::currentThread();
    if (Tcw.log() == Sophus::SE3f().log()) {
        viewer->removeAllPointClouds();
        cloudIdx = 0;
        viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        viewer->spinOnce();
        emit this->signalShowPtsFinished();
        return;
    }
    auto Twc = Tcw.inverse();
    auto quat = Twc.unit_quaternion();
    auto trans = Twc.translation();

    // point cloud
    viewer->addPointCloud(frameCloud, std::to_string(cloudIdx++));

    // camera
    Eigen::Vector3f pos(0.0f, 0.0f, -3.0f);
    Eigen::Vector3f view(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, -1.0f, -3.0f);
    pos = Twc * pos;
    view = Twc * view;
    up = Twc * up;
    up(0) = up(0) - pos(0), up(1) = up(1) - pos(1), up(2) = up(2) - pos(2);
    viewer->setCameraPosition(pos(0), pos(1), pos(2),
                              view(0), view(1), view(2),
                              up(0), up(1), up(2));

    Eigen::Isometry3f coord(quat);
    coord.pretranslate(trans);
    viewer->removeCoordinateSystem("Camera");
    viewer->addCoordinateSystem(0.2, Eigen::Affine3f(coord.affine()), "Camera");

    viewer->spinOnce();

    emit this->signalShowPtsFinished();
}
