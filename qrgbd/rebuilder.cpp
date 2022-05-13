#include "rebuilder.h"
#include "QThread"
#include "embed.h"
#include <QDebug>
#include <colorPrj.h>
#include <opencv2/highgui.hpp>

Rebuilder::Rebuilder(QObject *parent)
    : QObject(parent)
{
}

Rebuilder::~Rebuilder()
{
    delete ui;
}

void Rebuilder::processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f Tcw)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    frameCloud->reserve(depthImg.rows * depthImg.cols * 0.9);

    cv::Mat color(depthImg.rows, depthImg.cols, CV_8UC3);
    // Tcw -> Twc
    auto Twc = Tcw.inverse();
    auto quat = Twc.unit_quaternion();
    auto trans = Twc.translation();

    for (int v = 0; v < depthImg.rows; v++)
    {
        auto colorPtr = color.ptr<uchar>(v);
        for (int u = 0; u < depthImg.cols; u++)
        {
            unsigned int d = depthImg.ptr<unsigned short>(v)[u];
            if (d == 0)
            {
                colorPtr[3 * u + 0] = 255;
                colorPtr[3 * u + 1] = 255;
                colorPtr[3 * u + 2] = 255;
                continue;
            }
            // --- for cloud point ---
            float scalar = float(d) / depthScale;
            Eigen::Vector3f p((u - cx) * scalar / fx, (v - cy) * scalar / fy, scalar);
            Eigen::Vector3f p_w = quat * p + trans;

            auto c = colorImg.at<cv::Vec3b>(v, u);

            frameCloud->push_back(pcl::PointXYZRGB(p_w(0), p_w(1), p_w(2), c[2], c[1], c[0]));

            // --- for color projection ---
            d /= 1000.0;
            auto rgb = ns_clp::ColorPrj::project(d, 1.0, 4.0, true, 0, ns_clp::Color::red_yellow);
            colorPtr[3 * u + 0] = std::get<2>(rgb);
            colorPtr[3 * u + 1] = std::get<1>(rgb);
            colorPtr[3 * u + 2] = std::get<0>(rgb);
        }
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setLeafSize(0.02, 0.02, 0.02);
    filter.setInputCloud(frameCloud);
    filter.filter(*frameCloud);

    // show image and point cloud
    emit this->signalProcessNewFrameFinished(color, frameCloud, Tcw);
}

void Rebuilder::init(ConfigDialog *cof)
{
    qDebug() << "Rebuilder thread: " << QThread::currentThread();

    cv::FileStorage config(cof->_settingPath.toStdString(), cv::FileStorage::READ);
    config["Camera1.fx"] >> this->fx;
    config["Camera1.fy"] >> this->fy;
    config["Camera1.cx"] >> this->cx;
    config["Camera1.cy"] >> this->cy;
    config["RGBD.DepthMapFactor"] >> this->depthScale;
    config.release();
}
