#include "rebuilder.h"
#include "QThread"
#include "embed.h"
#include <QDebug>
#include <colorPrj.h>
#include <opencv2/highgui.hpp>

Rebuilder::Rebuilder(QObject *parent)
    : QObject(parent) {
}

Rebuilder::~Rebuilder() {
    delete ui;
}

void Rebuilder::changeToInterMode() {
    this->_pcl_visual->_mode = PCL_VISUAL_MODE::INTERACTIVE_MODE;
    emit this->signalPCLShowPointCloud(nullptr, Sophus::SE3f());
}

void Rebuilder::changeToRenderMode() {
    this->_pcl_visual->_mode = PCL_VISUAL_MODE::RENDER_MODE;
    auto tempPts = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    tempPts->resize(this->_pts->size());
    std::copy(this->_pts->cbegin(), this->_pts->cend(), tempPts->begin());
    emit this->signalPCLShowPointCloud(tempPts, this->_curTcw);
    this->_pts->clear();
    this->_pcl_visual_need_data = false;
}

void Rebuilder::processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f Tcw) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    frameCloud->reserve(depthImg.rows * depthImg.cols * 0.9);

    cv::Mat color(depthImg.rows, depthImg.cols, CV_8UC3);
    // Tcw -> Twc
    auto Twc = Tcw.inverse();
    auto quat = Twc.unit_quaternion();
    auto trans = Twc.translation();

    for (int v = 0; v < depthImg.rows; v++) {
        auto colorPtr = color.ptr<uchar>(v);
        for (int u = 0; u < depthImg.cols; u++) {
            unsigned int d = depthImg.ptr<unsigned short>(v)[u];
            if (d == 0) {
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
    emit this->signalProcessNewFrameFinished(color);

    this->_pts->resize(this->_pts->size() + frameCloud->size());
    std::copy(frameCloud->cbegin(), frameCloud->cend(), this->_pts->end() - frameCloud->size());
    this->_curTcw = Tcw;

    if (this->_pcl_visual_need_data) {
        auto tempPts = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        tempPts->resize(this->_pts->size());
        std::copy(this->_pts->cbegin(), this->_pts->cend(), tempPts->begin());
        emit this->signalPCLShowPointCloud(tempPts, this->_curTcw);
        this->_pts->clear();
        this->_pcl_visual_need_data = false;
    }
}

void Rebuilder::init(ConfigDialog *cof) {
    qDebug() << "Rebuilder thread: " << QThread::currentThread();

    cv::FileStorage config(cof->_settingPath.toStdString(), cv::FileStorage::READ);
    config["Camera1.fx"] >> this->fx;
    config["Camera1.fy"] >> this->fy;
    config["Camera1.cx"] >> this->cx;
    config["Camera1.cy"] >> this->cy;
    config["RGBD.DepthMapFactor"] >> this->depthScale;
    config.release();
    this->_pts = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    // pcl visual
    this->_pcl_visual = std::make_shared<PCLVisual>();
    connect(this, &Rebuilder::signalPCLShowPointCloud, this->_pcl_visual.get(), &PCLVisual::showPointCloud);
    connect(this->_pcl_visual.get(), &PCLVisual::signalShowPtsFinished, this, [=]() {
        this->_pcl_visual_need_data = true;
    });

    this->_pcl_visual->moveToThread(&this->_thread);
    this->_thread.start();
}
