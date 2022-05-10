#ifndef REBUILDER_H
#define REBUILDER_H

#include "configdialog.h"
#include "pcl-1.12/pcl/filters/voxel_grid.h"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/cloud_viewer.h"
#include <QGridLayout>
#include <QLabel>
#include <QObject>
#include <QWidget>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace Ui
{
    class Rebuilder;
}

class Rebuilder : public QObject
{
    Q_OBJECT

public:
    explicit Rebuilder(QObject *parent = nullptr);
    ~Rebuilder();

public slots:
    void processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f Tcw);

    void init(ConfigDialog *config);
signals:
    void signalProcessNewFrameFinished(cv::Mat img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Twc);

private:
    Ui::Rebuilder *ui;

    float fx, fy, cx, cy, depthScale;

public:
    const std::string _cvWinName = "Rebuilder";
};

#endif // REBUILDER_H
