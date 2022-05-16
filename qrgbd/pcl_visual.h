#ifndef PCL_VISUAL_H
#define PCL_VISUAL_H

#include "pcl-1.12/pcl/filters/voxel_grid.h"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/cloud_viewer.h"
#include <QObject>
#include <sophus/se3.hpp>

enum PCL_VISUAL_MODE {
    INTERACTIVE_MODE,
    RENDER_MODE
};

class PCLVisual : public QObject {
    Q_OBJECT
  public:
    explicit PCLVisual(QObject *parent = nullptr);

    void showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

  public slots:
    void renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

  signals:
    void signalShowPtsFinished();

  private:
    // pcl visualize

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCloudPts;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    std::size_t cloudIdx;

  public:
    PCL_VISUAL_MODE _mode = PCL_VISUAL_MODE::RENDER_MODE;
};

#endif // PCL_VISUAL_H
