#ifndef REBUILD_H
#define REBUILD_H

#include "configdialog.h"
#include <QGridLayout>
#include <QLabel>
#include <QObject>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <thread>

class ReBulider : public QObject
{
    Q_OBJECT
public:
    explicit ReBulider(const std::string &cvWinName, ConfigDialog *config, QObject *parent = nullptr);

public slots:
    void processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose);

    void init();
signals:
    void signalProcessDepthFrameFinished(cv::Mat img);

private:
    QGridLayout *_layout;
    ConfigDialog *_config;

    float fx, fy, cx, cy;

public:
    const std::string _cvWinName;
};

#endif // REBUILD_H
