#ifndef REBUILD_H
#define REBUILD_H

#include "configdialog.h"
#include <QLabel>
#include <QObject>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

class ReBulider : public QObject
{
    Q_OBJECT
public:
    explicit ReBulider(QLabel *label, ConfigDialog *config, QObject *parent = nullptr);

    bool eventFilter(QObject *object, QEvent *event) override;

public slots:
    void processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose);

signals:

private:
    QLabel *_label;
    ConfigDialog *_config;

    float fx, fy, cx, cy;

public:
    bool _loadParamFinished;
};

#endif // REBUILD_H
