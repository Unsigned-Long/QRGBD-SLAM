#ifndef REBUILDER_H
#define REBUILDER_H

#include "configdialog.h"
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
    void processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose);

    void init(ConfigDialog *config);

private:
    Ui::Rebuilder *ui;

    float fx, fy, cx, cy;

public:
    const std::string _cvWinName = "Rebuilder";
};

#endif // REBUILDER_H
