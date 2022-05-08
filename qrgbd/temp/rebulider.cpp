#include "rebulider.h"
#include "QDebug"
#include "QThread"
#include "embed.h"
#include "yaml-cpp/yaml.h"
#include <QDebug>
#include <QEvent>
#include <colorPrj.h>
#include <opencv2/highgui.hpp>

ReBulider::ReBulider(const std::string &cvWinName, ConfigDialog *config, QObject *parent)
    : QObject{parent}, _cvWinName(cvWinName), _config(config)
{
}

void ReBulider::processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose)
{

    double depthScale = 1000.0;
    cv::Mat color(depthImg.rows, depthImg.cols, CV_8UC3);
    for (int v = 0; v < depthImg.rows; v++)
    {
        auto p = color.ptr<uchar>(v);
        for (int u = 0; u < depthImg.cols; u++)
        {
            unsigned int d = depthImg.ptr<unsigned short>(v)[u];
            if (d == 0)
            {
                p[3 * u + 0] = 255;
                p[3 * u + 1] = 255;
                p[3 * u + 2] = 255;
                continue;
            }
            d /= 1000.0;
            auto rgb = ns_clp::ColorPrj::project(d, 1.0, 4.0, true, 0, ns_clp::Color::red_yellow);
            p[3 * u + 0] = std::get<2>(rgb);
            p[3 * u + 1] = std::get<1>(rgb);
            p[3 * u + 2] = std::get<0>(rgb);
        }
    }

    // show image
    emit this->signalProcessDepthFrameFinished(color);
}

void ReBulider::init()
{
    cv::FileStorage config(this->_config->_settingPath.toStdString(), cv::FileStorage::READ);
    config["Camera1.fx"] >> this->fx;
    config["Camera1.fy"] >> this->fy;
    config["Camera1.cx"] >> this->cx;
    config["Camera1.cy"] >> this->cy;
    config.release();
}
