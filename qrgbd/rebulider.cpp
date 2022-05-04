#include "rebulider.h"
#include "QDebug"
#include "QThread"
#include "yaml-cpp/yaml.h"
#include <QDebug>
#include <QEvent>
#include <colorPrj.h>
#include <opencv2/highgui.hpp>

ReBulider::ReBulider(QLabel *label, ConfigDialog *config, QObject *parent)
    : QObject{parent}, _label(label), _config(config), _loadParamFinished(false)
{
}

bool ReBulider::eventFilter(QObject *object, QEvent *event)
{
    if (object == this->_label)
    {
        if (event->type() == QEvent::Resize)
        {
            QPixmap img = this->_label->pixmap();
            img.scaled(this->_label->size() * 0.95, Qt::KeepAspectRatio);
            this->_label->setPixmap(img);
        }
    }
    return QObject::eventFilter(object, event);
}

void ReBulider::processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose)
{
    //  qDebug() << "rebulider thread: " << QThread::currentThread();
    if (!this->_loadParamFinished)
    {
        auto config = YAML::LoadFile(this->_config->_settingPath.toStdString());
        this->fx = config["Camera1.fx"].as<float>();
        this->fy = config["Camera1.fy"].as<float>();
        this->cx = config["Camera1.cx"].as<float>();
        this->cy = config["Camera1.cy"].as<float>();
        this->_loadParamFinished = true;
    }

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
            auto rgb = ns_clp::ColorPrj::project(d, 1.0, 20.0, true, 0, ns_clp::Color::red_yellow);
            p[3 * u + 0] = std::get<2>(rgb);
            p[3 * u + 1] = std::get<1>(rgb);
            p[3 * u + 2] = std::get<0>(rgb);
        }
    }
    // show image
    QImage img = QImage((const unsigned char *)(color.data), color.cols, color.rows, QImage::Format_BGR888);
    img = img.scaled(this->_label->size() * 0.95, Qt::KeepAspectRatio);
    this->_label->setPixmap(QPixmap::fromImage(img));
}
