#include "recognizer.h"
#include "QDebug"
#include "QThread"
#include "opencv2/highgui/highgui.hpp"
#include <QImage>

Recognizer::Recognizer(QLabel *label, ConfigDialog *config, QObject *parent)
    : QObject{parent}, _label(label), _config(config), _fileLoadFinished(false)
{
}

bool Recognizer::eventFilter(QObject *object, QEvent *event)
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

void Recognizer::processNewColorFrame(cv::Mat srcImg)
{
    if (!this->_fileLoadFinished)
    {
        std::ifstream ifs(this->_config->_classes.toStdString().c_str());
        std::string line;
        while (std::getline(ifs, line))
            this->_classesVec.push_back(line);
        this->_detector = std::make_shared<Detector>(_config->_modelConfig.toStdString(),
                                                     _config->_modelWeights.toStdString(), 0);
        this->_fileLoadFinished = true;
    }
    //  qDebug() << "recongnizer thread: " << QThread::currentThread();

    cv::Mat dstImg = srcImg.clone();

    // do process
    yoloDetector(dstImg, this->_classesVec, *(this->_detector));

    // show image
    QImage img = QImage((const unsigned char *)(dstImg.data), dstImg.cols, dstImg.rows, QImage::Format_BGR888);
    img = img.scaled(this->_label->size() * 0.95, Qt::KeepAspectRatio);
    this->_label->setPixmap(QPixmap::fromImage(img));

    emit this->signalProcessColorFrameFinished();
}
