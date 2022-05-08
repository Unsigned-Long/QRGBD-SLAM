#include "recognizer.h"
#include "QDebug"
#include "QThread"
#include "embed.h"
#include "opencv2/highgui/highgui.hpp"
#include <QImage>

Recognizer::Recognizer(const std::string &cvWinName, ConfigDialog *config, QObject *parent)
    : QObject{parent}, _cvWinName(cvWinName), _config(config)
{
}
void Recognizer::processNewColorFrame(cv::Mat srcImg)
{

    cv::Mat dstImg = srcImg.clone();

    // do process

    yoloDetector(dstImg, this->_classesVec, *(this->_detector));

    // show image

    emit this->signalProcessColorFrameFinished(dstImg);
}
void Recognizer::init()
{
    std::ifstream ifs(this->_config->_classes.toStdString().c_str());
    std::string line;
    while (std::getline(ifs, line))
        this->_classesVec.push_back(line);
    this->_detector = std::make_shared<Detector>(_config->_modelConfig.toStdString(),
                                                 _config->_modelWeights.toStdString(), 0);
}
