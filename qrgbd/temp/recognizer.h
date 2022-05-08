#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#include "configdialog.h"
#include "yolo.h"
#include "yolo_v2_class.hpp"
#include <QEvent>
#include <QGridLayout>
#include <QLabel>
#include <QObject>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <thread>

class Recognizer : public QObject
{
    Q_OBJECT
public:
    explicit Recognizer(const std::string &cvWinName, ConfigDialog *config, QObject *parent = nullptr);

    void init();

public slots:
    void processNewColorFrame(cv::Mat srcImg);

signals:
    void signalProcessColorFrameFinished(cv::Mat img);

private:
    QGridLayout *_layout;

    ConfigDialog *_config;

    std::vector<std::string> _classesVec;
    std::shared_ptr<Detector> _detector;

public:
    const std::string _cvWinName;
    std::mutex _imgMutex;
};

#endif // RECOGNIZER_H
