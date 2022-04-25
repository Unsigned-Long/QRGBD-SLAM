#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#include "configdialog.h"
#include "yolo.h"
#include "yolo_v2_class.hpp"
#include <QEvent>
#include <QLabel>
#include <QObject>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class Recognizer : public QObject
{
    Q_OBJECT
public:
    explicit Recognizer(QLabel *label, ConfigDialog *config, QObject *parent = nullptr);

    bool eventFilter(QObject *object, QEvent *event) override;
public slots:
    void processNewColorFrame(cv::Mat srcImg);

signals:
    void signalProcessColorFrameFinished();

private:
    QLabel *_label;
    ConfigDialog *_config;

    std::vector<std::string> _classesVec;
    std::shared_ptr<Detector> _detector;

public:
    bool _fileLoadFinished;
};

#endif // RECOGNIZER_H
