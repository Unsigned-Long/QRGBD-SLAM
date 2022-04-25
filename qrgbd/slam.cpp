#include "slam.h"

Slam::Slam(QObject *parent)
{
}

void Slam::createSlamSystem(ConfigDialog *config)
{
    qDebug() << "slam thread: " << QThread::currentThread();

    this->_config = config;

    // create a slam system
    this->_slamSystem = std::make_shared<ORB_SLAM3::System>(
        this->_config->_vocPath.toStdString(),
        this->_config->_settingPath.toStdString(),
        ORB_SLAM3::System::RGBD,
        true);
    // finished
    emit this->createSlamSystemFinished(this->_slamSystem->GetImageScale());

    //  emit this->createSlamSystemFinished(1.0f);
}

void Slam::processNewFrame(cv::Mat colorImg, cv::Mat depthImg, double tframe)
{
    // track a new frame

    Sophus::SE3f pose = this->_slamSystem->TrackRGBD(colorImg, depthImg, tframe);

    emit this->processNewFrameFinished(pose);
}
