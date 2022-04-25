#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QCloseEvent"
#include "QFileDialog"
#include "QMessageBox"
#include "csv.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("RGB-D Slam Handler");
    // create variables
    this->init();
    // build connections
    this->connection();
}

MainWindow::~MainWindow()
{
    delete this->_slam;
    delete this->_rebuilder;
    delete this->_recognizer;
    delete ui;
}

void MainWindow::connection()
{
    // for start button
    connect(ui->btn_start, &QPushButton::clicked, this, [=]() {
        ui->stackedWidget->setCurrentWidget(ui->page_use);
    });
    // for help button 2
    connect(ui->btn_help_2, &QPushButton::clicked, this, [=]() {
        ui->btn_help->click();
    });
    // for back button
    connect(ui->btn_back, &QPushButton::clicked, this, [=]() {
        ui->tabWidget->setCurrentWidget(ui->page_main);
    });
    // for save button
    connect(ui->btn_save, &QPushButton::clicked, this, [=]() {
        if (ui->tabW_mappoint->rowCount() == 0 && ui->tabW_keyframes->rowCount() == 0)
        {
            // no data
            QMessageBox::information(this, "Attention", "No data to save!");
            return;
        }
        auto path = QFileDialog::getExistingDirectory(this, "save data path", QDir::currentPath());
        if (path.isEmpty())
        {
            return;
        }

        auto slamSystem = this->_slam->getSlamSystem();
        const auto atlas = slamSystem->mpAtlas;
        std::vector<ORB_SLAM3::MapPoint *> mapPoints = atlas->GetAllMapPoints();
        std::vector<ORB_SLAM3::KeyFrame *> keyFrames = atlas->GetAllKeyFrames();

        // save map points and key frames info
        auto mapPointsFile = ns_csv::CSVWriter::create(path.toStdString() + "/mapPoints.csv");
        auto keyFramesFile = ns_csv::CSVWriter::create(path.toStdString() + "/keyFrames.csv");
        mapPointsFile->setPrecision(6);
        keyFramesFile->setPrecision(6);

        mapPointsFile->writeLine(',', "id", "X(M)", "Y(M)", "Z(M)");

        for (auto mpPtr : mapPoints)
        {
            if (mpPtr != nullptr)
            {
                // write data
                auto pos = mpPtr->GetWorldPos();
                mapPointsFile->writeLine(',', mpPtr->mnId, pos(0), pos(1), pos(2));
            }
        }
        keyFramesFile->writeLine(',', "id", "Qx", "Qy", "Qz", "Qw", "X(M)", "Y(M)", "Z(M)");

        for (auto framePtr : keyFrames)
        {
            if (framePtr != nullptr)
            {
                // write data
                auto rot = Eigen::Quaternionf(framePtr->GetRotation()).normalized();
                auto trans = framePtr->GetTranslation();

                keyFramesFile->writeLine(',', framePtr->mnId, rot.x(), rot.y(), rot.z(), rot.w(),
                                         trans(0), trans(1), trans(2));
            }
        }
        QMessageBox::information(this, "Attention", "Save finished!");
    });
    // for quit button
    connect(ui->btn_quit, &QPushButton::clicked, this, [=]() {
        this->close();
    });
    // for help button
    connect(ui->btn_help, &QPushButton::clicked, this, [=]() {
        this->_helpDig.exec();
    });
    // for configure button
    connect(ui->btn_config, &QPushButton::clicked, this, [=]() {
        this->_configDig.display();
        this->_configDig.exec();
    });
    // [2] create slam system
    connect(this, &MainWindow::signalCreateSlamSystem, this->_slam, &Slam::createSlamSystem);
    // [3] run slam system
    connect(this->_slam, &Slam::createSlamSystemFinished, this, [=](float scale) {
        // show message
        this->statusBar()->showMessage("Create slam system finished, image scale is " +
                                       QString::number(scale, 'f', 3) + ", " +
                                       QString::fromStdString(this->_timing.last_elapsed("cost")));

        // get image scale, set current frame index to zero
        this->_imgScale = scale;
        this->_curFrameIdx = -1;

        // start slaming [process a new frame]
        this->_timer.singleShot(0, this, &MainWindow::processNewFrame);
    });
    // [1] run button [read config files, create slam system]
    connect(ui->btn_run, &QPushButton::clicked, this, [=]() {
        // Check whether the configuration is complete
        if (!this->_configDig.isSetted())
        {
            QMessageBox::information(
                this,
                "Info",
                "Your settings are incomplete. Please set the relevant path first.");
            return;
        }

        // load configure info from files
        this->loadImages(this->_configDig._assoPath.toStdString());

        // get total image size
        this->_nImages = this->_vstrImageFilenamesRGB.size();
        this->_vTimesTrack.resize(_nImages);

        // if no images or quantity does not correspond, don't process
        if (this->_vstrImageFilenamesRGB.empty())
        {
            QMessageBox::information(
                this,
                "Info",
                "No images found in provided path.");
            return;
        }
        else if (this->_vstrImageFilenamesD.size() != this->_vstrImageFilenamesRGB.size())
        {
            QMessageBox::information(
                this,
                "Info",
                "Different number of images for rgb and depth.");
            return;
        }

        // start the threads
        this->_slamThread.start();
        this->_rebulidThread.start();
        this->_recogThread.start();

        // start timing
        this->_timing.reStart();

        // some settings
        this->_isRunning = true;
        this->_rebuilder->_loadParamFinished = false;
        this->_recognizer->_fileLoadFinished = false;

        // some settings for buttons
        ui->btn_run->setEnabled(false);
        ui->btn_config->setEnabled(false);

        // to create a new slam system
        emit this->signalCreateSlamSystem(&this->_configDig);

        // sho message
        this->statusBar()->showMessage("creating a new slam sysstem.");
    });
    // [5] finish the frame
    connect(this->_slam, &Slam::processNewFrameFinished, this, [=](Sophus::SE3f pose) {
        // send to the rebulider
        emit this->signalNewDepthFrameToReBulider(this->_imRGB, this->_imD, pose);

        this->statusBar()->showMessage("Process frame [" + QString::number(this->_curFrameIdx) + "] finished.");

        // organize a new frame
        double ttrack = this->_timing.last_elapsed<ns_timer::DurationType::S>();
        _vTimesTrack[this->_curFrameIdx] = ttrack;

        double T = 0;
        if (this->_curFrameIdx < _nImages - 1)
        {
            T = _vTimestamps[this->_curFrameIdx + 1] - _tframe;
        }
        else if (this->_curFrameIdx > 0)
        {
            T = _tframe - _vTimestamps[this->_curFrameIdx - 1];
        }

        qDebug() << "'frame idx':" << this->_curFrameIdx
                 << ", 'gather delay':" << QString::number(T, 'f', 5)
                 << ", 'track delay':" << QString::number(ttrack, 'f', 5)
                 << ", 'tframe':" << QString::number(_tframe, 'f', 5);

        if (ttrack < T)
            this->_timer.singleShot(T - ttrack, this, &MainWindow::processNewFrame);
        else
            this->_timer.singleShot(0, this, &MainWindow::processNewFrame);
    });
    // [4] send a new frame
    connect(this, &MainWindow::signalNewFrameToSlamSystem, this->_slam, &Slam::processNewFrame);
    // stop
    connect(ui->btn_stop, &QPushButton::clicked, this, [=]() {
        if (!this->_isRunning)
        {
            return;
        }
        this->_isRunning = false;

        this->statusBar()->showMessage("process stoped.");
    });
    // continue
    connect(ui->btn_continue, &QPushButton::clicked, this, [=]() {
        if (this->_curFrameIdx > 0 && this->_isRunning == false)
        {
            this->_isRunning = true;
            this->_timer.singleShot(0, this, &MainWindow::processNewFrame);
        }
    });
    // force quit
    connect(ui->btn_forcequit, &QPushButton::clicked, this, [=]() {
        if (this->_curFrameIdx < 0)
        {
            return;
        }
        this->_curFrameIdx = -1;
        this->_isRunning = false;

        ui->btn_run->setEnabled(true);
        ui->btn_config->setEnabled(true);

        this->quitThreads();

        this->statusBar()->showMessage("quit all threads");
    });
    // new color frame
    connect(this, &MainWindow::signalNewColorFrameToRecongnizer, this->_recognizer, &Recognizer::processNewColorFrame);
    // recognize finished
    connect(this->_recognizer, &Recognizer::signalProcessColorFrameFinished, this, [=]() {
        if (this->_isRunning)
        {
            emit this->signalNewColorFrameToRecongnizer(_imRGB);
        }
    });
    // new depth frame
    connect(this, &MainWindow::signalNewDepthFrameToReBulider, this->_rebuilder, &ReBulider::processNewDepthFrame);
}

void MainWindow::init()
{
    qDebug() << "main thread: " << QThread::currentThread();

    this->_slam = new Slam();
    this->_slam->moveToThread(&this->_slamThread);

    this->_recognizer = new Recognizer(ui->label_recongnizer, &this->_configDig);
    this->_recognizer->moveToThread(&this->_recogThread);

    this->_rebuilder = new ReBulider(ui->label_rebulider, &this->_configDig);
    this->_rebuilder->moveToThread(&this->_rebulidThread);
}

void MainWindow::closeEvent(QCloseEvent *e)
{
    // quit running thread
    this->quitThreads();
    return QMainWindow::closeEvent(e);
}

void MainWindow::loadImages(const string &strAssociationFilename)
{
    // load [imageName, association] from folder
    this->_vstrImageFilenamesD.clear();
    this->_vstrImageFilenamesRGB.clear();
    this->_vTimestamps.clear();

    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while (!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            _vTimestamps.push_back(t);
            ss >> sRGB;
            _vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            _vstrImageFilenamesD.push_back(sD);
        }
    }
}

void MainWindow::processNewFrame()
{
    if (!this->_isRunning)
    {
        return;
    }

    ++this->_curFrameIdx;

    // if all images have been processed, then return
    if (this->_curFrameIdx == this->_vstrImageFilenamesRGB.size())
    {
        this->statusBar()->showMessage("process for all frames finished.");
        qDebug() << "slam finished!";
        this->_slam->shutdown();
        ui->btn_forcequit->click();
        this->displayMapInfo();
        return;
    }

    // load color and depth images
    this->_imD = cv::imread(this->_configDig._seqPath.toStdString() + "/" +
                                _vstrImageFilenamesD[this->_curFrameIdx],
                            cv::IMREAD_UNCHANGED);
    this->_imRGB = cv::imread(this->_configDig._seqPath.toStdString() + "/" +
                                  _vstrImageFilenamesRGB[this->_curFrameIdx],
                              cv::IMREAD_UNCHANGED);

    // get frame time
    _tframe = this->_vTimestamps[this->_curFrameIdx];

    // check color image
    if (this->_imRGB.empty())
    {
        QMessageBox::information(
            this,
            "Info",
            "Failed to load image at: " + this->_configDig._seqPath +
                "/" + QString::fromStdString(this->_vstrImageFilenamesRGB[this->_curFrameIdx]));
        this->quitThreads();

        ui->btn_run->setEnabled(true);
        ui->btn_config->setEnabled(true);

        return;
    }

    // resize the image
    if (this->_imgScale != 1.f)
    {
        int width = _imRGB.cols * _imgScale;
        int height = _imRGB.rows * _imgScale;
        cv::resize(_imRGB, _imRGB, cv::Size(width, height));
        cv::resize(_imD, _imD, cv::Size(width, height));
    }

    // restart timing
    this->_timing.reStart();

    // process a new frame

    // slam
    emit this->signalNewFrameToSlamSystem(_imRGB, _imD, _tframe);

    if (this->_curFrameIdx == 0)
    {
        emit this->signalNewColorFrameToRecongnizer(_imRGB);
    }
}

void MainWindow::quitThreads()
{
    // quit the running thread
    if (this->_slamThread.isRunning())
    {
        this->_slamThread.quit();
        this->_slamThread.wait();
    }

    if (this->_recogThread.isRunning())
    {
        this->_recogThread.quit();
        this->_recogThread.wait();
    }

    if (this->_rebulidThread.isRunning())
    {
        this->_rebulidThread.quit();
        this->_rebulidThread.wait();
    }
}

void MainWindow::displayMapInfo()
{
    this->displayMapPoints();
    this->displayKeyFrames();
    ui->tabWidget->setCurrentIndex(1);
}

void MainWindow::displayMapPoints()
{
    auto slamSystem = this->_slam->getSlamSystem();
    const auto atlas = slamSystem->mpAtlas;
    std::vector<ORB_SLAM3::MapPoint *> mapPoints = atlas->GetAllMapPoints();
    int num = 0;
    for (auto mpPtr : mapPoints)
    {
        if (mpPtr != nullptr)
        {
            ++num;
        }
    }
    ui->tabW_mappoint->setColumnCount(4);
    ui->tabW_mappoint->setHorizontalHeaderLabels({"id", "X(M)", "Y(M)", "Z(M)"});
    ui->tabW_mappoint->setRowCount(num);
    QTableWidgetItem *item;
    int idx = 0;
    for (auto mpPtr : mapPoints)
    {
        if (mpPtr != nullptr)
        {
            item = new QTableWidgetItem(QString::number(mpPtr->mnId));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 0, item);

            auto pos = mpPtr->GetWorldPos();

            item = new QTableWidgetItem(QString::number(pos(0), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 1, item);

            item = new QTableWidgetItem(QString::number(pos(1), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 2, item);

            item = new QTableWidgetItem(QString::number(pos(2), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 3, item);

            ++idx;
        }
    }
}

void MainWindow::displayKeyFrames()
{
    auto slamSystem = this->_slam->getSlamSystem();
    const auto &atlas = slamSystem->mpAtlas;
    std::vector<ORB_SLAM3::KeyFrame *> keyFrames = atlas->GetAllKeyFrames();
    int num = 0;
    for (auto framePtr : keyFrames)
    {
        if (framePtr != nullptr)
        {
            ++num;
        }
    }

    int idx = 0;
    ui->tabW_keyframes->setColumnCount(8);
    ui->tabW_keyframes->setHorizontalHeaderLabels({"id", "Qx", "Qy", "Qz", "Qw", "X(M)", "Y(M)", "Z(M)"});
    ui->tabW_keyframes->setRowCount(num);
    QTableWidgetItem *item;
    for (auto framePtr : keyFrames)
    {
        if (framePtr != nullptr)
        {
            auto rot = Eigen::Quaternionf(framePtr->GetRotation()).normalized();
            auto trans = framePtr->GetTranslation();

            item = new QTableWidgetItem(QString::number(framePtr->mnId));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 0, item);

            item = new QTableWidgetItem(QString::number(rot.x(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 1, item);

            item = new QTableWidgetItem(QString::number(rot.y(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 2, item);

            item = new QTableWidgetItem(QString::number(rot.z(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 3, item);

            item = new QTableWidgetItem(QString::number(rot.w(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 4, item);

            item = new QTableWidgetItem(QString::number(trans(0), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 5, item);

            item = new QTableWidgetItem(QString::number(trans(1), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 6, item);

            item = new QTableWidgetItem(QString::number(trans(2), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 7, item);

            ++idx;
        }
    }
}
