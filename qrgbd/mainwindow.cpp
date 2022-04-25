#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QCloseEvent"
#include "QMessageBox"

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
    delete ui;
}

void MainWindow::connection()
{
    // for start button
    connect(ui->btn_start, &QPushButton::clicked, this, [=]() {
        ui->stackedWidget->setCurrentWidget(ui->page_use);
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
        ui->btn_forcequit->click();
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
