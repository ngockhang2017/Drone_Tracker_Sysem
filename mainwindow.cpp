#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QPixmap>
#include <QResizeEvent>
#include <QSerialPort>
#include <QSerialPortInfo>
#include<cmath>
#include <QtMath>
#include <iostream>
#include <limits>
#include <QRect>
#include <QPoint>
#include <QDebug>
#include <algorithm>
#include <cmath>
#include<QThread>
using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , imageSocket(new QUdpSocket(this))
    , bboxSocket(new QUdpSocket(this))
    , imageLabel(new QLabel(this))
    , ptzSerialPort(new QSerialPort(this))
{
    ui->setupUi(this);
    this->showFullScreen();

    // Bind UDP socket for image
    imageSocket->bind(QHostAddress("192.168.0.138"), 5006);
    connect(imageSocket, &QUdpSocket::readyRead, this, &MainWindow::processPendingImageDatagrams);

    // Bind UDP socket for bounding boxes
    bboxSocket->bind(QHostAddress("192.168.0.138"), 5007);
    connect(bboxSocket, &QUdpSocket::readyRead, this, &MainWindow::processPendingBboxDatagrams);

    // Set up image label
    imageLabel->setScaledContents(true);
    imageLabel->setAlignment(Qt::AlignCenter);
    setCentralWidget(imageLabel);

    // Setup PTZ Serial Port
    ptzSerialPort->setPortName("/dev/ttyUSB0");
    ptzSerialPort->setBaudRate(QSerialPort::Baud9600);
    ptzSerialPort->setDataBits(QSerialPort::Data8);
    ptzSerialPort->setParity(QSerialPort::NoParity);
    ptzSerialPort->setStopBits(QSerialPort::OneStop);
    ptzSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (ptzSerialPort->open(QIODevice::ReadWrite))
    {
        ui->statusbar->showMessage("Serial port opened successfully");
    }
    else
    {
        ui->statusbar->showMessage("Failed to open serial port");
    }

    connect(ptzSerialPort, &QSerialPort::readyRead, this, &MainWindow::onSerialDataReceived);
    //    requestPosition(this->ptzSerialPort, true);

    //    requestPosition(this->ptzSerialPort, false);
    //    qDebug() << "AT THE FIRST TIME, TILT ANGAL IS: " << this->currentTiltAngle;

    m_Manual_Control = new Manual_Control();
    connect(m_Manual_Control, SIGNAL(UpSignal()), this, SLOT(UpSlot()));
    connect(m_Manual_Control, SIGNAL(DownSignal()), this, SLOT(DownSlot()));
    connect(m_Manual_Control, SIGNAL(LeftSignal()), this, SLOT(LeftSlot()));
    connect(m_Manual_Control, SIGNAL(RightSignal()), this, SLOT(RightSlot()));
    connect(m_Manual_Control, SIGNAL(FocusInSignal()), this, SLOT(FocusInSlot()));
    connect(m_Manual_Control, SIGNAL(FocusOutSignal()), this, SLOT(FocusOutSlot()));
    connect(m_Manual_Control, SIGNAL(ZoomInSignal()), this, SLOT(ZoomInSlot()));
    connect(m_Manual_Control, SIGNAL(ZoomOutSignal()), this, SLOT(ZoomOutSlot()));
    connect(m_Manual_Control, SIGNAL(StopSignal()), this, SLOT(StopSlot()));
    startTimer(1500);


    // Tạo Timer cho hiệu ứng nhấp nháy
    blinkTimer = new QTimer(this);
    connect(blinkTimer, &QTimer::timeout, this, &MainWindow::updateBlinking);
    blinkTimer->start(500); // 500 ms cho chu kỳ nhấp nháy

}

MainWindow::~MainWindow()
{
    ptzSerialPort->close();
    delete ui;
}

void MainWindow::processPendingImageDatagrams()
{
    while (imageSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(imageSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        imageSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        QDataStream stream(&datagram, QIODevice::ReadOnly);
        stream.setByteOrder(QDataStream::LittleEndian);
        quint32 packetSize, packetIndex, totalPackets;
        stream >> packetSize >> packetIndex >> totalPackets;

        QByteArray imageData = datagram.right(datagram.size() - sizeof(quint32) * 3);

        if (packetIndex == 0) {
            receivedImageData.clear();
        }

        receivedImageData.append(imageData);

        if (packetIndex == totalPackets - 1) {
            QImage img = QImage::fromData(receivedImageData, "JPEG");
            if (!img.isNull()) {
                currentImage = img;
                imageLabel->update();
            }
        }
    }
}

void MainWindow::processPendingBboxDatagrams()
{

    while (bboxSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(bboxSocket->pendingDatagramSize());

        QHostAddress sender;
        quint16 senderPort;
        bboxSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        QJsonDocument jsonDoc = QJsonDocument::fromJson(datagram);
        if (!jsonDoc.isObject()) {
            qDebug() << "Invalid JSON format";
            continue;
        }

        QJsonObject jsonObj = jsonDoc.object();
        QJsonArray bboxes = jsonObj["bboxes"].toArray();

        //        QRect maxAccuracyBBox;
        float maxAccuracy = 0.0;

        for (const QJsonValue &value : bboxes)
        {
            if (!value.isObject()) continue;
            QJsonObject bboxObj = value.toObject();

            int x = bboxObj["x"].toInt();
            int y = bboxObj["y"].toInt();
            int width = bboxObj["width"].toInt();
            int height = bboxObj["height"].toInt();
            float accuracy = bboxObj.contains("score") ? bboxObj["score"].toDouble() : 0.0;
            QRect bbox(x, y, width, height);
            if (accuracy > maxAccuracy)
            {
                maxAccuracy = accuracy;
                this->currentBoundingBox = bbox;
            }
        }
    }
}

//old
float sigmoid(float x) {
    return 63.0f / (1.0f + std::exp(-x));
}
void MainWindow::trackObjectWithPTZ(const QRect &bbox, int zoomLevel) {

    QPoint center = bbox.center();

    int frameCenterX = currentImage.width() / 2;
    int frameCenterY = currentImage.height() / 2;

    int bboxCenterX = center.x();
    int bboxCenterY = center.y();

    qDebug() << "Frame center: (" << frameCenterX << ", " << frameCenterY << ")";
    qDebug() << "Bounding box center: (" << bboxCenterX << ", " << bboxCenterY << ")";

    int deltaX = bboxCenterX - frameCenterX;
    int deltaY = bboxCenterY - frameCenterY;

    qDebug() << "DeltaX: " << deltaX << ", DeltaY: " << deltaY;

    int threshold = 10;

    float Kp = 4;
    float Ki = 0.02;
    float Kd = 0.02;

    float adjustedKp = Kp ;/// (1 + std::log(zoomLevel / 1771.0f)); // Normalized zoom level
    float adjustedKd = Kd ;//* (1 + std::log(zoomLevel / 1771.0f));
    float adjustedKi = Ki ; // Giữ nguyên Ki

    qDebug() << "Adjusted PID values - Kp: " << adjustedKp << ", Ki: " << adjustedKi << ", Kd: " << adjustedKd;

    static int prevDeltaX = 0;
    static int prevDeltaY = 0;
    static int integralX = 0;
    static int integralY = 0;

    integralX += deltaX;
    if(integralX>1000)integralX=1000;
    if(integralX<-10000)integralX=-1000;
    integralY += deltaY;
    if(integralY>1000)integralY=1000;
    if(integralY<-1000)integralY=-1000;

    float panSpeed = adjustedKp * deltaX + adjustedKi * integralX + adjustedKd * (deltaX - prevDeltaX);
    float tiltSpeed = adjustedKp * deltaY + adjustedKi * integralY + adjustedKd * (deltaY - prevDeltaY);

    qDebug() << "PID control values - panSpeed: " << panSpeed << ", tiltSpeed: " << tiltSpeed;

    prevDeltaX = deltaX;
    prevDeltaY = deltaY;

    float distanceToCenter = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    float speedScale = 0.05;
    if(zoomLevel==0)speedScale=0.00;
    //khoảng cách tới trung tâm và hệ số điều chỉnh tốc độ
    qDebug() << "Distance to center: " << distanceToCenter << ", Speed scale: " << speedScale;

    panSpeed *= speedScale;
    tiltSpeed *= speedScale;

    //bounding box đã gần trung tâm (trong ngưỡng)
    if (std::abs(deltaX) < threshold && std::abs(deltaY) < threshold) {
        panSpeed = 0;
        tiltSpeed = 0;
        qDebug() << "Bounding box is within the threshold, stopping movement.";
    }

    // Áp dụng hàm sigmoid để chuẩn hóa giá trị tốc độ về khoảng [0, 63]
    auto sigmoid = [](float x) -> float {
        return 63.0f / (1.0f + std::exp(-x));
    };

    float absPanSpeed = (panSpeed);
    float absTiltSpeed = sigmoid(tiltSpeed);

    //tốc độ sau khi áp dụng hàm sigmoid
    qDebug() << "Sigmoid adjusted values - absPanSpeed: " << absPanSpeed << ", absTiltSpeed: " << absTiltSpeed;

    bool panRight = (deltaX > 0);  // Pan right nếu deltaX dương
    bool tiltDown = (deltaY > 0);  // Tilt down nếu deltaY dương

    qDebug() << "Pan right: " << panRight << ", Tilt down: " << tiltDown;

    if(absPanSpeed>63)absPanSpeed=63;
    char panSpeedChar = int(absPanSpeed);
    char tiltSpeedChar = static_cast<char>(std::clamp(absTiltSpeed, 0.0f, 63.0f));

    qDebug() << "Final panSpeedChar: " << int(panSpeedChar);
    qDebug() << "Final tiltSpeedChar: " << int(tiltSpeedChar);

    sendPanTiltCommand(panRight, panSpeedChar, tiltDown, tiltSpeedChar);
}

void MainWindow::sendPanTiltCommand(bool panRight, float panSpeed, bool tiltDown, float tiltSpeed)
{
    unsigned char command2 = 0x00;

    // Điều chỉnh Pan
    if (panRight) {
        command2 |= 0x02;  // Pan phải
    } else {
        command2 |= 0x04;  // Pan trái
    }

    // Điều chỉnh Tilt
    if (tiltDown) {
        command2 |= 0x10;  // Tilt xuống
    } else {
        command2 |= 0x08;  // Tilt lên
    }

    // Chuyển đổi tốc độ Pan và Tilt thành byte (0 đến 63)
    unsigned char panSpeedByte = static_cast<unsigned char>(panSpeed);  // Tốc độ pan
    unsigned char tiltSpeedByte = static_cast<unsigned char>(tiltSpeed); // Tốc độ tilt

    // Tính checksum cho khung lệnh Pelco D
    unsigned char address = 0x01;
    unsigned char command1 = 0x00;

    frameControl.clear();
    frameControl.append(static_cast<char>(0xFF));        // Byte bắt đầu
    frameControl.append(static_cast<char>(address));     // Địa chỉ camera
    frameControl.append(static_cast<char>(command1));    // Byte đồng bộ
    frameControl.append(static_cast<char>(command2));    // Lệnh Pan/Tilt
    frameControl.append(static_cast<char>(panSpeedByte)); // Tốc độ Pan
    frameControl.append(static_cast<char>(tiltSpeedByte)); // Tốc độ Tilt
    quint8 checksum = (0x01 + command1 + command2 + panSpeedByte + tiltSpeedByte) % 0x100;
    frameControl.append(static_cast<char>(checksum));    // Checksum

    // Gửi khung lệnh điều khiển qua cổng nối tiếp
    sendPelcoDCommand(frameControl);
}

void MainWindow::sendPelcoDCommand(QByteArray frame)
{
    if (ptzSerialPort->isOpen()) {
        ptzSerialPort->write(frame); // Gửi lệnh qua cổng serial
        ptzSerialPort->flush();      // Xả dữ liệu ra
    } else {
        qDebug() << "ptzSerialPort is not open!!!";
    }
}

//******************** new
void MainWindow::timerEvent(QTimerEvent *event)
{
    if(this->Auto_tracking)
    {
        if(deltaX >= deltaY && this->deltaX >0)
            ObjectTracking(this->currentBoundingBox, true);
        if(deltaY > deltaX && deltaY >0)
            ObjectTracking(this->currentBoundingBox, false);
    }
}

void MainWindow::requestPosition(QSerialPort *serialPort, bool isPan) {
    QByteArray command;
    if (isPan) {
        command.append(static_cast<uchar>(0xFF));
        command.append(static_cast<uchar>(0x01));
        command.append(static_cast<uchar>(0x00));
        command.append(static_cast<uchar>(0x51)); // Lệnh yêu cầu trạng thái
        command.append(static_cast<uchar>(0x00));
        command.append(static_cast<uchar>(0x00));
        command.append(static_cast<uchar>(0x52)); //check sum

    } else {
        command.append(static_cast<uchar>(0xFF));
        command.append(static_cast<uchar>(0x01));
        command.append(static_cast<uchar>(0x00));
        command.append(static_cast<uchar>(0x53)); // Lệnh yêu cầu trạng thái
        command.append(static_cast<uchar>(0x00));
        command.append(static_cast<uchar>(0x00));
        command.append(static_cast<uchar>(0x54));//check sum
    }

    this->ptzSerialPort->write(command);
    this->ptzSerialPort->flush();
    command.clear();
}

void MainWindow::updatePosition(QByteArray response, double &panCurrent, double &tiltCurrent) {
    if (response.mid(3, 1).toHex() == "59")  // Mã lệnh trả về Pan Position
    {
        QByteArray data = response.mid(4, 2); // Lấy PMSB và PLSB
        uint16_t PMSB = data[0] & 0xFF;
        uint16_t PLSB = data[1] & 0xFF;
        uint16_t Pdata = (PMSB * 256) + PLSB;
        panCurrent = static_cast<double>(Pdata) / 100.0;
        emit PanPosition_Updated_signal();
    }

    else if (response.mid(3, 1).toHex() == "5b")  // Mã lệnh trả về Tilt Position
    {
        QByteArray data = response.mid(4, 2); // Lấy TMSB và TLSB
        uint16_t TMSB = data[0] & 0xFF;
        uint16_t TLSB = data[1] & 0xFF;
        uint16_t Tdata = (TMSB * 256) + TLSB;

        int16_t tiltValue;
        if (Tdata >= 32768) {
            tiltValue = 32768 - Tdata;  // Điều chỉnh nếu Tdata nằm trong khoảng giá trị âm
        } else {
            tiltValue = Tdata;  // Giá trị tilt nằm trong khoảng giá trị dương
        }

        tiltCurrent = static_cast<double>(tiltValue) / 100.0;
        emit TiltPosition_Updated_signal();
    }

    // Trường hợp không rõ mã lệnh
    else
    {
        qDebug() << "Unknown response: " << response.toHex();
    }
}

void MainWindow::onSerialDataReceived()
{
    QByteArray response = ptzSerialPort->readAll();
    updatePosition(response, this->currentPanAngle, this->currentTiltAngle);
}
void MainWindow::ObjectTracking(const QRect &bbox, bool isPan)
{
    qDebug()<<"*******************************************************************";
    //    qDebug() << "Aspect RATIO: (" << currentImage.width()<<", " << currentImage.height()<<")";

    if(isPan) // adjust pan
    {
        QEventLoop loop_pan;
        connect(this, &MainWindow::PanPosition_Updated_signal, &loop_pan, &QEventLoop::quit);
        requestPosition(this->ptzSerialPort, true);
        loop_pan.exec();

        qDebug()<<"* CURRENT Pan: " <<this->currentPanAngle;

        int frameCenterX = currentImage.width() / 2;
        int bboxCenterX = bbox.x() + bbox.width() / 2;

        qDebug()<<"Center_X: "<< frameCenterX;
        qDebug()<<"bbox_X: "<< bboxCenterX;

        this->deltaX = bboxCenterX - frameCenterX;
        qDebug()<<"deltaX = " << deltaX;

        double panAdjustment = (static_cast<double>(deltaX) / currentImage.width()) * 58.7;  // Pan có phạm vi từ 2.0° đến 58.7°
        qDebug() << "panAdjustment = "<<panAdjustment;

        this->currentPanAngle += panAdjustment;

        if (currentPanAngle >= 360.0) {
            currentPanAngle = currentPanAngle - 360.0;
        } else if (currentPanAngle < 0.0) {
            currentPanAngle += 360.0;
        }

        Absolute_Pan_Position(currentPanAngle);
        this->deltaX = -1;
    }
    else // adjust tilt
    {
        QEventLoop loop_tilt;
        connect(this, &MainWindow::TiltPosition_Updated_signal, &loop_tilt, &QEventLoop::quit);
        requestPosition(this->ptzSerialPort, false);
        loop_tilt.exec();

        qDebug()<<"* CURRENT Tilt: " <<this->currentTiltAngle;

        int frameCenterY = currentImage.height() / 2;
        int bboxCenterY = bbox.y() + bbox.height() / 2;

        qDebug()<<"Center_Y: "<< frameCenterY;
        qDebug()<<"bbox_Y: "<< bboxCenterY;

        this->deltaY = bboxCenterY - frameCenterY;
        qDebug()<<"deltaY = " << deltaY ;

        double tiltAdjustment = (static_cast<double>(deltaY) / currentImage.height()) * 35.136;  // Tilt có phạm vi dự đoán là 35.136° ở mức zoom 1
        qDebug() << ", tiltAdjustment = "<<tiltAdjustment;

        this->currentTiltAngle -= tiltAdjustment;

        if (currentTiltAngle > 40.0) currentTiltAngle = 40.0;
        else if (currentTiltAngle < -90.0) currentTiltAngle = -90.0;

        Absolute_Tilt_Position(currentTiltAngle);
        this->deltaY = -1;
    }
    qDebug()<<"* NEW Pan: " <<this->currentPanAngle;
    qDebug()<<"* NEW Tilt: " <<this->currentTiltAngle;
}

void MainWindow::Absolute_Pan_Position(double pan_angle)
{
    // Đọc giá trị Pan Angle từ giao diện người dùng (giả sử góc nhập bằng độ)
    float panAngle = float(pan_angle);
    // Tính giá trị lệnh pan (angle * 100)
    int panValue = int(panAngle * 100);

    // Tách giá trị panValue thành DATA1 và DATA2
    quint8 msb = (panValue >> 8) & 0xFF;  // Lấy MSB (byte cao)
    quint8 lsb = panValue & 0xFF;         // Lấy LSB (byte thấp)

    // Cấu trúc lệnh Pelco D để pan
    QByteArray frame;
    frame.append(static_cast<uchar>(0xFF));  // Byte khởi đầu
    frame.append(static_cast<uchar>(0x01));  // Địa chỉ camera
    frame.append(static_cast<uchar>(0x00));  // Byte cố định
    frame.append(static_cast<uchar>(0x4B));  // Lệnh Pan Position
    frame.append(static_cast<uchar>(msb));   // MSB của Pan Position
    frame.append(static_cast<uchar>(lsb));   // LSB của Pan Position

    // Tính checksum (checksum là tổng của tất cả các byte ngoại trừ byte bắt đầu, sau đó lấy modulo 256)
    quint8 checksum = (0x01 + 0x00 + 0x4B + msb + lsb) % 256;
    frame.append(static_cast<uchar>(checksum));

    // Gửi lệnh tới camera qua giao tiếp serial
    this->ptzSerialPort->write(frame);
}
void MainWindow::Absolute_Tilt_Position(double tilt_angle)
{
    float tiltAngle = float(tilt_angle);

    int tiltValue = int(tiltAngle*100);
    if(tiltValue>18000)tiltValue = 36000-tiltValue;
    if(tiltValue<0)tiltValue=32768-tiltValue;

    // Tách giá trị tiltValue thành MSB và LSB
    quint8 msb = (tiltValue >> 8) & 0xFF;  // Lấy MSB (byte cao)
    quint8 lsb = tiltValue & 0xFF;         // Lấy LSB (byte thấp)

    // Cấu trúc lệnh Pelco D để tilt
    QByteArray frame;
    frame.append(static_cast<uchar>(0xFF));  // Byte khởi đầu
    frame.append(static_cast<uchar>(0x01));  // Địa chỉ camera
    frame.append(static_cast<uchar>(0x00));  // Byte cố định
    frame.append(static_cast<uchar>(0x4D));  // Lệnh Tilt Position
    frame.append(static_cast<uchar>(msb));   // MSB của Tilt Position
    frame.append(static_cast<uchar>(lsb));   // LSB của Tilt Position

    // Tính checksum (tổng tất cả các byte ngoại trừ byte khởi đầu, sau đó lấy modulo 256)
    quint8 checksum = (0x01 + 0x00 + 0x4D + msb + lsb) % 256;
    frame.append(static_cast<uchar>(checksum));

    // Gửi lệnh tới camera qua giao tiếp serial
    this->ptzSerialPort->write(frame);
}

//************
void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    imageLabel->update();
}

void MainWindow::paintEvent(QPaintEvent *event) {
    if (currentImage.isNull())
        return;

    QPainter painter(this);
    QImage scaledImage = currentImage.scaled(imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    QPixmap pixmap = QPixmap::fromImage(scaledImage);

    painter.drawPixmap(imageLabel->rect(), pixmap);

    // Tính toán các tham số
    double xScale = static_cast<double>(scaledImage.width()) / currentImage.width();
    double yScale = static_cast<double>(scaledImage.height()) / currentImage.height();

    QPen pen;
    drawObjectInCenter(painter);
    drawPanGauge(painter, this->currentPanAngle);
    drawTiltGauge(painter, this->currentTiltAngle);

    // Font và kích thước chữ
    QFont font("Arial", 20, QFont::Bold);
    painter.setFont(font);

    // Màu sắc và vị trí
    if (this->Auto_tracking)
    {
        // Chế độ Auto Tracking với nhấp nháy
        if (isBlinkingVisible)
        {
            // Vị trí hiển thị chữ
            int x = 50;
            int y = 50;

            // Vẽ đường viền đen bằng cách vẽ chữ nhiều lần xung quanh vị trí chính
            painter.setPen(QColor(0, 0, 0));  // Màu đen
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx != 0 || dy != 0) {
                        painter.drawText(x + dx, y + dy, "Auto tracking is Started");
                    }
                }
            }

            // Vẽ chữ chính (màu xanh lục)
            painter.setPen(QColor(Qt::green));  // Màu xanh lục
            painter.drawText(x, y, "Auto tracking is Started");
        }
    }
    else
    {
        // Chế độ Manual Control, không nhấp nháy
        // Vị trí hiển thị chữ
        int x = 50;
        int y = 50;
        // Vẽ đường viền đen bằng cách vẽ chữ nhiều lần xung quanh vị trí chính
        painter.setPen(QColor(0, 0, 0));  // Màu đen
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx != 0 || dy != 0) {
                    painter.drawText(x + dx, y + dy, "Manual Control");
                }
            }
        }
        // Vẽ chữ chính (màu xanh lục)
        painter.setPen(QColor(Qt::white));  // Màu xanh lục
        painter.drawText(x, y, "Manual Control");
    }

    // Vẽ thời gian hiện tại ở góc dưới bên trái
    // Font và kích thước chữ
    QFont font1("Arial", 24, QFont::Bold);
    painter.setFont(font1);



    // Vẽ thời gian hiện tại ở góc dưới bên trái
    QString currentTime = QDateTime::currentDateTime().toString("dd/MM/yyyy hh:mm:ss");

    //        int textWidth = painter.fontMetrics().horizontalAdvance(currentTime);
    int screenHeight = height();

    int x = 20;
    int y = screenHeight - 70;
    // Vẽ đường viền đen bằng cách vẽ chữ nhiều lần xung quanh vị trí chính
    painter.setPen(QColor(0, 0, 0));  // Màu đen
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx != 0 || dy != 0) {
                painter.drawText(x + dx, y + dy, currentTime);
            }
        }
    }
    // Vẽ chữ chính (màu xanh lục)
    painter.setPen(QColor(Qt::white));
    painter.drawText(x, y, currentTime);
}

void MainWindow::updateBlinking()
{
    // Đảo trạng thái nhấp nháy
    isBlinkingVisible = !isBlinkingVisible;

    // Cập nhật lại giao diện
    update();
}

void MainWindow::drawPanGauge(QPainter &painter, int panAngle) {
    int gaugeWidth = this->width() - 120;  // Chiều dài thước đo
    int gaugeHeight = 40;  // Chiều cao thước đo
    int gaugeX = 60;  // Vị trí thước đo từ cạnh trái
    int gaugeY = this->height() - gaugeHeight - 20;  // Vị trí thước đo ở cạnh dưới

    // Vẽ nền của thước đo (hình chữ nhật lớn)
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(50, 50, 50));  // Màu nền tối
    painter.drawRect(gaugeX, gaugeY, gaugeWidth, gaugeHeight);

    // Vẽ các vạch chia và số
    painter.setPen(QPen(Qt::white, 1));
    int numTicks = 36;  // Số lượng vạch chia
    int tickSpacing = gaugeWidth / (numTicks - 1);  // Khoảng cách giữa các vạch chia
    int textOffset = 25;  // Khoảng cách giữa vạch chia và số, điều chỉnh để đảm bảo số nằm trong hình chữ nhật

    for (int i = 0; i < numTicks; ++i) {
        int pos = gaugeX + i * tickSpacing;
        int lineHeight = (i % 9 == 0) ? 20 : ((i % 3 == 0) ? 15 : 10);  // Vạch lớn ở các giá trị chính (0, 90, 180, 270), vạch nhỏ ở các giá trị khác

        // Vẽ vạch chia
        painter.drawLine(pos, gaugeY + gaugeHeight - lineHeight, pos, gaugeY + gaugeHeight);

        // Vẽ số dưới các vạch chia chính
        if (i % 9 == 0) {
            QString text = QString::number(i * 10);
            QRect textRect = painter.fontMetrics().boundingRect(text);
            int textWidth = textRect.width();
            int textX = pos - textWidth / 2;  // Canh giữa số theo vạch chia
            int textY = gaugeY + gaugeHeight + textOffset;

            if (textX < gaugeX) textX = gaugeX;  // Đảm bảo số không nằm ngoài bên trái hình chữ nhật
            if (textX + textWidth > gaugeX + gaugeWidth) textX = gaugeX + gaugeWidth - textWidth;  // Đảm bảo số không nằm ngoài bên phải hình chữ nhật

            painter.drawText(textX, textY, text);
        }
    }

    // Vẽ mũi tên màu cam biểu thị giá trị Pan hiện tại
    int panPosition = (panAngle % 360) * gaugeWidth / 360;
    painter.setBrush(QColor(255, 165, 0));  // Màu cam cho mũi tên
    QPointF points[3] = {
        QPointF(gaugeX + panPosition - 10, gaugeY - 10),
        QPointF(gaugeX + panPosition + 10, gaugeY - 10),
        QPointF(gaugeX + panPosition, gaugeY)  // Đỉnh mũi tên
    };
    painter.drawPolygon(points, 3);

    // Vẽ hình chữ nhật đánh dấu vị trí hiện tại
    painter.setBrush(QColor(255, 165, 0));  // Màu cam
    painter.drawRect(gaugeX + panPosition - 2, gaugeY, 4, gaugeHeight);

    // Hiển thị số độ Pan hiện tại bên trên mũi tên
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 12, QFont::Bold));
    painter.drawText(gaugeX + panPosition - 15, gaugeY - 20, QString::number(panAngle) + "°");
}

void MainWindow::drawTiltGauge(QPainter &painter, int tiltAngle) {
    int gaugeWidth = 40;  // Chiều rộng thước đo
    int gaugeHeight = this->height() - 120;  // Chiều cao thước đo
    int gaugeX = this->width() - gaugeWidth - 60;  // Đẩy thước đo Tilt lệch sang trái một chút
    int gaugeY = 60;  // Vị trí thước đo từ cạnh trên

    // Vẽ nền của thước đo (hình chữ nhật lớn)
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(50, 50, 50));  // Màu nền tối
    painter.drawRect(gaugeX, gaugeY, gaugeWidth, gaugeHeight);

    // Vẽ các vạch lớn và vạch nhỏ bên trong hình chữ nhật nền
    painter.setPen(QPen(Qt::white, 1));
    for (int i = -90; i <= 90; i += 10) {
        int pos = gaugeY + gaugeHeight - ((i + 90) * gaugeHeight / 180);
        int lineWidth = (i % 45 == 0) ? 20 : ((i % 15 == 0) ? 15 : 10);  // Vạch lớn ở các giá trị chính (0, -45, 45), vạch nhỏ ở các giá trị khác
        painter.drawLine(gaugeX + gaugeWidth - lineWidth, pos, gaugeX + gaugeWidth, pos);
        if (i % 45 == 0) {
            painter.drawText(gaugeX + gaugeWidth + 5, pos + 5, QString::number(i));  // Hiển thị số cho vạch lớn
        }
    }

    // Vẽ mũi tên màu cam biểu thị giá trị Tilt hiện tại
    int tiltPosition = ((tiltAngle + 90) * gaugeHeight) / 180;  // Từ -90 đến 90
    painter.setBrush(QColor(255, 165, 0));  // Màu cam cho mũi tên
    QPointF points[3] = {
        QPointF(gaugeX - 10, gaugeY + gaugeHeight - tiltPosition - 10),
        QPointF(gaugeX - 10, gaugeY + gaugeHeight - tiltPosition + 10),
        QPointF(gaugeX, gaugeY + gaugeHeight - tiltPosition)  // Đỉnh mũi tên
    };
    painter.drawPolygon(points, 3);

    // Vẽ hình chữ nhật đánh dấu vị trí hiện tại
    painter.setBrush(QColor(255, 165, 0));  // Màu cam
    painter.drawRect(gaugeX, gaugeY + gaugeHeight - tiltPosition - 2, gaugeWidth, 4);

    // Hiển thị số độ Tilt hiện tại bên trái mũi tên
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 12, QFont::Bold));
    painter.drawText(gaugeX - 40, gaugeY + gaugeHeight - tiltPosition + 5, QString::number(tiltAngle) + "°");
}

void MainWindow::drawObjectInCenter(QPainter &painter) {
    // Lấy kích thước của cửa sổ
    int windowWidth = width();
    int windowHeight = height();

    // Kích thước hình chữ nhật sẽ tỷ lệ với kích thước cửa sổ
    int rectWidth = windowWidth / 4;  // Chiều rộng hình chữ nhật
    int rectHeight = windowHeight / 6; // Chiều cao hình chữ nhật

    // Tọa độ trung tâm của hình chữ nhật
    int centerX = windowWidth / 2;
    int centerY = windowHeight / 2;

    // Hình chữ nhật nằm giữa màn hình
    QRect rectangle(centerX - rectWidth / 2, centerY - rectHeight / 2, rectWidth, rectHeight);

    QPen pen;

    // Hiệu ứng nổi trên từng cạnh của hình chữ nhật
    int penWidth = 2;

    // Vẽ cạnh trên (hiệu ứng nổi)
    pen.setWidth(penWidth);
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(rectangle.topLeft(), rectangle.topRight());  // Viền trên sáng
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(QPoint(rectangle.topLeft().x(), rectangle.topLeft().y() + penWidth),
                     QPoint(rectangle.topRight().x(), rectangle.topRight().y() + penWidth));  // Viền trên tối

    // Vẽ cạnh trái (hiệu ứng nổi)
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(rectangle.topLeft(), rectangle.bottomLeft()); // Viền trái sáng
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(QPoint(rectangle.topLeft().x() + penWidth, rectangle.topLeft().y()),
                     QPoint(rectangle.bottomLeft().x() + penWidth, rectangle.bottomLeft().y())); // Viền trái tối

    // Vẽ cạnh dưới (hiệu ứng nổi)
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(rectangle.bottomLeft(), rectangle.bottomRight()); // Viền dưới tối
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(QPoint(rectangle.bottomLeft().x(), rectangle.bottomLeft().y() - penWidth),
                     QPoint(rectangle.bottomRight().x(), rectangle.bottomRight().y() - penWidth)); // Viền dưới sáng

    // Vẽ cạnh phải (hiệu ứng nổi)
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(rectangle.topRight(), rectangle.bottomRight()); // Viền phải tối
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(QPoint(rectangle.topRight().x() - penWidth, rectangle.topRight().y()),
                     QPoint(rectangle.bottomRight().x() - penWidth, rectangle.bottomRight().y())); // Viền phải sáng

    // Độ dài của các đường thẳng kéo dài ra từ hình chữ nhật
    int lineLength = windowHeight / 5;

    // Vẽ các đường thẳng xuất phát từ các cạnh của hình chữ nhật với hiệu ứng nổi
    pen.setWidth(2);

    // Đường thẳng từ cạnh trên (hiệu ứng nổi)
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(centerX, centerY - rectHeight / 2, centerX, centerY - rectHeight / 2 - lineLength);
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(centerX + penWidth, centerY - rectHeight / 2, centerX + penWidth, centerY - rectHeight / 2 - lineLength);

    // Đường thẳng từ cạnh dưới (hiệu ứng nổi)
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(centerX, centerY + rectHeight / 2, centerX, centerY + rectHeight / 2 + lineLength);
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(centerX - penWidth, centerY + rectHeight / 2, centerX - penWidth, centerY + rectHeight / 2 + lineLength);

    // Đường thẳng từ cạnh trái (hiệu ứng nổi)
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(centerX - rectWidth / 2, centerY, centerX - rectWidth / 2 - lineLength, centerY);
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(centerX - rectWidth / 2, centerY + penWidth, centerX - rectWidth / 2 - lineLength, centerY + penWidth);

    // Đường thẳng từ cạnh phải (hiệu ứng nổi)
    pen.setColor(Qt::black);  // Đường tối
    painter.setPen(pen);
    painter.drawLine(centerX + rectWidth / 2, centerY, centerX + rectWidth / 2 + lineLength, centerY);
    pen.setColor(Qt::white);  // Đường sáng
    painter.setPen(pen);
    painter.drawLine(centerX + rectWidth / 2, centerY - penWidth, centerX + rectWidth / 2 + lineLength, centerY - penWidth);
}

QByteArray MainWindow::createPelcoCommand(quint8 command) {
    QByteArray packet;
    packet.append(static_cast<char>(0xFF));  // Sync byte
    packet.append(static_cast<char>(0x01));  // Address byte (camera ID)
    packet.append(static_cast<char>(0x00));  // Command 1 byte
    packet.append(static_cast<char>(command)); // Command 2 byte (specific command like 0x51 for pan)
    packet.append(static_cast<char>(0x00));  // Data 1 byte
    packet.append(static_cast<char>(0x01));  // Data 2 byte
    packet.append(static_cast<char>(calculateChecksum(packet))); // Checksum byte
    return packet;
}

quint8 MainWindow::calculateChecksum(const QByteArray &packet) {
    quint8 checksum = 0;
    for (int i = 1; i < packet.size(); ++i) {
        checksum += packet[i];
    }
    return checksum % 0x100;
}

void MainWindow::on_actionControl_Manual_triggered()
{

    m_Manual_Control->show();
}

//control manual
void MainWindow::UpSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x08)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x20)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x08 + 0x00 + 0x20) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::DownSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x10)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x20)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x10 + 0x00 + 0x20) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::LeftSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x04)); // Command
    frame.append(static_cast<char>(0x20)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x04 + 0x20 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::RightSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x02)); // Command
    frame.append(static_cast<char>(0x20)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x02 + 0x20 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::FocusInSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x08)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x08 + 0x00 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::FocusOutSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x01)); // Sync byte
    frame.append(static_cast<char>(0x00)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x01 + 0x00 + 0x00 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::ZoomInSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x20)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x20 + 0x00 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::ZoomOutSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x40)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x40 + 0x00 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::StopSlot()
{
    QByteArray frame;
    frame.append(static_cast<char>(0xFF)); // Start byte
    frame.append(static_cast<char>(0x01)); // Camera address
    frame.append(static_cast<char>(0x00)); // Sync byte
    frame.append(static_cast<char>(0x00)); // Command
    frame.append(static_cast<char>(0x00)); // Data1
    frame.append(static_cast<char>(0x00)); // Data2

    // Calculate checksum
    quint8 checksum = (0x01 + 0x00 + 0x00 + 0x00 + 0x00) % 100;
    frame.append(static_cast<uchar>(checksum)); // Append checksum

    this->ptzSerialPort->write(frame);
}

void MainWindow::on_actionFull_Screen_triggered()
{
    this->showFullScreen();
}

void MainWindow::on_actionMinimize_Screen_triggered()
{
    this->showNormal();
}

void MainWindow::on_actionExit_triggered()
{
    QApplication::exit();
}

void MainWindow::on_actionStart_Auto_tracking_triggered()
{
    this->Auto_tracking = true;
}

void MainWindow::on_actionStop_Auto_tracking_triggered()
{
    this->Auto_tracking = false;
    StopSlot();
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        UpSlot();
        break;
    case Qt::Key_Down:
        DownSlot();
        break;
    case Qt::Key_Left:
        LeftSlot();
        break;
    case Qt::Key_Right:
        RightSlot();
        break;
    case Qt::Key_Plus:
        ZoomInSlot();
        break;
    case Qt::Key_Minus:
        ZoomOutSlot();
        break;
    case Qt::Key_Space:
        StopSlot();
        break;
    case Qt::Key_Enter:
        StopSlot();
        if(this->Auto_tracking) this->Auto_tracking = false;
        else this->Auto_tracking = true;
        break;
    case Qt::Key_E:
        QApplication::quit();
        break;
    default:
        QMainWindow::keyPressEvent(event);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        StopSlot();
        break;
    case Qt::Key_Down:
        StopSlot();
        break;
    case Qt::Key_Left:
        StopSlot();
        break;
    case Qt::Key_Right:
        StopSlot();
        break;
    case Qt::Key_Plus:
        StopSlot();
        break;

    case Qt::Key_Minus:
        StopSlot();
        break;

    default:
        QMainWindow::keyReleaseEvent(event);
    }
}
