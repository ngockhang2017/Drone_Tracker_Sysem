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
    startTimer(50);
}

MainWindow::~MainWindow()
{
    ptzSerialPort->close();
    delete ui;
}
void MainWindow::timerEvent(QTimerEvent *event)
{
    if(this->Auto_tracking)
    sendPelcoDCommand(frameControl);
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
            qDebug() << "AUTO TRACKING STARTED!!!";
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
            if(bboxes.isEmpty() && this->Auto_tracking)
                StopSlot();

            QRect maxAccuracyBBox;
            float maxAccuracy = 0.0;

            for (const QJsonValue &value : bboxes)
            {
                if (!value.isObject()) continue;
                QJsonObject bboxObj = value.toObject();

                int x = bboxObj["x"].toInt();
                int y = bboxObj["y"].toInt();
                int width = bboxObj["width"].toInt();
                int height = bboxObj["height"].toInt();
                // Giả sử rằng JSON cũng chứa trường "accuracy", nếu không bạn cần thêm nó.
                float accuracy = bboxObj.contains("score") ? bboxObj["score"].toDouble() : 0.0;
                qDebug() << x << "," << y << "," << (x + width) << "," << (y + height) << "," << accuracy;
                QRect bbox(x, y, width, height);
                if (accuracy > maxAccuracy)
                {
                    maxAccuracy = accuracy;
                    maxAccuracyBBox = bbox;
                }
            }
            requestPTZPositions();
            if (maxAccuracy > 0 && this->Auto_tracking )
            {
                qDebug() << "maxAccuracy: " << maxAccuracy;
                trackObjectWithPTZ(maxAccuracyBBox, this->currentZoom);
            } else {
                // No object detected, adjust zoom out
                // adjustZoomForNoObject();
//                StopSlot();
            }
//        }
//        else
//            qDebug() << "AUTO TRACKING STOPED!!!";
    }
}

void MainWindow::trackObjectWithPTZ(const QRect &bbox, int zoomLevel)
{
    QPoint center = bbox.center();
    qDebug() << "===> Bouding box:("<<center.x()<<", "<<center.y()<<")";

    int frameCenterX = currentImage.width() / 2;
    int frameCenterY = currentImage.height() / 2;

    int bboxCenterX = center.x() ;//+ bbox.width() / 2;
    int bboxCenterY = center.y();// + bbox.height() / 2;

    int deltaX = bboxCenterX - frameCenterX;
    int deltaY = bboxCenterY - frameCenterY;
    //    int panOffset = (center.x() - imageCenterX) / 10;  // Adjust the divisor for sensitivity
    //    int tiltOffset = (center.y() - imageCenterY) / 10;

    // để không cần điều chỉnh nếu lệch không đáng kể
    int threshold = 5;

    float panSpeed = (std::abs(deltaX) * 0.4f) / (1+ log(zoomLevel));
    float tiltSpeed = (std::abs(deltaY) * 0.4f) / (1 + log(zoomLevel));

    bool panDirection = deltaX > 0 ? true : false; // true = pan right, false = pan left
    bool tiltDirection = deltaY > 0 ? true : false; // true = tilt down, false = tilt up

    if (std::abs(deltaX) > threshold || std::abs(deltaY) > threshold)
        sendPanTiltCommand(panDirection, panSpeed, tiltDirection, tiltSpeed);
}

void MainWindow::sendPanTiltCommand(bool panRight, float panSpeed, bool tiltDown, float tiltSpeed)
{

    unsigned char command2 = 0x00;

    if (panRight) {
        command2 |= 0x02;  // Pan right
    } else {
        command2 |= 0x04;  // Pan left
    }

    if (tiltDown) {
        command2 |= 0x10;  // Tilt down
    } else {
        command2 |= 0x08;  // Tilt up
    }

    unsigned char panSpeedByte = static_cast<unsigned char>(std::min(panSpeed, 63.0f));  // Tốc độ pan (0x00 đến 0x3F)
    unsigned char tiltSpeedByte = static_cast<unsigned char>(std::min(tiltSpeed, 63.0f)); // Tốc độ tilt (0x00 đến 0x3F)

    // Tính checksum
    unsigned char address = 0x01;
    unsigned char command1 = 0x00;

    frameControl.clear();
    frameControl.append(static_cast<char>(0xFF));
    frameControl.append(static_cast<char>(address));  // Địa chỉ camera
    frameControl.append(static_cast<char>(command1));     // Byte đồng bộ
    frameControl.append(static_cast<char>(command2));  // Lệnh
    frameControl.append(static_cast<char>(panSpeedByte));    // Data1
    frameControl.append(static_cast<char>(tiltSpeedByte));    // Data2
    quint8 checksum = (0x01 + command1 + command2 + panSpeedByte + tiltSpeedByte) % 0x100;
    frameControl.append(static_cast<char>(checksum));


}

void MainWindow::sendPelcoDCommand(QByteArray frame) {
    if (ptzSerialPort->isOpen()) {
        ptzSerialPort->write(frame);
        ptzSerialPort->flush();
    }
    else
        qDebug() << "ptzSerialPort is not open!!!";
}

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
    int cornerSize = 10;
    double xScale = static_cast<double>(scaledImage.width()) / currentImage.width();
    double yScale = static_cast<double>(scaledImage.height()) / currentImage.height();
    int centerX = imageLabel->width() / 2;
    int centerY = imageLabel->height() / 2;
    int centerRectSize = qMin(scaledImage.width(), scaledImage.height()) / 4;

    // Vẽ đường nét nổi
    QColor lightGreen(144, 238, 144); // Màu xanh nhạt
    QColor darkGreen(0, 100, 0);      // Màu xanh đậm
    QPen pen;
    drawObjectInCenter(painter);
    drawPanGauge(painter, this->currentPan);
    drawTiltGauge(painter, this->currentTilt);
    // Vẽ hộp bao quanh (bounding box)
    if (!currentBoundingBox.isNull()) {
        QRect bbox = currentBoundingBox;
        QRect scaledBbox(bbox.left() * xScale, bbox.top() * yScale, bbox.width() * xScale, bbox.height() * yScale);

        painter.setPen(QPen(Qt::red, 2));  // Vẽ bounding box màu đỏ
        painter.drawRect(scaledBbox);
    }
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

void MainWindow::onSerialDataReceived()
{
    QByteArray response = ptzSerialPort->readAll();

    // Check if the response is valid and has enough data
    if (response.size() < 7) {
        qDebug() << "Invalid response size.";
        return;
    }

    // Extract Pan, Tilt, or Zoom values based on the command
    quint8 command = response[3];
    quint16 value = (static_cast<quint8>(response[4]) << 8) | static_cast<quint8>(response[5]);

    switch (command) {
    case 0x59: // Pan Position
        currentPan = value / 100.0;  // Convert from 0.01 degrees to degrees
        qDebug() << "Current Pan:" << currentPan << "degrees";
        break;
    case 0x5B: // Tilt Position
        currentTilt = value / 100.0;  // Convert from 0.01 degrees to degrees
        qDebug() << "Current Tilt:" << currentTilt << "degrees";
        break;
    case 0x5D: // Zoom Position
        currentZoom = value;  // Zoom might not need conversion
        qDebug() << "Current Zoom Level:" << currentZoom;
        break;
    default:
        qDebug() << "Unknown command in response.";
        break;
    }
}

void MainWindow::requestPTZPositions() {
    // Query Pan Position
    QByteArray panRequest = createPelcoCommand(0x51);
    ptzSerialPort->write(panRequest);

    // Query Tilt Position
    QByteArray tiltRequest = createPelcoCommand(0x53);
    ptzSerialPort->write(tiltRequest);

    // Query Zoom Position
    QByteArray zoomRequest = createPelcoCommand(0x55);
    ptzSerialPort->write(zoomRequest);
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
    qDebug() << "Up slot";
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
