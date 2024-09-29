#include "mainwindow.h"
#include "ui_mainwindow.h"

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

    //    this->showFullScreen();

    // Bind UDP socket for image
    imageSocket->bind(QHostAddress("127.0.0.1"), 5006);
    connect(imageSocket, &QUdpSocket::readyRead, this, &MainWindow::processPendingImageDatagrams);

    // Bind UDP socket for bounding boxes
    bboxSocket->bind(QHostAddress("127.0.0.1"), 5007);
    connect(bboxSocket, &QUdpSocket::readyRead, this, &MainWindow::processPendingBboxDatagrams);

    // Set up image label
    imageLabel->setScaledContents(true);
    imageLabel->setAlignment(Qt::AlignCenter);
    setCentralWidget(imageLabel);

    // Setup PTZ Serial Port
    ptzSerialPort->setPortName("COM4");
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
        ui->statusbar->showMessage("Failed to open serial port!");
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

    startTimer(1000);

    //Tạo timer cho cập nhật doa
    //    update_pantilt = new QTimer(this);
    //    connect(update_pantilt, &QTimer::timeout, this, &MainWindow::UpdatePanTiltStatus);
    //    update_pantilt->start(3000);

    // Tạo Timer cho hiệu ứng nhấp nháy
    blinkTimer = new QTimer(this);
    connect(blinkTimer, &QTimer::timeout, this, &MainWindow::updateBlinking);
    blinkTimer->start(500); // 500 ms cho chu kỳ nhấp nháy

    //Timer kiểm tra góc tín hiệu tới (check PAN)
    update_doa = new QTimer(this);
    connect(update_doa, &QTimer::timeout, this, &MainWindow::CheckDOA);
    //    update_doa->start(3000);

    // Timer kiểm tra góc tín hiệu tới (check TILT)
    tilt_check_by_step_timer = new QTimer(this);
    connect(tilt_check_by_step_timer, &QTimer::timeout, this, &MainWindow::CheckTiltStep);
    //    tilt_check_by_step_timer->start(1000);

    this->manager = new QNetworkAccessManager();
    QObject::connect(manager, SIGNAL(finished(QNetworkReply*)),this, SLOT(networkReplyKraken(QNetworkReply*)));

    //DOA Timer
    this->timer_request = new QTimer(this);
    connect(timer_request, SIGNAL(timeout()), this, SLOT(Request()));
    //    timer_request->start(500);
}

void MainWindow::Request()  //timer for request to server
{
    QNetworkRequest request(QUrl("http://192.168.100.10:8081/DOA_value.html"));
    manager->get(request);
}
void MainWindow::networkReplyKraken(QNetworkReply *reply)
{
    QByteArray data = reply->readAll();
    if(!data.isEmpty())
    {
        QString GetDataString = QString(data);
        DOASample datarow1;
        QStringList cols = GetDataString.split(",");

        double MaxValue =  0.1;
        double value = 0.0;

        current_doa_check = 360.0 - cols.at(1).toDouble();

        qDebug() << "current_doa_check = " << current_doa_check;
    }
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

        //        qDebug() << "bboxes size: " << bboxes.size();
        if(bboxes.size() == 0)
        {
            //không detect được hoặc chưa detect được
            //            Pan_Expectaion = currentPanAngle;
            //            Tilt_Expectation = currentTiltAngle;
            this->Object_Detected = false;
        }
        else
        {
            this->Object_Detected = true;
            this->Auto_tracking = true;
            this->stop_check_pan = true;

            // QRect maxAccuracyBBox;
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

                    //                    if(std::abs(currentPanAngle - Pan_Expectaion) <= 0.1)
                    //                    {
                    //                        this->deltaX = (currentBoundingBox.x() + currentBoundingBox.width()/2) - currentImage.width()/2;
                    //                        qDebug() << "Cap nhat delta X!";
                    //                    }
                    //                    if(std::abs(currentTiltAngle - Tilt_Expectation) <= 0.1)
                    //                    {
                    //                        this->deltaY = (currentBoundingBox.y() + currentBoundingBox.height()/2) - currentImage.height()/2;
                    //                        qDebug() << "Cap nhat delta Y!";
                    //                    }
                }
            }
        }
    }
}
void MainWindow::CheckDOA()
{
    //    if(this->Object_Detected == false && stop_check_pan == false)
    //    {
    //        Absolute_Pan_Position(double(this->current_doa_check));
    //    }
}
void MainWindow::CheckTiltStep()
{
    //    if(this->Object_Detected == false)
    //    {
    //        Absolute_Tilt_Position(this->tilt_check_by_step);
    //        tilt_check_by_step += -20.0;
    //        if(tilt_check_by_step == -40.0)
    //            tilt_check_by_step = 40.0;
    //    }
}
void MainWindow::UpdatePanTiltStatus()
{
    QEventLoop loop_pan;
    connect(this, &MainWindow::PanPosition_Updated_signal, &loop_pan, &QEventLoop::quit);
    requestPosition(this->ptzSerialPort, true);
    loop_pan.exec();

    QEventLoop loop_tilt;
    connect(this, &MainWindow::TiltPosition_Updated_signal, &loop_tilt, &QEventLoop::quit);
    requestPosition(this->ptzSerialPort, false);
    loop_tilt.exec();
}
//******************** new control function
void MainWindow::timerEvent(QTimerEvent *event)
{
    if(this->Auto_tracking && Object_Detected)
    {
        QEventLoop loop_pan;
        connect(this, &MainWindow::PanPosition_Updated_signal, &loop_pan, &QEventLoop::quit);
        requestPosition(this->ptzSerialPort, true);
        loop_pan.exec();

        this->deltaX = (currentBoundingBox.x() + currentBoundingBox.width()/2) - currentImage.width()/2;


        QEventLoop loop_tilt;
        connect(this, &MainWindow::TiltPosition_Updated_signal, &loop_tilt, &QEventLoop::quit);
        requestPosition(this->ptzSerialPort, false);
        loop_tilt.exec();

        this->deltaY = (currentBoundingBox.y() + currentBoundingBox.height()/2) - currentImage.height()/2;


        qDebug() << "deltaX = " << (deltaX) << "; deltaY = " << (deltaY);
        if(std::abs(deltaX) > std::abs(deltaY))
        {
            ObjectTracking(this->currentBoundingBox, true);
        }
        else if(std::abs(deltaX) < std::abs(deltaY))
        {

            ObjectTracking(this->currentBoundingBox, false);
        }
    }
    qDebug() << "TIMER ĐƯỢC GỌI ĐẾN!!!";
}
void MainWindow::ObjectTracking(const QRect &bbox, bool isPan)
{
    //    qDebug()<<"******************************************************************* Time count = " << this->count;
    if(isPan) // adjust pan
    {
        double panAdjustment = (static_cast<double>(deltaX) / currentImage.width()) * 58.7;  // Pan có phạm vi từ 2.0° đến 58.7°

        double new_pan_angle = this->currentPanAngle + panAdjustment;

        if (new_pan_angle >= 360.0) new_pan_angle = new_pan_angle - 360.0;
        else if (new_pan_angle < 0.0) new_pan_angle += 360.0;

        this->Pan_Expectaion = new_pan_angle;

        Absolute_Pan_Position(new_pan_angle);
        this->deltaX = 0;
    }
    else // adjust tilt
    {
        double tiltAdjustment = (static_cast<double>(deltaY) / currentImage.height()) * 35.136;  // Tilt có phạm vi dự đoán là 35.136° ở mức zoom 1

        double new_tilt_angle = this->currentTiltAngle - tiltAdjustment;

        if (new_tilt_angle > 40.0) new_tilt_angle = 40.0;
        else if (new_tilt_angle < -90.0) new_tilt_angle = -90.0;

        Tilt_Expectation = new_tilt_angle;

        Absolute_Tilt_Position(new_tilt_angle);
        this->deltaY = 0;
    }
}
void MainWindow::requestPosition(QSerialPort *serialPort, bool isPan) {
    QByteArray command;
    if (isPan) {
        command.append(static_cast<char>(0xFF));
        command.append(static_cast<char>(0x01));
        command.append(static_cast<char>(0x00));
        command.append(static_cast<char>(0x51)); // Lệnh yêu cầu trạng thái
        command.append(static_cast<char>(0x00));
        command.append(static_cast<char>(0x00));
        command.append(static_cast<char>(0x52)); //check sum

    } else {
        command.append(static_cast<char>(0xFF));
        command.append(static_cast<char>(0x01));
        command.append(static_cast<char>(0x00));
        command.append(static_cast<char>(0x53)); // Lệnh yêu cầu trạng thái
        command.append(static_cast<char>(0x00));
        command.append(static_cast<char>(0x00));
        command.append(static_cast<char>(0x54));//check sum
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
    if (ptzSerialPort->bytesAvailable() >= 7)
    {
        QByteArray response = ptzSerialPort->read(7);
        //    QByteArray response = ptzSerialPort->readAll();
        qDebug() << "Response size is: " << response.size();

        if (response.size() < 7) {
            qDebug() << "Response too short, ignoring: " << response.toHex();
            return;
        }

        if (response.size() == 7)
            updatePosition(response, this->currentPanAngle, this->currentTiltAngle);
         else
            qDebug() << "Received an empty response at -onSerialDataReceived-!";

    }
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
    frame.append(static_cast<char>(0xFF));  // Byte khởi đầu
    frame.append(static_cast<char>(0x01));  // Địa chỉ camera
    frame.append(static_cast<char>(0x00));  // Byte cố định
    frame.append(static_cast<char>(0x4B));  // Lệnh Pan Position
    frame.append(static_cast<char>(msb));   // MSB của Pan Position
    frame.append(static_cast<char>(lsb));   // LSB của Pan Position

    // Tính checksum (checksum là tổng của tất cả các byte ngoại trừ byte bắt đầu, sau đó lấy modulo 256)
    quint8 checksum = (0x01 + 0x00 + 0x4B + msb + lsb) % 256;
    frame.append(static_cast<char>(checksum));

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
    frame.append(static_cast<char>(0xFF));  // Byte khởi đầu
    frame.append(static_cast<char>(0x01));  // Địa chỉ camera
    frame.append(static_cast<char>(0x00));  // Byte cố định
    frame.append(static_cast<char>(0x4D));  // Lệnh Tilt Position
    frame.append(static_cast<char>(msb));   // MSB của Tilt Position
    frame.append(static_cast<char>(lsb));   // LSB của Tilt Position

    // Tính checksum (tổng tất cả các byte ngoại trừ byte khởi đầu, sau đó lấy modulo 256)
    quint8 checksum = (0x01 + 0x00 + 0x4D + msb + lsb) % 256;
    frame.append(static_cast<char>(checksum));

    // Gửi lệnh tới camera qua giao tiếp serial
    this->ptzSerialPort->write(frame);
}
//===============================>
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
    //    drawPanGauge(painter, this->currentPanAngle);
    drawTiltGauge(painter, this->currentTiltAngle);
    Paint_Pan_Status(painter, this->currentPanAngle);

    // Font và kích thước chữ
    QFont font("Arial", 20, QFont::Bold);
    painter.setFont(font);

    if (this->Auto_tracking)
    {
        // Chế độ Auto Tracking với nhấp nháy
        if (isBlinkingVisible)
        {
            // Vị trí hiển thị chữ
            int x = 50;
            int y = 100;

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
        int y = 100;
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

    int x = this->width() - 500;
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
void MainWindow::drawTiltGauge(QPainter &painter, double tiltAngle) {
    int gaugeWidth = 40;  // Chiều rộng thước đo
    int gaugeHeight = this->height() - 120;  // Chiều cao thước đo
    int gaugeX = this->width() - gaugeWidth-60;  // Đẩy thước đo Tilt lệch sang trái một chút
    int gaugeY = 60;  // Vị trí thước đo từ cạnh trên

    // Vẽ viền của thước đo (chỉ viền, không tô màu bên trong)
    painter.setPen(QPen(Qt::black, 3));
    painter.setBrush(Qt::NoBrush);  // Không tô màu
    painter.drawRect(gaugeX, gaugeY, gaugeWidth, gaugeHeight);

    painter.setPen(QPen(Qt::white, 1));
    painter.setBrush(Qt::NoBrush);  // Không tô màu
    painter.drawRect(gaugeX, gaugeY, gaugeWidth, gaugeHeight);

    // Giới hạn góc tilt
    int tiltMin = -90;  // Giới hạn tilt xuống
    int tiltMax = 40;   // Giới hạn tilt lên
    int tiltRange = tiltMax - tiltMin;

    // Vẽ các vạch chia độ nổi màu trắng, 0 độ ở giữa, vạch dài hơn mỗi 10 độ, vạch ngắn mỗi 5 độ
    painter.setPen(QPen(Qt::black, 3));  // Nét đen lớn
    painter.setFont(QFont("Arial", 10, QFont::Bold));

    for (int i = tiltMin; i <= tiltMax; i += 5) {
        // Tính vị trí của vạch
        int pos = gaugeY + gaugeHeight - ((i - tiltMin) * gaugeHeight / tiltRange);
        int lineLength = (i % 10 == 0) ? 20 : 10;  // Vạch dài mỗi 10 độ, ngắn mỗi 5 độ

        // Vẽ vạch nổi (nét đen lớn trước, sau đó nét trắng nhỏ hơn)
        painter.setPen(QPen(Qt::black, 3));  // Nét đen lớn
        //        painter.drawLine(gaugeX + gaugeWidth - lineLength, pos, gaugeX + gaugeWidth, pos);
        painter.drawLine(gaugeX , pos, gaugeX + lineLength, pos);

        painter.setPen(QPen(Qt::white, 1));  // Nét trắng nhỏ hơn
        painter.drawLine(gaugeX , pos, gaugeX + lineLength, pos);

        // Hiển thị số độ ở mỗi vạch chia lớn (mỗi 10 độ)
        if (i % 10 == 0) {
            QString text = QString::number(i);
            painter.setPen(QPen(Qt::black, 10));  // Nét đen lớn cho chữ
            painter.drawText(gaugeX + gaugeWidth + 10, pos + 5, text);  // Vẽ chữ đen
            painter.setPen(QPen(Qt::white, 5));  // Nét trắng nhỏ cho chữ
            painter.drawText(gaugeX + gaugeWidth + 10, pos + 5, text);  // Vẽ chữ trắng nổi
        }
    }

    // Tính vị trí của mũi tên hiện tại dựa trên góc tilt
    int tiltPos = gaugeY + gaugeHeight - ((tiltAngle - tiltMin) * gaugeHeight / tiltRange);

    // Vẽ mũi tên nổi màu cam
    QPointF arrowTip(gaugeX - 10, tiltPos);  // Đỉnh mũi tên
    QPointF arrowLeft(gaugeX - 20, tiltPos - 10);
    QPointF arrowRight(gaugeX - 20, tiltPos + 10);

    // Vẽ mũi tên nổi (đen trước, cam sau)
    painter.setPen(QPen(Qt::black, 3));  // Nét đen lớn cho mũi tên
    painter.setBrush(QBrush(Qt::NoBrush));  // Không tô màu bên trong
    painter.drawPolygon(QPolygonF() << arrowTip << arrowLeft << arrowRight);
    painter.setPen(QPen(QColor(255, 109, 1), 2));  // Màu cam cho mũi tên
    painter.drawPolygon(QPolygonF() << arrowTip << arrowLeft << arrowRight);

    // Hiển thị góc tilt hiện tại bên trái mũi tên
    QString tiltText = QString::number(tiltAngle) + "°";
    painter.setPen(QPen(Qt::black, 6));  // Nét đen lớn cho chữ
    painter.drawText(gaugeX - 50, tiltPos + 5, tiltText);  // Vẽ chữ đen
    painter.setPen(QPen(Qt::white, 4));  // Nét trắng nhỏ cho chữ
    painter.drawText(gaugeX - 50, tiltPos + 5, tiltText);  // Vẽ chữ trắng nổi
}
void MainWindow::Paint_Pan_Status(QPainter &painter, double Pan_Angle) {
    // Thiết lập kích thước và vị trí của đồng hồ ở góc dưới bên trái
    int width = 250;  // Đường kính đồng hồ
    int height = 150;
    int radius = width / 2;
    QPoint center(100 + radius, this->height() - 100 - radius);  // Góc dưới bên trái

    // Vẽ vòng tròn nổi đại diện cho đồng hồ (chỉ viền, không tô màu bên trong)
    painter.setRenderHint(QPainter::Antialiasing);

    // Vẽ nét viền bên ngoài (màu đen, nét to hơn)
    painter.setPen(QPen(Qt::black, 6));
    painter.setBrush(Qt::NoBrush);  // Không tô màu bên trong đồng hồ

    painter.drawEllipse(center, radius, radius);

    // Vẽ nét viền bên trong (màu trắng, nét nhỏ hơn)
    painter.setPen(QPen(Qt::white, 4));
    painter.setBrush(Qt::NoBrush);  // Không tô màu bên trong đồng hồ
    painter.drawEllipse(center, radius, radius);

    // Vẽ các vạch chia độ (0 độ tại vị trí 12 giờ)
    int numTicks = 36; // 10 độ mỗi vạch
    for (int i = 0; i < numTicks; ++i) {
        double angle_deg = i * (360.0 / numTicks) - 90; // -90 để 0 độ tại vị trí 12 giờ
        double angle_rad = qDegreesToRadians(static_cast<double>(angle_deg));
        int line_length = (i % 3 == 0) ? 15 : 8; // Vạch lớn và nhỏ

        // Tính vị trí của các vạch
        int x1 = center.x() + (radius - 10) * qCos(angle_rad);
        int y1 = center.y() + (radius - 10) * qSin(angle_rad);
        int x2 = center.x() + (radius - 10 - line_length) * qCos(angle_rad);
        int y2 = center.y() + (radius - 10 - line_length) * qSin(angle_rad);

        // Vẽ vạch chia độ nổi
        painter.setPen(QPen(Qt::black, 3));  // Nét đen dày hơn
        painter.drawLine(x1, y1, x2, y2);

        painter.setPen(QPen(Qt::white, 1));  // Nét trắng mảnh hơn
        painter.drawLine(x1, y1, x2, y2);
    }


    // Vẽ kim đồng hồ hình tam giác cân mô phỏng góc Pan của camera
    double pan_angle_rad = qDegreesToRadians(static_cast<double>(-Pan_Angle) - 90); // -90 để 0 độ tại vị trí 12 giờ
    int x_tip = center.x() + (radius - 20) * qCos(pan_angle_rad);
    int y_tip = center.y() + (radius - 20) * qSin(pan_angle_rad);

    // Tạo tam giác cân làm kim đồng hồ
    QPointF tip(x_tip, y_tip);  // Đỉnh tam giác (đầu kim đồng hồ)
    QPointF left(center.x() + (radius - 40) * qCos(pan_angle_rad + qDegreesToRadians(10.0)),
                 center.y() + (radius - 40) * qSin(pan_angle_rad + qDegreesToRadians(10.0)));
    QPointF right(center.x() + (radius - 40) * qCos(pan_angle_rad - qDegreesToRadians(10.0)),
                  center.y() + (radius - 40) * qSin(pan_angle_rad - qDegreesToRadians(10.0)));

    // Vẽ tam giác viền (đen)
    painter.setPen(QPen(Qt::black, 6));  // Nét đen to
    painter.setBrush(QBrush(Qt::NoBrush)); // Không tô màu
    painter.drawPolygon(QPolygonF() << center << left << right);

    // Vẽ tam giác bên trong (cam)
    painter.setPen(QPen(QColor(255, 109, 1), 3));  // Nét cam mảnh hơn
    painter.drawPolygon(QPolygonF() << center << left << right);

    // Hiển thị giá trị Pan Angle bên ngoài đồng hồ
    int pan_text_x = center.x() + (radius + 15) * qCos(pan_angle_rad);
    int pan_text_y = center.y() + (radius + 15) * qSin(pan_angle_rad);

    // Vẽ giá trị Pan Angle nổi
    QString pan_text = QString::number(Pan_Angle, 'f', 1) + "°";
    painter.setPen(QPen(Qt::black, 6));  // Nét đen viền
    painter.drawText(pan_text_x - 10, pan_text_y + 10, pan_text);

    painter.setPen(QPen(Qt::white, 2));  // Nét trắng mảnh hơn
    painter.drawText(pan_text_x - 10, pan_text_y + 10, pan_text);
}
void MainWindow::drawObjectInCenter(QPainter &painter) {


    // Kích thước của hình chữ nhật (1/4 khung hình)
    int rectWidth = width() / 4;  // Chiều rộng
    int rectHeight = height() / 4; // Chiều cao

    // Tính toán vị trí chính giữa của widget
    int centerX = width() / 2;
    int centerY = height() / 2;

    // Tính toán tọa độ của bốn góc
    QPoint topLeft(centerX - rectWidth / 2, centerY - rectHeight / 2);
    QPoint topRight(centerX + rectWidth / 2, centerY - rectHeight / 2);
    QPoint bottomLeft(centerX - rectWidth / 2, centerY + rectHeight / 2);
    QPoint bottomRight(centerX + rectWidth / 2, centerY + rectHeight / 2);

    // Vẽ bốn góc vuông của hình chữ nhật
    QPen whitePen(Qt::white);
    whitePen.setWidth(2);

    QPen blackPen1(Qt::black);
    blackPen1.setWidth(4);

    // Kích thước của các góc vuông
    int cornerSize = 45; // Kích thước của các góc vuông
    painter.setPen(blackPen1);
    painter.drawLine(topLeft, QPoint(topLeft.x() + cornerSize, topLeft.y()));
    painter.drawLine(topLeft, QPoint(topLeft.x(), topLeft.y() + cornerSize));

    painter.setPen(whitePen);
    painter.drawLine(topLeft, QPoint(topLeft.x() + cornerSize, topLeft.y()));
    painter.drawLine(topLeft, QPoint(topLeft.x(), topLeft.y() + cornerSize));
    //-------
    painter.setPen(blackPen1);
    painter.drawLine(topRight, QPoint(topRight.x() - cornerSize, topRight.y()));
    painter.drawLine(topRight, QPoint(topRight.x(), topRight.y() + cornerSize));

    painter.setPen(whitePen);
    painter.drawLine(topRight, QPoint(topRight.x() - cornerSize, topRight.y()));
    painter.drawLine(topRight, QPoint(topRight.x(), topRight.y() + cornerSize));

    //-------
    painter.setPen(blackPen1);
    painter.drawLine(bottomLeft, QPoint(bottomLeft.x() + cornerSize, bottomLeft.y()));
    painter.drawLine(bottomLeft, QPoint(bottomLeft.x(), bottomLeft.y() - cornerSize));

    painter.setPen(whitePen);
    painter.drawLine(bottomLeft, QPoint(bottomLeft.x() + cornerSize, bottomLeft.y()));
    painter.drawLine(bottomLeft, QPoint(bottomLeft.x(), bottomLeft.y() - cornerSize));

    //-------
    painter.setPen(blackPen1);
    painter.drawLine(bottomRight, QPoint(bottomRight.x() - cornerSize, bottomRight.y()));
    painter.drawLine(bottomRight, QPoint(bottomRight.x(), bottomRight.y() - cornerSize));

    painter.setPen(whitePen);
    painter.drawLine(bottomRight, QPoint(bottomRight.x() - cornerSize, bottomRight.y()));
    painter.drawLine(bottomRight, QPoint(bottomRight.x(), bottomRight.y() - cornerSize));

    // Vẽ chữ 'T' màu đỏ ở giữa hình chữ nhật
    QPen redPen(Qt::red);
    redPen.setWidth(5);
    painter.setPen(redPen);

    // Đoạn thẳng ngang
    int shortLineLength = rectWidth / 2; // Độ dài đoạn thẳng ngang
    int gap = 10; // Khoảng trống giữa các đoạn thẳng

    // Vẽ đoạn thẳng ngang bị đứt khúc
    painter.drawLine(centerX - shortLineLength / 2, centerY, centerX - shortLineLength / 2 + gap, centerY);
    painter.drawLine(centerX + shortLineLength / 2 - gap, centerY, centerX + shortLineLength / 2, centerY);

    // Vẽ đoạn thẳng dọc của chữ 'T' (đi xuống)
    int verticalLineLength = 30; // Độ dài đoạn thẳng dọc
    painter.drawLine(centerX, centerY, centerX, centerY + verticalLineLength);

    // Vẽ viền đen cho các đoạn thẳng
    QPen blackPen(Qt::black);
    blackPen.setWidth(1);
    painter.setPen(blackPen);

    // Vẽ viền cho đoạn ngang bị đứt khúc
    painter.drawLine(centerX - shortLineLength / 2, centerY, centerX - shortLineLength / 2 + gap, centerY);
    painter.drawLine(centerX + shortLineLength / 2 - gap, centerY, centerX + shortLineLength / 2, centerY);

    // Vẽ viền cho đoạn thẳng dọc
    painter.drawLine(centerX, centerY, centerX, centerY + verticalLineLength);
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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
    frame.append(static_cast<char>(checksum)); // Append checksum

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
