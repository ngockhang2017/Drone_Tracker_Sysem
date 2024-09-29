#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QLabel>
#include <QImage>
#include <QByteArray>
#include <QRect>
#include <QSerialPort>
#include<QPushButton>
#include<QVBoxLayout>
#include<QStackedLayout>
#include<manual_control.h>

#include <algorithm>
#include <cmath>
#include<QTimer>
#include <QDateTime>
#include <QFont>
#include <QColor>
#include<QEventLoop>

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
#include <QtMath>
#include <QPainter>
#include <QWidget>
// Giả định rằng bạn có một lớp MainWindow kế thừa từ QWidget
#include <QPainter>
#include <QWidget>
#include <QPainter>
#include <QWidget>
#include<QDesktopServices>
#include<QNetworkAccessManager>
#include <QNetworkReply>
typedef  QVector<double> DOASample;


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

private slots:
    void processPendingImageDatagrams();
    void processPendingBboxDatagrams();
    void on_actionControl_Manual_triggered();

    void UpSlot();
    void DownSlot();
    void LeftSlot();
    void RightSlot();
    void FocusInSlot();
    void FocusOutSlot();
    void ZoomInSlot();
    void ZoomOutSlot();
    void StopSlot();



    void on_actionFull_Screen_triggered();

    void on_actionMinimize_Screen_triggered();

    void on_actionExit_triggered();

    void on_actionStart_Auto_tracking_triggered();

    void on_actionStop_Auto_tracking_triggered();

    void updateBlinking();

public:
    void sendPanTiltCommand(bool panRight, float panSpeed, bool tiltDown, float tiltSpeed);
    void sendPelcoDCommand(QByteArray frame);

    void onSerialDataReceived();

    QByteArray createPelcoCommand(quint8 command);
    quint8 calculateChecksum(const QByteArray &packet);
    void drawPanGauge(QPainter &painter, int panAngle);
    void drawTiltGauge(QPainter &painter, double tiltAngle);
    void ObjectTracking(const QRect &bbox, bool isPan);
    void drawObjectInCenter(QPainter &painter);

    void Absolute_Pan_Position(double pan_angle);
    void Absolute_Tilt_Position(double tilt_angle);
    void requestPosition(QSerialPort *serialPort, bool isPan);
    void updatePosition(QByteArray response, double &panCurrent, double &tiltCurrent);

    void CheckDOA();
    void CheckTiltStep();
    void UpdatePanTiltStatus();
    void Paint_Pan_Status(QPainter &painter, double Pan_Angle);

protected slots:
    void timerEvent(QTimerEvent *event) override;

public slots:
    void networkReplyKraken(QNetworkReply *reply);
    void Request();

signals:
    void PanPosition_Updated_signal();
    void TiltPosition_Updated_signal();

    void Tilt_Adjust_finised();

private:
    QByteArray frameControl;
    Ui::MainWindow *ui;
    QUdpSocket *imageSocket;
    QUdpSocket *bboxSocket;
    QLabel *imageLabel;
    QImage currentImage;
    QByteArray receivedImageData;
    QRect currentBoundingBox;
    QSerialPort *ptzSerialPort;

    double currentPanAngle = -1.0, currentTiltAngle = -1.0;
    Manual_Control *m_Manual_Control;
    bool Auto_tracking = false;

    bool isBlinkingVisible = false;
    QTimer *timer_request, *blinkTimer, *update_doa, *update_pantilt, *tilt_check_by_step_timer;

    int count = 0, count1 = 0;
    bool pan_adjust_finised =false;

    int deltaX = 0, deltaY = 0;
    double Pan_Expectaion = -1.0 , Tilt_Expectation = -1.0;
    bool Object_Detected = false;
    int current_doa_check = 0.0;
    double tilt_check_by_step = 40.0;

    QNetworkAccessManager *manager;
    QVector<DOASample> allSample;
    bool stop_check_pan = false;

};

#endif // MAINWINDOW_H
