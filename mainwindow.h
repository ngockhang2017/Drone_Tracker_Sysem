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

    void trackObjectWithPTZ(const QRect &bbox, int zoomLevel);

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
    void drawTiltGauge(QPainter &painter, int tiltAngle);
    void ObjectTracking(const QRect &bbox);
    void drawObjectInCenter(QPainter &painter);

    void Absolute_Pan_Position(double pan_angle);
    void Absolute_Tilt_Position(double tilt_angle);
    void requestPosition(QSerialPort *serialPort, bool isPan);
    void updatePosition(QByteArray response, double &panCurrent, double &tiltCurrent);

protected slots:
    void timerEvent(QTimerEvent *event) override;

signals:
    void PanPosition_Updated_signal();
     void TiltPosition_Updated_signal();

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

    double currentPanAngle, currentTiltAngle;
    Manual_Control *m_Manual_Control;
    bool Auto_tracking = false;

    bool isBlinkingVisible = false;
    QTimer *blinkTimer;

    int count = 0;
};

#endif // MAINWINDOW_H
