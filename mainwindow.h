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

public:
    void sendPanTiltCommand(bool panRight, float panSpeed, bool tiltDown, float tiltSpeed);
    void sendPelcoDCommand(QByteArray frame);

    void onSerialDataReceived();
    void requestPTZPositions();
    QByteArray createPelcoCommand(quint8 command);
    quint8 calculateChecksum(const QByteArray &packet);
    void drawPanGauge(QPainter &painter, int panAngle);
    void drawTiltGauge(QPainter &painter, int tiltAngle);
    void drawObjectInCenter(QPainter &painter);
protected slots:
    void timerEvent(QTimerEvent *event) override;
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

    float currentPan, currentTilt, currentZoom;
    Manual_Control *m_Manual_Control;
    bool Auto_tracking = false;


};

#endif // MAINWINDOW_H
