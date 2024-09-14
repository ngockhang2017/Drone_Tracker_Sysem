#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <QMainWindow>
#include<QDebug>

namespace Ui {
class Manual_Control;
}

class Manual_Control : public QMainWindow
{
    Q_OBJECT

public:
    explicit Manual_Control(QWidget *parent = nullptr);
    ~Manual_Control();

private slots:
    void on_upButton_clicked();

    void on_downButton_clicked();

    void on_leftButton_clicked();

    void on_rightButton_clicked();

    void on_FocusInButton_clicked();

    void on_FocusOutButton_clicked();

    void on_RequesStButton_clicked();

    void on_zoomInButton_clicked();

    void on_zoomOutButton_clicked();

    void on_stopButton_clicked();



signals:
    void UpSignal();
    void DownSignal();
    void LeftSignal();
    void RightSignal();
    void FocusInSignal();
    void FocusOutSignal();
    void ZoomInSignal();
    void ZoomOutSignal();
    void StopSignal();

private:
    Ui::Manual_Control *ui;
};

#endif // MANUAL_CONTROL_H
