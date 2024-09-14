#include "manual_control.h"
#include "ui_manual_control.h"

Manual_Control::Manual_Control(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Manual_Control)
{
    ui->setupUi(this);

//    setWindowFlags(Qt::FramelessWindowHint);

    // Thiết lập cửa sổ có nền trong suốt
    setAttribute(Qt::WA_TranslucentBackground);
}

Manual_Control::~Manual_Control()
{
    delete ui;
}

void Manual_Control::on_upButton_clicked()
{
    emit UpSignal();
    qDebug()<<"Up signal";

}

void Manual_Control::on_downButton_clicked()
{
    emit DownSignal();
}

void Manual_Control::on_leftButton_clicked()
{
    emit LeftSignal();
}

void Manual_Control::on_rightButton_clicked()
{
    emit RightSignal();
}

void Manual_Control::on_FocusInButton_clicked()
{
    emit FocusInSignal();
}

void Manual_Control::on_FocusOutButton_clicked()
{
    emit FocusOutSignal();
}

void Manual_Control::on_RequesStButton_clicked()
{

}

void Manual_Control::on_zoomInButton_clicked()
{
    emit ZoomInSignal();
}

void Manual_Control::on_zoomOutButton_clicked()
{
    emit ZoomOutSignal();
}

void Manual_Control::on_stopButton_clicked()
{
    emit StopSignal();
}

