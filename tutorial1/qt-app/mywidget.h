#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QWidget>
#include <opencv2/opencv.hpp>
#include "controlcam.h"
namespace Ui {
class MyWidget;
}

class MyWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MyWidget(QWidget *parent = nullptr);
    ~MyWidget();

private:
    Ui::MyWidget *ui;
    std::shared_ptr<ControlCAM> mControlCam;

};

#endif // MYWIDGET_H
