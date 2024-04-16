#include "mywidget.h"
#include "ui_mywidget.h"

MyWidget::MyWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MyWidget),mControlCam(std::make_shared<ControlCAM>())
{
    ui->setupUi(this);
    connect(ui->pbQuit,&QPushButton::clicked,qApp,&QApplication::quit);

    connect(ui->pbCamON,&QPushButton::clicked,[this]() {
        mControlCam->turnON();
    });

    connect(ui->pbCamOFF,&QPushButton::clicked,[this]() {
        mControlCam->turnOFF();
    });

    connect(mControlCam.get(),&ControlCAM::imageLoaded,[this](const QImage &qImg) {
        ui->mImage->setPixmap(QPixmap::fromImage(qImg));
        return;
    });

    QPixmap emptyPixmap(320, 240); // Crée un QPixmap vide avec les dimensions souhaitées
    emptyPixmap.fill(Qt::black);
    ui->mImage->setPixmap(emptyPixmap);    
}

MyWidget::~MyWidget()
{
    delete ui;
}
