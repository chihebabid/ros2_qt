#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include "mywidget.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    rclcpp::init(argc, argv);
    MyWidget myWidget;
    myWidget.show();
    return a.exec();
}
