#include <QApplication>
#include "mainwindow.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  

    // Initialize Qt Application
    QApplication a(argc, argv);

    // Create and show your main window
    MainWindow w;
    w.show();
    return a.exec();
}