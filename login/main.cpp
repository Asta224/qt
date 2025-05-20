#include "mainwindow.h"
#include <QFile>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QFile file("/home/thukha/qtcreator/login/style.qss");
    if(file.open(QFile::ReadOnly | QFile::Text)){
       
        a.setStyleSheet(file.readAll());
    }
    MainWindow w;
    w.show();
    return a.exec();
}
