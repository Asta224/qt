#include "topic.h"
#include "ui_topic.h"

topic::topic(QMainWindow *mainwindow,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::topic),
    m_mainwindow(mainwindow)
{
    ui->setupUi(this);
}

topic::~topic()
{
    delete ui;
}

void topic::on_homeBtn_clicked()
{
    this->close();
    m_mainwindow->show();

}

