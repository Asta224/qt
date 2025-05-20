#include "service.h"
#include "ui_service.h"

service::service(QMainWindow *maindwidow,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::service),
    m_mainwindow(maindwidow)
{
    ui->setupUi(this);
}

service::~service()
{
    delete ui;
}

void service::on_homebtn_clicked()
{
    this->close();
    m_mainwindow->show();
}

