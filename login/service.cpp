#include "service.h"
#include "ui_service.h"

service::service(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::service)
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
}

