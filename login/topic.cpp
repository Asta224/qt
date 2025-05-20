#include "topic.h"
#include "ui_topic.h"

topic::topic(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::topic)
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
}

