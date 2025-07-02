#include "action.h"
#include "ui_action.h"

action::action(QMainWindow *mainwindow, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::action),
    m_mainwindow(mainwindow)
{
    ui->setupUi(this);
}

action::~action()
{
    delete ui;
}

void action::on_homeButton_clicked()
{
    this->close();
    m_mainwindow->show();
}
