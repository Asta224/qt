#include "function.h"
#include "ui_function.h"

function::function(QMainWindow *mainwindow, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::function),
    m_mainwindow(mainwindow)
{
    ui->setupUi(this);
}

function::~function()
{
    delete ui;
}

void function::on_homeBtn_clicked()
{
    this->close();
    m_mainwindow->show();
}

