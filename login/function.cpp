#include "function.h"
#include "ui_function.h"

function::function(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::function)
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
}

