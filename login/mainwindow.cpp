#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // ui->logoLabel->setPixmap(QPixmap(":/image2.jpg"));
    ui->currentDateTime->setStyleSheet("QLabel { color: white; }");
    ui->logoLabel->setStyleSheet("image: url(:/image2.jpg);");
    this->setStyleSheet("#centralwidget{border-image: url(:image.jpeg) 0 0 0 0 stretch stretch;}"
                        "#loginForm { border: 1px solid black; background: rgba(0, 0, 0, 80);border-radius: 8px; }"
                        "#centralwidget {background: rgba(32,80,96,100);}"
                        "#topPanel { background-color:qlineargradient(spread:reflect, x1:0.5, y1:0, x2:0, y2:0,stop:0 rgba(91, 204, 233, 100), stop:1 rgba(32, 80, 96,100)); }"
                        "QLabel { color: grey; }"
                        "QLineEdit { border-radius: 3px; }"
                        "QPushButton{color: white;background-color: #27a9e3;border-width: 0px;border-radius: 3px;}"
                        "QPushButton:hover { background-color: #66c011; }");
    QDateTime *time = new QDateTime(QDateTime::currentDateTime());
    // qDebug() << "Current time:" << time->toString("dddd dd-M-yy hh:mm");
    ui->currentDateTime->setText(time->toString("dddd dd-M-yy hh:mm"));
}
MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_loginButton_clicked()
{
    check_password();
}

void MainWindow::check_password()
{
    QString input_name = ui->usernameEdit->text();
    QString input_pass = ui->passwordEdit->text();

    if( input_name == "topic" && input_pass== "topic")
    {   
        this->hide();
        topic *t_opic = new topic(this);
        t_opic->show();

    } else if (input_name == "service" && input_pass== "service")
    {
        this->hide();
        service *s_ervice = new service(this);
        s_ervice->show();

    } else if (input_name == "function" && input_pass== "function")
    {
        this->hide();
        function *f_unction = new function(this);
        f_unction->show();
    } else
    {
        QMessageBox::warning(this, "Error", "Invalid username or password");
    }
}
void MainWindow::clear_and_focus()
{
    ui->usernameEdit->clear();
    ui->passwordEdit->clear();
    ui->usernameEdit->setFocus();
}

void MainWindow::on_usernameEdit_editingFinished()
{
    ui->passwordEdit->setFocus();
    
}


void MainWindow::on_passwordEdit_editingFinished()
{
    ui->centralwidget->setFocus();
}

