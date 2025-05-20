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
    qDebug() << "Current time:" << time->toString("dddd dd-M-yy hh:mm");
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

    if( input_name == "topic" || input_pass== "topic")
    {
        topic topic_dialog;
        topic_dialog.setModal(true);
        topic_dialog.exec();
    } else if (input_name == "service" || input_pass== "service")
    {
        service service_dialog;
        service_dialog.setModal(true);
        service_dialog.exec();
    } else if (input_name == "function" || input_pass== "function")
    {
        function function_dialog;
        function_dialog.setModal(true);
        function_dialog.exec();
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

