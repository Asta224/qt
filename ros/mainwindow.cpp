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
    //ui->currentDateTime->setStyleSheet("QLabel { color: white; }");

    this->setStyleSheet(
                        "#loginForm { border: 1px solid black; background: rgba(0, 0, 0, 80);border-radius: 8px; }"
                        "#centralwidget {background: rgba(32,80,96,100);}"
                        "#topPanel { background-color:qlineargradient(spread:reflect, x1:0.5, y1:0, x2:0, y2:0,stop:0 rgba(91, 204, 233, 100), stop:1 rgba(32, 80, 96,100)); }"
                        //"QLabel { color: white; }"
                        "QLineEdit { border-radius: 3px; }"
                        "QPushButton{color: white;background-color: #27a9e3;border-width: 0px;border-radius: 3px;}"
                        "QPushButton:hover { background-color: #66c011; }");
    QDateTime *time = new QDateTime(QDateTime::currentDateTime());
    // qDebug() << "Current time:" << time->toString("dddd dd-M-yy hh:mm");
    ui->currentDateTime->setText(time->toString("dddd dd-M-yy hh:mm"));
    ui->currentDateTime->setStyleSheet("color: white;");
    ui->username->setStyleSheet("color: white");
    ui->password->setStyleSheet("color: white");
    animation1 = new QPropertyAnimation (ui->loginForm,"geometry");
    animation1 -> setDuration(3000);
    animation1 -> setStartValue(ui->loginForm->geometry());
    animation1 -> setEndValue(QRect(197,320,350,200));
        
    QEasingCurve curve;
    curve.setType(QEasingCurve::OutBounce);
    curve.setAmplitude(1.00);
    curve.setOvershoot(1.70);
    curve.setPeriod(0.30);
    animation1 -> setEasingCurve (curve);

    group = new QParallelAnimationGroup;
    group->addAnimation(animation1);
    group->start();
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

    } else if (input_name == "action" && input_pass== "action")
    {
        this->hide();
        action *a_ction = new action(this);
        a_ction->show();
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

