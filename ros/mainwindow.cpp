#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
     this->setStyleSheet(
    "#centralwidget { border-image: url(:data/logo1.jpg) 0 0 0 0 stretch stretch; background-color: #E0E0E0; }"
    "#loginForm { "
        "border: 1px solidrgba(160, 160, 160, 0.5); "
        "background-color:rgba(240, 240, 240, 0.5); "
        "border-radius: 4px; "
    "}"
    "#topPanel { background-color: #607D8B; }"
    "QLabel { color: #333333; }"
    "QLineEdit { "
        "border: 1px solid #A0A0A0; "
        "border-radius: 3px; "
        "background-color: white; "
        "color: #333333; "
        "padding: 2px;"
    "}"
    "QPushButton { "
        "color: white; "
        "background-color:hsl(194, 77.90%, 46.10%); "
        "border: 1px solidhsl(0, 0.00%, 30.20%); "
        "border-radius: 3px; "
        "padding: 5px 10px; "
    "}"
    "QPushButton:hover { "
        "background-color:rgb(207, 37, 193); "
        "border: 1px solidhsl(120, 2.50%, 31.40%); "
    "}"
    "QPushButton:pressed { "
        "background-color:hsl(332, 100.00%, 49.80%); "
        "border: 1px solidhsl(311, 74.20%, 18.20%); "
    "}"
);
ui->loginButton->setDefault(true);
   
ui->currentDateTime->setStyleSheet("color:hsl(0, 33.30%, 81.80%);");
ui->username->setStyleSheet("color: #333333;");
ui->password->setStyleSheet("color: #333333;");
    QDateTime *time = new QDateTime(QDateTime::currentDateTime());
    // qDebug() << "Current time:" << time->toString("dddd dd-M-yy hh:mm");
    ui->currentDateTime->setText(time->toString("dddd dd-M-yy hh:mm"));
 
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
    ui->passwordEdit->setFocus();
}

