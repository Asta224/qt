#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setStyleSheet("#centralwidget{border-image: url(:logo1.jpg) 0 0 0 0 stretch stretch;}" // centralwidget ၏ နောက်ခံပုံကို logo1.jpg ဖြင့် သတ်မှတ်ခြင်း (Set background image of centralwidget to logo1.jpg)
                        "#loginForm { border: 1px solid black; background: rgba(0, 0, 0, 80);border-radius: 8px; }" // loginForm ၏ ဘောင်၊ နောက်ခံအရောင်နှင့် ထောင့်များကို သတ်မှတ်ခြင်း (Set border, background color, and border-radius for loginForm)
                        "#centralwidget {background: rgba(32,80,96,100);}"// centralwidget ၏ နောက်ခံအရောင်ကို သတ်မှတ်ခြင်း (Set background color for centralwidget)
                        "#topPanel { background-color:qlineargradient(spread:reflect, x1:0.5, y1:0, x2:0, y2:0,stop:0 rgba(91, 204, 233, 100), stop:1 rgba(32, 80, 96,100)); }"
                    
                        "QLineEdit { border-radius: 3px; }"
                        "QPushButton{color: white;background-color: #27a9e3;border-width: 0px;border-radius: 3px;}"
                        "QPushButton:hover { background-color: #66c011; }");// QPushButton ပေါ်တွင် mouse တင်လိုက်သောအခါ နောက်ခံအရောင် ပြောင်းလဲခြင်း 
    QDateTime *time = new QDateTime(QDateTime::currentDateTime()); // လက်ရှိအချိန်နှင့် ရက်စွဲကို ရယူခြင်း 
    
    ui->currentDateTime->setText(time->toString("dddd dd-M-yy hh:mm"));// currentDateTime QLabel တွင် လက်ရှိအချိန်နှင့် ရက်စွဲကို ပြသခြင်
    ui->currentDateTime->setStyleSheet("color: white;");
    ui->username->setStyleSheet("color: white");
    ui->password->setStyleSheet("color: white");
    animation1 = new QPropertyAnimation (ui->loginForm,"geometry");
    animation1 -> setDuration(3000);
    animation1 -> setStartValue(ui->loginForm->geometry());
    animation1 -> setEndValue(QRect(197,320,350,200));
    
    QEasingCurve curve; // easing curve object အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new easing curve object)
    curve.setType(QEasingCurve::OutBounce); // easing curve အမျိုးအစားကို OutBounce အဖြစ် သတ်မှတ်ခြင်း (Set easing curve type to OutBounce)
    curve.setAmplitude(1.00); // amplitude ကို 1.00 အဖြစ် သတ်မှတ်ခြင်း (Set amplitude to 1.00)
    curve.setOvershoot(1.70); // overshoot ကို 1.70 အဖြစ် သတ်မှတ်ခြင်း (Set overshoot to 1.70)
    curve.setPeriod(0.30); // period ကို 0.30 အဖြစ် သတ်မှတ်ခြင်း (Set period to 0.30)
    animation1 -> setEasingCurve (curve); // animation1 တွင် easing curve ကို သတ်မှတ်ခြင်း (Set the easing curve for animation1)

    group = new QParallelAnimationGroup; // animation များကို တစ်ပြိုင်နက်တည်း လုပ်ဆောင်ရန် QParallelAnimationGroup object အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new QParallelAnimationGroup object to run animations in parallel)
    group->addAnimation(animation1); // group ထဲသို့ animation1 ကို ထည့်သွင်းခြင်း (Add animation1 to the group)
    group->start(); // animation group ကို စတင်ခြင်း (Start the animation group)
}
MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_loginButton_clicked()
{
    check_password();
}
// password စစ်ဆေးသည့် private function ကို အကောင်အထည်ဖော်ခြင်း (Implement the private check_password function)
void MainWindow::check_password()
{
    QString input_name = ui->usernameEdit->text(); // usernameEdit မှ စာသားကို ရယူခြင်း (Get text from usernameEdit)
    QString input_pass = ui->passwordEdit->text(); // passwordEdit မှ စာသားကို ရယူခြင်း (Get text from passwordEdit)

    if( input_name == "topic" && input_pass== "topic") // username "topic" နှင့် password "topic" ဖြစ်ပါက (If username is "topic" and password is "topic")
    {   
        this->hide(); // လက်ရှိ window ကို ဖျောက်ထားခြင်း (Hide the current window)
        topic *t_opic = new topic(this); // topic window အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new topic window)
        t_opic->show(); // topic window ကို ပြသခြင်း (Show the topic window)

    } else if (input_name == "service" && input_pass== "service") // username "service" နှင့် password "service" ဖြစ်ပါက (If username is "service" and password is "service")
    {
        this->hide(); // လက်ရှိ window ကို ဖျောက်ထားခြင်း (Hide the current window)
        service *s_ervice = new service(this); // service window အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new service window)
        s_ervice->show(); // service window ကို ပြသခြင်း (Show the service window)

    } else if (input_name == "function" && input_pass== "function") // username "function" နှင့် password "function" ဖြစ်ပါက (If username is "function" and password is "function")
    {
        this->hide(); // လက်ရှိ window ကို ဖျောက်ထားခြင်း (Hide the current window)
        function *f_unction = new function(this); // function window အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new function window)
        f_unction->show(); // function window ကို ပြသခြင်း (Show the function window)
    } else // အခြား username/password များ ဖြစ်ပါက (For any other username/password)
    {
        QMessageBox::warning(this, "Error", "Invalid username or password"); // "Invalid username or password" ဟူသော သတိပေးချက်ကို ပြသခြင်း (Display a warning message "Invalid username or password")
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

