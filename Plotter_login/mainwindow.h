#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileInfo>
#include<QTextStream>
#include "topic.h"
#include <QDialog>
#include "service.h"
#include "function.h"
#include <QMessageBox>
#include <QPropertyAnimation>  // UI Element များကို ကာတွန်းပုံစံ လှုပ်ရှားမှုများ ပြုလုပ်ရန်အတွက် QPropertyAnimation ကို ထည့်သွင်းခြင်း (Include QPropertyAnimation for animating UI elements)
#include <QParallelAnimationGroup> // ကာတွန်းပုံစံ လှုပ်ရှားမှုများစွာကို တစ်ပြိုင်နက်တည်း လုပ်ဆောင်ရန်အတွက် QParallelAnimationGroup ကို ထည့်သွင်းခြင်း (Include QParallelAnimationGroup for running multiple animations simultaneously)

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void clear_and_focus();
private slots:
    void on_loginButton_clicked();

    void on_usernameEdit_editingFinished();

    void on_passwordEdit_editingFinished();

private:
    Ui::MainWindow *ui;
    void check_password();
    QPropertyAnimation *animation1; // ပထမဆုံး animation အတွက် QPropertyAnimation object (QPropertyAnimation object for the first animation)
    QPropertyAnimation *animation2; // ဒုတိယ animation အတွက် QPropertyAnimation object (QPropertyAnimation object for the second animation)
    QPropertyAnimation *animation3; // တတိယ animation အတွက် QPropertyAnimation object (QPropertyAnimation object for the third animation)
    QPropertyAnimation *animation4; // စတုတ္ထ animation အတွက် QPropertyAnimation object (QPropertyAnimation object for the fourth animation)
    QParallelAnimationGroup *group; // animation များကို တစ်ပြိုင်နက်တည်း run ရန်အတွက် animation group object (Animation group object to run animations simultaneously)
   
   

};
#endif // MAINWINDOW_H
