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
};
#endif // MAINWINDOW_H
