#ifndef FUNCTION_H
#define FUNCTION_H

#include <QDialog>
#include "mainwindow.h"

namespace Ui {
class function;
}

class function : public QDialog
{
    Q_OBJECT

public:
    explicit function(QMainWindow *mainwindow, QWidget *parent = nullptr);
    ~function();

private slots:
    void on_homeBtn_clicked();

private:
    Ui::function *ui;
    QMainWindow *m_mainwindow;
};

#endif // FUNCTION_H
