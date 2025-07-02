#ifndef ACTION_H
#define ACTION_H

#include <QDialog>
#include "mainwindow.h"

namespace Ui {
class action;
}

class action : public QDialog
{
    Q_OBJECT

public:
     explicit action(QMainWindow *mainwindow,QWidget *parent = nullptr);
    ~action();
private slots:
    void on_homeButton_clicked();
private:
    Ui::action *ui;
    QMainWindow *m_mainwindow;

};

#endif // ACTION_H
