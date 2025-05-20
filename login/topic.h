#ifndef TOPIC_H
#define TOPIC_H

#include <QDialog>
#include "mainwindow.h"
namespace Ui {
class topic;
}

class topic : public QDialog
{
    Q_OBJECT

public:
    explicit topic(QMainWindow *mainwindow,QWidget *parent = nullptr);
    ~topic();

private slots:
    void on_homeBtn_clicked();

private:
    Ui::topic *ui;
    QMainWindow *m_mainwindow;
};

#endif // TOPIC_H
