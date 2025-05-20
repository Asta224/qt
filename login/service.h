#ifndef SERVICE_H
#define SERVICE_H

#include <QDialog>
#include <mainwindow.h>
namespace Ui {
class service;
}

class service : public QDialog
{
    Q_OBJECT

public:
    explicit service(QMainWindow *mainwindow,QWidget *parent = nullptr);
    ~service();

private slots:
    void on_homebtn_clicked();

private:
    Ui::service *ui;
    QMainWindow *m_mainwindow;
};

#endif // SERVICE_H
