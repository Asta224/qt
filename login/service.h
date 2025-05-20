#ifndef SERVICE_H
#define SERVICE_H

#include <QDialog>

namespace Ui {
class service;
}

class service : public QDialog
{
    Q_OBJECT

public:
    explicit service(QWidget *parent = nullptr);
    ~service();

private slots:
    void on_homebtn_clicked();

private:
    Ui::service *ui;
};

#endif // SERVICE_H
