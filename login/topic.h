#ifndef TOPIC_H
#define TOPIC_H

#include <QDialog>

namespace Ui {
class topic;
}

class topic : public QDialog
{
    Q_OBJECT

public:
    explicit topic(QWidget *parent = nullptr);
    ~topic();

private slots:
    void on_homeBtn_clicked();

private:
    Ui::topic *ui;
};

#endif // TOPIC_H
