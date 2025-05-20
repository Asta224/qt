#ifndef FUNCTION_H
#define FUNCTION_H

#include <QDialog>

namespace Ui {
class function;
}

class function : public QDialog
{
    Q_OBJECT

public:
    explicit function(QWidget *parent = nullptr);
    ~function();

private slots:
    void on_homeBtn_clicked();

private:
    Ui::function *ui;
};

#endif // FUNCTION_H
