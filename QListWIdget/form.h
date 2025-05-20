#ifndef FORM_H
#define FORM_H

#include <QDialog>
#include <QMap>
#include <QStringList>
#include <QListWidgetItem>

namespace Ui {
class Form;
}

class Form : public QDialog
{
    Q_OBJECT

public:
    explicit Form(const QString &personName, QWidget *parent = nullptr);
    ~Form();

private slots:
    void on_closeButton_clicked();

private:
    Ui::Form *ui;
    QMap<QString, QStringList> peopleInfo;
};

#endif // FORM_H
