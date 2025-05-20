#include "form.h"
#include "ui_form.h"

Form::Form(const QString &personName, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Form)
{
    ui->setupUi(this);

    peopleInfo["MgMg"] = QStringList({"Name: Mg Mg", "Age: 25", "Phone: 09-123456789", "Address: Mandalay"});
    peopleInfo["KoKo"] = QStringList({"Name: Ko Ko", "Age: 30", "Phone: 09-987654321", "Address: Yangon"});
    peopleInfo["PhyuPhyu"] = QStringList({"Name: Phyu Phyu", "Age: 28", "Phone: 09-456789123", "Address: Naypyitaw"});

    QListWidget *infoList = ui->tabWidget->findChild<QListWidget*>("infoList");
    if (infoList) {
        infoList->clear();
        if (peopleInfo.contains(personName)) {
            infoList->addItems(peopleInfo[personName]);
        } else {
            infoList->addItem("No information available");
        }
    }

    ui->tabWidget->setCurrentIndex(0);
}

Form::~Form()
{
    delete ui;
}

void Form::on_closeButton_clicked()
{
    this->close();
}
