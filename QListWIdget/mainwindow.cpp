#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDialog>
#include "form.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->listWidget->addItem(new QListWidgetItem(QIcon(":/img/image/content.jpg"), "MgMg"));
    ui->listWidget->addItem(new QListWidgetItem(QIcon(":/img/image/content.jpg"), "KoKo"));
    ui->listWidget->addItem(new QListWidgetItem(QIcon(":/img/image/content.jpg"), "PhyuPhyu"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QListWidgetItem *item = ui->listWidget->currentItem();
    if (!item) {
        qDebug() << "No item selected.";
        return;
    }

    QString name = item->text();
    Form form_dialog(name);
    form_dialog.setModal(true);
    form_dialog.exec();

    qDebug() << "Form widget opened with:" << name;
}
