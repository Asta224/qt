#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QTextStream>
#include <QLabel>
#include <QPixmap>
#include <QTableWidgetItem>
#include <QBrush>
#include <QFont>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->tableWidget->setColumnCount(5);
    ui->tableWidget->setHorizontalHeaderLabels({"No", "Description", "Image", "Price", "Remark"});
    loadCsvAndPopulateTable("/home/thukha/qtcreator/testlist/dataRobot Estimate Cost - Sheet1.csv");
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::loadCsvAndPopulateTable(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

    QTextStream in(&file);
    in.readLine(); // skip first line (header/comment)

    QString currentCategory;
    int row = 0;

    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(",");

        if (parts.size() < 2) continue;

        QString no = parts[0].trimmed();
        QString desc = parts[1].trimmed();
        QString img = (parts.size() > 2) ? parts[2].trimmed() : "";
        QString price = (parts.size() > 3) ? parts[3].trimmed() : "";
        QString remark = (parts.size() > 5) ? parts[5].trimmed() : "";

        if (!no.isEmpty() && desc.isEmpty()) {
            currentCategory = no;
            continue;
        }

        if (desc.isEmpty()) continue;

        ui->tableWidget->insertRow(row);
        ui->tableWidget->setItem(row, 0, new QTableWidgetItem(no));
        ui->tableWidget->setItem(row, 1, new QTableWidgetItem(desc));

        QLabel *imageLabel = new QLabel;
        QPixmap pix("images/" + img);
        if (!pix.isNull())
            imageLabel->setPixmap(pix.scaled(64, 64, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        imageLabel->setAlignment(Qt::AlignCenter);
        ui->tableWidget->setCellWidget(row, 2, imageLabel);

        ui->tableWidget->setItem(row, 3, new QTableWidgetItem(price));
        ui->tableWidget->setItem(row, 4, new QTableWidgetItem(remark));
        row++;
    }
    ui->tableWidget->resizeColumnsToContents();
}

