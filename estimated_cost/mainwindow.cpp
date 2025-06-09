#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QComboBox>
#include <QStringList>
#include <QSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QDebug>
#include <QFile>
#include <QTextStream>

QMap<int, QStringList> comboOptions = {
    {0, {"Motor", "Motor 1 (120kg)", "Motor 2 (300kg)"}},
    {1, {"Motor Driver", "L2DB4830-CAFR", "L2DB4830-CAFC" }},
    {2, {"IMU", "LIDAR", "IMU", "Ultrasonic"}},
    {3, {"LIDAR", "Rubber Wheel", "Omni Wheel"}},
    {4, {"Frame", "Aluminum Frame", "Steel Frame"}},
    {5, {"Controller", "STM32", "Arduino", "Raspberry Pi"}},
    {6, {"Power Supply", "12V 5A", "24V 5A"}},
    {7, {"Battery", "Battery 24V 20Ah", "Battery 24V 40Ah"}},
    {8, {"Wire Set", "Signal Wires", "Power Cables"}},
    {9, {"Mount", "Bracket A", "Bracket B"}}
};

QMap<QString, int> itemPriceMap;

void loadPricesFromCSV(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open CSV file:" << filePath;
        return;
    }
    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(',');
        if (parts.size() >= 2) {
            QString itemName = parts[0].trimmed();
            bool ok;
            int price = parts[1].trimmed().toInt(&ok);
            if (ok) {
                itemPriceMap[itemName] = price;
            }
        }
    }
    file.close();
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    loadPricesFromCSV("/home/thukha/git/QT/estimated_cost/data/Robot Estimate Cost - Sheet1.csv");

    int itemRows = 10;
    int totalRow = itemRows;
    ui->tableWidget->setRowCount(itemRows + 1);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setColumnWidth(0, 10);
    ui->tableWidget->setColumnWidth(1, 200);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    ui->tableWidget->setColumnWidth(3, 80);
    ui->tableWidget->setColumnWidth(4, 200);
    ui->tableWidget->setColumnWidth(5, 200);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);

    for(int row = 0; row <= itemRows; ++row) {
        ui->tableWidget->setRowHeight(row, 150);
    }

    for (int row = 0; row < itemRows; ++row) {
        ui->tableWidget->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));

        QComboBox *combo = new QComboBox(this);
        combo->addItems(comboOptions[row]);
        ui->tableWidget->setCellWidget(row, 1, combo);

        QWidget *descWidget = new QWidget();
        QLabel *imageLabel = new QLabel();
        QLabel *descLabel = new QLabel("Select an item");
        QHBoxLayout *layout = new QHBoxLayout(descWidget);
        layout->addWidget(imageLabel);
        layout->addWidget(descLabel);
        layout->setContentsMargins(0, 0, 0, 0);
        descWidget->setLayout(layout);
        ui->tableWidget->setCellWidget(row, 2, descWidget);

        QSpinBox *spinBox = new QSpinBox(this);
        spinBox->setRange(0, 100);
        ui->tableWidget->setCellWidget(row, 3, spinBox);

        QTableWidgetItem *unitPriceItem = new QTableWidgetItem("0");
        unitPriceItem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(row, 4, unitPriceItem);

        QTableWidgetItem *totalItem = new QTableWidgetItem("0");
        totalItem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(row, 5, totalItem);

        auto updateTotalCost = [=]() {
            int grandTotal = 0;
            for (int r = 0; r < itemRows; ++r) {
                grandTotal += ui->tableWidget->item(r, 5)->text().toInt();
            }
            QTableWidgetItem *grandtotal = new QTableWidgetItem(QString::number(grandTotal));
            grandtotal->setTextAlignment(Qt::AlignCenter);
            grandtotal->setBackground(Qt::yellow);
            ui->tableWidget->setItem(totalRow, 5, grandtotal);
        };

        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
            int unitPrice = ui->tableWidget->item(row, 4)->text().toInt();
            int total = unitPrice * value;
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total));
            totalItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 5, totalItem);
            updateTotalCost();
        });

        connect(combo, &QComboBox::currentTextChanged, this, [=](const QString &text){
            QString imagePath;
            QString description;
            int unitPrice = itemPriceMap.value(text, 0);

            if (text.contains("Motor 1")) {
                imagePath = ":/images/motor1.jpg";
                description = "Motor 1 - 120kg, 24V DC";
            } else if (text.contains("Motor 2")) {
                imagePath = ":/images/motor2.jpg";
                description = "Motor 2 - 300kg, 24V DC";
            } else if (text.contains("L2DB4830-CAFR")) {
                imagePath = ":/images/motordriver1.png";
                description = "Communication Uart+Can (115200bps,1Mbps)";
            } else if (text.contains("L2DB4830-CAFC")) {
                imagePath = ":/images/motordriver1.png";
                description = "Communication Uart+RS485 (115200bps)";
            } else if (text.contains("Battery 24V 20Ah")) {
                imagePath = ":/images/battery1.jpeg";
                description = "24V 20Ah LiPo4 Battery";
            } else if (text.contains("Battery 24V 40Ah")) {
                imagePath = ":/images/battery2.jpg";
                description = "24V 40Ah LiPo4 Battery";
            } else {
                description = "Select an item";
            }

            imageLabel->setPixmap(QPixmap(imagePath).scaled(200, 200, Qt::KeepAspectRatio));
            descLabel->setText(description);

            QTableWidgetItem *unitPriceItem = new QTableWidgetItem(QString::number(unitPrice));
            unitPriceItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 4, unitPriceItem);

            int quantity = spinBox->value();
            int total = quantity * unitPrice;
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total));
            totalItem->setTextAlignment(Qt::AlignCenter);
            ui->tableWidget->setItem(row, 5, totalItem);
            updateTotalCost();
        });
    }

    QTableWidgetItem *labelItem = new QTableWidgetItem("Total Cost:");
    QFont font = labelItem->font();
    font.setBold(true);
    labelItem->setFont(font);
    labelItem->setForeground(QBrush(Qt::blue));
    labelItem->setTextAlignment(Qt::AlignCenter);
    labelItem->setBackground(Qt::yellow);
    ui->tableWidget->setSpan(totalRow, 0, 1, 5);
    ui->tableWidget->setItem(totalRow, 0, labelItem);
    QTableWidgetItem *totalCostItem = new QTableWidgetItem("0");
    totalCostItem->setTextAlignment(Qt::AlignCenter);
    totalCostItem->setBackground(Qt::yellow);
    ui->tableWidget->setItem(totalRow, 5, totalCostItem);
}

MainWindow::~MainWindow()
{
    delete ui;
}
