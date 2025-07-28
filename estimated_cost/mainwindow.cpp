
#include "ui_mainwindow.h"
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QMap>
#include <QPixmap>
#include <QHeaderView>
#include "mainwindow.h"

// QMap တစ်ခုဖြစ်ပြီး combo box ရဲ့ options တွေကို သိမ်းထားပါတယ်။
// Key က row index ဖြစ်ပြီး Value က QStringList (combo box ထဲမှာ ပြမယ့် item တွေ) ဖြစ်ပါတယ်။
QMap<int, QStringList> comboOptions = {
    {0, {"Motor", "Motor 1 (120kg)", "Motor 2 (300kg)"}},
    {1, {"Motor Driver","L2DB4830-CAFC"}},
    {2, {"IMU", "WT901 C-RS485", "WT901 C-232", "HWT901B"}},
    {3, {"LIDAR", "RPLIDAR A2", "LTME-02A","SLAMTEC RPLIDAR S1","HESAI XT16 XT32 Lidar","Velodyne VLS-128"}},
    {4, {"GPS", "WTRTK-980", "WTRTK-982", "WTRTK-982-LTE-G", "WTGAHRS2"}},
    {5, {"GPS Accessories", "RTK Base Station Antenna", "RTK Mobile Station Antenna"}},
    {6, {"Caster Wheel", "Wheel 1"}},
    {7, {"Camera", "Camera 1", "Camera 2", "Camera 3"}},
    {8, {"Motherboard","Motherboard 1"}},
    {9, {"Robot Controller","ROM Robot Controller"}},
    {10, {"Robot Desgin & Body parts","Po Po robor chassis","Kilo robot chassis"}},
    {11, {"Battery & Docking System","Battery 1","Docking System 1"}},
};
// ItemData struct ကို မူလ mainwindow.h မှာ ကြေညာထားပြီးဖြစ်လို့ ဒီမှာ ထပ်မကြေညာတော့ပါ။
// itemDataMap သည် CSV မှ load လုပ်ထားသော item များ၏ အချက်အလက် (imagePath, price, description) များကို သိမ်းဆည်းထားသည်။
QMap<QString, ItemData> itemDataMap;
// CSV file မှ item ဈေးနှုန်းများကို load လုပ်သည့် function။
void MainWindow::loadPricesFromCSV(const QString &filePath) {
    QFile file(filePath); // CSV file ကို ဖွင့်ရန် QFile object ဖန်တီးခြင်း။
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { // file ကို read-only text mode ဖြင့် ဖွင့်မရပါက။
        qDebug() << "Failed to open CSV file:" << filePath; // error message ထုတ်ခြင်း။
        return; // function ကနေ ပြန်ထွက်ခြင်း။
    }

    QTextStream in(&file); // file မှ text ကို ဖတ်ရန် QTextStream ဖန်တီးခြင်း။
    itemDataMap.clear(); // အရင် load လုပ်ထားသော data များကို ရှင်းလင်းခြင်း။
    int lineNumber = 0; // line နံပါတ်ကို မှတ်သားရန် variable။
    while (!in.atEnd()) { // file ရဲ့ အဆုံးကို မရောက်မချင်း loop ပတ်ခြင်း။
        QString line = in.readLine().trimmed(); // တစ်ကြောင်းချင်းစီ ဖတ်ပြီး အစအဆုံး whitespace တွေ ဖြတ်ခြင်း။
        lineNumber++; // line နံပါတ် တိုးခြင်း။
        if (line.isEmpty()) continue; // line ဗလာဖြစ်ပါက နောက် line ကို ဆက်သွားခြင်း။

        QStringList parts = line.split(','); // comma (,) နဲ့ ပိုင်းခြားပြီး parts တွေ ခွဲထုတ်ခြင်း။

        if (parts.size() >= 5) { // အနည်းဆုံး ၅ ခု ရှိမှသာ ဆက်လုပ်ခြင်း။
            QString itemName = parts[1].trimmed().remove('"'); // item name ကို ယူပြီး quotation mark တွေ ဖြုတ်ခြင်း။
            QString description = parts[2].trimmed().remove('"'); // description ကို ယူပြီး quotation mark တွေ ဖြုတ်ခြင်း။
            QString imagePath = parts[3].trimmed().remove('"'); // image path ကို ယူပြီး quotation mark တွေ ဖြုတ်ခြင်း။
            if (imagePath.startsWith(':')) imagePath = imagePath.mid(1); // image path က ':' နဲ့ စပါက ပထမဆုံး character ကို ဖြုတ်ခြင်း (resource path အတွက်)။

            QString priceStr = parts[4].trimmed().remove('$'); // ဈေးနှုန်း string ကို ယူပြီး '$' sign ဖြုတ်ခြင်း။
            bool ok; // toDouble() ရဲ့ success/failure ကို မှတ်ရန် boolean variable။
            double price = priceStr.toDouble(&ok); // ဈေးနှုန်း string ကို double အဖြစ် ပြောင်းခြင်း။
            if (ok && !itemName.isEmpty()) { // double ပြောင်းတာ အောင်မြင်ပြီး item name လည်း ဗလာမဟုတ်ပါက။
                itemDataMap[itemName] = {imagePath, price, description}; // itemDataMap ထဲသို့ data ထည့်ခြင်း။
                qDebug() << "Loaded CSV ->" << itemName << description << price; // debug message ထုတ်ခြင်း။
            } else {
                qDebug() << "Invalid price or item name at line" << lineNumber; // ဈေးနှုန်း သို့မဟုတ် item name မမှန်က error message ထုတ်ခြင်း။
            }
        } else {
            qDebug() << "Skipping invalid line:" << line; // parts အရေအတွက် မလုံလောက်ပါက warning message ထုတ်ခြင်း။
        }
    }
    file.close(); 
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
  // CSV file မှ ဈေးနှုန်းများကို load လုပ်ခြင်း။
    loadPricesFromCSV("/home/thukha/qt/estimated_cost/data/Robot Estimate Cost - Sheet1.csv");

    int itemRows = 11; // item row အရေအတွက် (0 ကနေ 10 အထိ)။
    int totalRow = itemRows; // စုစုပေါင်း ဈေးနှုန်းပြမယ့် row နံပါတ်။
    ui->tableWidget->setRowCount(itemRows + 1); // table widget ရဲ့ row အရေအတွက်ကို itemRows + 1 (total row အတွက်) သတ်မှတ်ခြင်း။
    ui->tableWidget->verticalHeader()->setVisible(false); // vertical header ကို ဖျောက်ထားခြင်း။
    ui->tableWidget->setColumnWidth(0, 10); // Column 0 (နံပါတ်စဉ်) ရဲ့ အကျယ်ကို 10 သတ်မှတ်ခြင်း။
    ui->tableWidget->setColumnWidth(1, 200); // Column 1 (Item Name) ရဲ့ အကျယ်ကို 200 သတ်မှတ်ခြင်း။
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch); // Column 2 (Description) ကို ကျန်တဲ့ နေရာ အကုန်ယူစေခြင်း။
    ui->tableWidget->setColumnWidth(3, 80); // Column 3 (Quantity) ရဲ့ အကျယ်ကို 80 သတ်မှတ်ခြင်း။
    ui->tableWidget->setColumnWidth(4, 200); // Column 4 (Unit Price) ရဲ့ အကျယ်ကို 200 သတ်မှတ်ခြင်း။
    ui->tableWidget->setColumnWidth(5, 200); // Column 5 (Total) ရဲ့ အကျယ်ကို 200 သတ်မှတ်ခြင်း။
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch); // Column 6 (နောက်ဆုံး column) ကို ကျန်တဲ့ နေရာ အကုန်ယူစေခြင်း။

    for (int row = 0; row <= itemRows; ++row) { // row တစ်ခုချင်းစီအတွက်။
        ui->tableWidget->setRowHeight(row, 150); // row အမြင့်ကို 150 သတ်မှတ်ခြင်း။
    }

    for (int row = 0; row < itemRows; ++row) { // item row တစ်ခုချင်းစီအတွက် loop ပတ်ခြင်း (total row မပါ)။
        ui->tableWidget->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1))); // နံပါတ်စဉ်ကို Column 0 မှာ ထည့်ခြင်း။

        QComboBox *combo = new QComboBox(this); // combo box အသစ် ဖန်တီးခြင်း။
        combo->addItems(comboOptions[row]); // comboOptions map ကနေ သက်ဆိုင်ရာ row အတွက် item တွေ ထည့်ခြင်း။
        ui->tableWidget->setCellWidget(row, 1, combo); // combo box ကို Column 1 မှာ ထည့်ခြင်း။

        QWidget *descWidget = new QWidget(); // description နဲ့ image အတွက် widget အသစ် ဖန်တီးခြင်း။
        QLabel *imageLabel = new QLabel(); // image ပြသဖို့ QLabel ဖန်တီးခြင်း။
        QLabel *descLabel = new QLabel("Select an item"); // description ပြသဖို့ QLabel ဖန်တီးခြင်း။
        QHBoxLayout *layout = new QHBoxLayout(descWidget); // imageLabel နဲ့ descLabel ကို ထည့်ဖို့ horizontal layout ဖန်တီးခြင်း။
        layout->addWidget(imageLabel); // imageLabel ကို layout ထဲ ထည့်ခြင်း။
        layout->addWidget(descLabel); // descLabel ကို layout ထဲ ထည့်ခြင်း။
        layout->setContentsMargins(0, 0, 0, 0); // layout ရဲ့ margins တွေကို 0 ပေးခြင်း။
        ui->tableWidget->setCellWidget(row, 2, descWidget); // descWidget ကို Column 2 မှာ ထည့်ခြင်း။

        QSpinBox *spinBox = new QSpinBox(this); // quantity အတွက် spin box အသစ် ဖန်တီးခြင်း။
        spinBox->setRange(0, 100); // spin box ရဲ့ range ကို 0 ကနေ 100 သတ်မှတ်ခြင်း။
        ui->tableWidget->setCellWidget(row, 3, spinBox); // spin box ကို Column 3 မှာ ထည့်ခြင်း။

        QTableWidgetItem *unitPriceItem = new QTableWidgetItem("0"); // unit price အတွက် QTableWidgetItem အသစ် ဖန်တီးပြီး 0 ပေးခြင်း။
        unitPriceItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
        ui->tableWidget->setItem(row, 4, unitPriceItem); // unitPriceItem ကို Column 4 မှာ ထည့်ခြင်း။

        QTableWidgetItem *totalItem = new QTableWidgetItem("0"); // total price အတွက် QTableWidgetItem အသစ် ဖန်တီးပြီး 0 ပေးခြင်း။
        totalItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
        ui->tableWidget->setItem(row, 5, totalItem); // totalItem ကို Column 5 မှာ ထည့်ခြင်း။

        // စုစုပေါင်း ကုန်ကျစရိတ်ကို update လုပ်သည့် lambda function။
        auto updateTotalCost = [=]() {
            double grandTotal = 0; // grand total ကို 0 အဖြစ် အစပျိုးခြင်း။
            for (int r = 0; r < itemRows; ++r) { // item row တစ်ခုချင်းစီအတွက် loop ပတ်ခြင်း။
                grandTotal += ui->tableWidget->item(r, 5)->text().toDouble(); // Column 5 မှ total price များကို ပေါင်းထည့်ခြင်း။
            }
            QTableWidgetItem *grandtotal = new QTableWidgetItem(QString::number(grandTotal, 'f', 2)); // grand total အတွက် QTableWidgetItem ဖန်တီးခြင်း။
            grandtotal->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
            grandtotal->setBackground(Qt::yellow); // နောက်ခံအရောင်ကို အဝါရောင် ပေးခြင်း။
            ui->tableWidget->setItem(totalRow, 5, grandtotal); // grand total ကို totalRow ရဲ့ Column 5 မှာ ထည့်ခြင်း။
        };

        // spinBox ရဲ့ value ပြောင်းလဲသောအခါ total price ကို update လုပ်ရန် connect လုပ်ခြင်း။
        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
            double unitPrice = ui->tableWidget->item(row, 4)->text().toDouble(); // Column 4 မှ unit price ကို ရယူခြင်း။
            double total = unitPrice * value; // quantity နဲ့ unit price ကို မြှောက်ပြီး total ကို တွက်ခြင်း။
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total, 'f', 2)); // total price အတွက် QTableWidgetItem ဖန်တီးခြင်း။
            totalItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
            ui->tableWidget->setItem(row, 5, totalItem); // totalItem ကို Column 5 မှာ ထည့်ခြင်း။
            updateTotalCost(); // grand total ကို update လုပ်ရန် ခေါ်ခြင်း။
        });

        // combo box ရဲ့ text ပြောင်းလဲသောအခါ item information များကို update လုပ်ရန် connect လုပ်ခြင်း။
        connect(combo, &QComboBox::currentTextChanged, this, [=](const QString &text){
            QString imagePath; // image path string ကို ကြေညာခြင်း။
            double unitPrice = 0; // unit price ကို 0 အဖြစ် အစပျိုးခြင်း။
            QString description = "Select an item"; // description ကို default အဖြစ် သတ်မှတ်ခြင်း။

            QString bestMatch; // အကောင်းဆုံး တူညီသော item name ကို သိမ်းရန်။
            int bestScore = 0; // အကောင်းဆုံး တူညီမှု score ကို သိမ်းရန်။
            for (const QString &key : itemDataMap.keys()) { // itemDataMap ထဲရှိ key (item name) တစ်ခုချင်းစီကို စစ်ဆေးခြင်း။
                int score = 0; // လက်ရှိ item အတွက် score ကို 0 အဖြစ် အစပျိုးခြင်း။
                for (const QString &word : text.split(" ", Qt::SkipEmptyParts)) { // combo box text ကို space နဲ့ ခွဲပြီး စကားလုံး တစ်ခုချင်းစီကို စစ်ဆေးခြင်း။
                    if (key.contains(word, Qt::CaseInsensitive)) score++; // item name ထဲမှာ စကားလုံး ပါဝင်ပါက score တိုးခြင်း။
                }
                if (score > bestScore) { // လက်ရှိ score က bestScore ထက် များပါက။
                    bestScore = score; // bestScore ကို update လုပ်ခြင်း။
                    bestMatch = key; // bestMatch ကို update လုပ်ခြင်း။
                }
            }

            if (!bestMatch.isEmpty()) { // bestMatch ရှိပါက။
                const ItemData &data = itemDataMap[bestMatch]; // itemDataMap မှ သက်ဆိုင်ရာ data ကို ရယူခြင်း။
                imagePath = data.imagePath; // image path ကို ရယူခြင်း။
                unitPrice = data.price; // unit price ကို ရယူခြင်း။
                description = data.description; // description ကို ရယူခြင်း။
            }

            imageLabel->clear(); // imageLabel ကို ရှင်းလင်းခြင်း။
            if (!imagePath.isEmpty()) { // imagePath ဗလာမဟုတ်ပါက။
                QPixmap pix; // QPixmap object ဖန်တီးခြင်း။
                if (!pix.load(":/"+imagePath)) { // resource path ကနေ image ကို load လုပ်ဖို့ ကြိုးစားခြင်း။
                    if (!pix.load(imagePath)) { // resource ကနေ load မရရင် direct path ကနေ load လုပ်ဖို့ ကြိုးစားခြင်း။
                        qDebug() << "Failed to load image:" << imagePath; // image load မရပါက error message ထုတ်ခြင်း။
                    }
                }
                if (!pix.isNull()) { // pixmap ဗလာမဟုတ်ပါက။
                    imageLabel->setPixmap(pix.scaled(200, 200, Qt::KeepAspectRatio)); // image ကို 200x200 အထိ scale ချဲ့ပြီး imageLabel မှာ ပြသခြင်း။
                }
            }

            descLabel->setText(description); // description ကို descLabel မှာ ပြသခြင်း။
            QTableWidgetItem *unitPriceItem = new QTableWidgetItem(QString::number(unitPrice, 'f', 2)); // unit price အတွက် QTableWidgetItem ဖန်တီးခြင်း။
            unitPriceItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
            ui->tableWidget->setItem(row, 4, unitPriceItem); // unitPriceItem ကို Column 4 မှာ ထည့်ခြင်း။

            int quantity = spinBox->value(); // spin box မှ quantity ကို ရယူခြင်း။
            double total = quantity * unitPrice; // total price ကို တွက်ခြင်း။
            QTableWidgetItem *totalItem = new QTableWidgetItem(QString::number(total, 'f', 2)); // total price အတွက် QTableWidgetItem ဖန်တီးခြင်း။
            totalItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
            ui->tableWidget->setItem(row, 5, totalItem); // totalItem ကို Column 5 မှာ ထည့်ခြင်း။
            updateTotalCost(); // grand total ကို update လုပ်ရန် ခေါ်ခြင်း။
        });
    }

    QTableWidgetItem *labelItem = new QTableWidgetItem("Total Cost:"); // "Total Cost:" label အတွက် QTableWidgetItem ဖန်တီးခြင်း။
    QFont font = labelItem->font(); // labelItem ရဲ့ font ကို ရယူခြင်း။
    font.setBold(true); // font ကို bold လုပ်ခြင်း။
    labelItem->setFont(font); // bold font ကို labelItem မှာ သတ်မှတ်ခြင်း။
    labelItem->setForeground(QBrush(Qt::blue)); // text အရောင်ကို အပြာရောင် ပေးခြင်း။
    labelItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
    labelItem->setBackground(Qt::yellow); // နောက်ခံအရောင်ကို အဝါရောင် ပေးခြင်း။
    ui->tableWidget->setSpan(totalRow, 0, 1, 5); // totalRow ရဲ့ Column 0 ကနေ Column 4 အထိ span လုပ်ခြင်း။
    ui->tableWidget->setItem(totalRow, 0, labelItem); // labelItem ကို totalRow ရဲ့ Column 0 မှာ ထည့်ခြင်း။
    QTableWidgetItem *totalCostItem = new QTableWidgetItem("0"); // စုစုပေါင်း ကုန်ကျစရိတ်အတွက် QTableWidgetItem ဖန်တီးပြီး 0 ပေးခြင်း။
    totalCostItem->setTextAlignment(Qt::AlignCenter); // text ကို အလယ်မှာ ထားခြင်း။
    totalCostItem->setBackground(Qt::yellow); // နောက်ခံအရောင်ကို အဝါရောင် ပေးခြင်း။
    ui->tableWidget->setItem(totalRow, 5, totalCostItem); // totalCostItem ကို totalRow ရဲ့ Column 5 မှာ ထည့်ခြင်း။
}


MainWindow::~MainWindow() {
    delete ui;
}
