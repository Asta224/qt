#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMap>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


// ItemData struct ကို ကြေညာခြင်း။
// ၎င်းသည် CSV file မှ ဖတ်ယူရရှိသော item တစ်ခုစီ၏ အချက်အလက်များကို သိမ်းဆည်းရန် အသုံးပြုသည်။
struct ItemData {
    QString imagePath; // item နှင့် သက်ဆိုင်သော ပုံ၏ path ကို သိမ်းရန် QString variable။
    double price; // item ၏ ဈေးနှုန်းကို သိမ်းရန် double variable။
    QString description; // item ၏ ဖော်ပြချက်ကို သိမ်းရန် QString variable။

    ItemData() {} // default constructor (အချက်အလက်မပါဘဲ ဖန်တီးနိုင်ရန်)။
    ItemData(const QString &img, double pr, const QString &desc) // parameter ပါသော constructor။
        : imagePath(img), price(pr), description(desc) {} // member များကို အစပျိုးခြင်း။
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;

     QMap<QString, ItemData> itemDataMap; // CSV မှ load လုပ်ထားသော item data များကို သိမ်းဆည်းရန် QMap ကို ကြေညာခြင်း။
                                         // Key က item name (QString) ဖြစ်ပြီး Value က ItemData struct ဖြစ်သည်။
    void loadPricesFromCSV(const QString &filepath); // CSV file မှ ဈေးနှုန်းများကို load လုပ်မည့် private function ကို ကြေညာခြင်း။
                                                     // file path ကို parameter အဖြစ် လက်ခံသည်။; 
};

#endif // MAINWINDOW_H
