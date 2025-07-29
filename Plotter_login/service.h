#ifndef SERVICE_H
#define SERVICE_H

#include <QDialog>
#include <QSerialPort> // serial port ဖြင့် ဆက်သွယ်ရန်အတွက် QSerialPort ကို ထည့်သွင်းခြင်း (Include QSerialPort for serial port communication)
#include <QSerialPortInfo> // serial port အချက်အလက်များကို ရယူရန်အတွက် QSerialPortInfo ကို ထည့်သွင်းခြင်း (Include QSerialPortInfo for getting serial port information)
#include <QVector> // ဒေတာများကို array ပုံစံ သိမ်းဆည်းရန်အတွက် QVector ကို ထည့်သွင်းခြင်း (Include QVector for storing data in an array-like structure)
#include <QElapsedTimer> // အချိန်တိုင်းတာရန်အတွက် QElapsedTimer ကို ထည့်သွင်းခြင်း (Include QElapsedTimer for measuring elapsed time)
#include <QTimer> // သတ်မှတ်ထားသော အချိန်တိုင်း လုပ်ဆောင်ရန်အတွက် QTimer ကို ထည့်သွင်းခြင်း (Include QTimer for performing actions at regular intervals)
#include <QMainWindow> // GUI အပလီကေးရှင်းများအတွက် အဓိက window class ဖြစ်သော QMainWindow ကို ထည့်သွင်းခြင်း (Include QMainWindow, the main window class for GUI applications)
#include <QEvent> // event များကို ကိုင်တွယ်ရန်အတွက် QEvent ကို ထည့်သွင်းခြင်း (Include QEvent for handling events)


namespace Ui {
class service;
}

class service : public QDialog
{
    Q_OBJECT

public:
    explicit service(QMainWindow *maindwidow, QWidget *parent = nullptr);
    ~service();
protected:
    // event များကို စစ်ထုတ်ရန်အတွက် override လုပ်ထားသော function (Overridden function for filtering events)
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots: // Qt ရဲ့ signal/slot mechanism အတွက် slot များ (Slots for Qt's signal/slot mechanism)
    void on_sendButton_clicked(); // sendButton ကို နှိပ်လိုက်သောအခါ လုပ်ဆောင်မည့် slot (Slot executed when sendButton is clicked)
    void on_clearHistoryButton_clicked(); // clearHistoryButton ကို နှိပ်လိုက်သောအခါ လုပ်ဆောင်မည့် slot (Slot executed when clearHistoryButton is clicked)
    // pausePlotCheckBox ၏ state ပြောင်းလဲသောအခါ လုပ်ဆောင်မည့် slot (Slot executed when the state of pausePlotCheckBox changes)
    void on_pausePlotCheckBox_stateChanged(int state);
    void on_homebtn_clicked(); // homebtn ကို နှိပ်လိုက်သောအခါ လုပ်ဆောင်မည့် slot (Slot executed when homebtn is clicked)

    void checkSerialData(); // serial port မှ ဒေတာများကို စစ်ဆေးသည့် slot (Slot to check data from serial port)

private:
    Ui::service *ui;
    QMainWindow *m_mainwindow;
    QSerialPort *serialPort; // serial port ဆက်သွယ်မှုအတွက် QSerialPort object (QSerialPort object for serial port communication)

    // CORRECTED: xData နှင့် yData တို့ကို plot line တစ်ခုတည်းအတွက် ကြေညာခြင်း (Declaration of xData and yData for a single plot line)
    QVector<double> xData, yData;
    QElapsedTimer plotTimer; // plot အတွက် အချိန်တိုင်းတာရန် timer (Timer for measuring time for plotting)
    QTimer *serialPollTimer; // serial port ဒေတာများကို ပုံမှန်စစ်ဆေးရန် timer (Timer for regularly polling serial port data)

    void sendData(const QString& message); // ဒေတာများ ပေးပို့သည့် private function (Private function to send data)
    void populateSerialPorts(); // serial port များကို စာရင်းပြုစုသည့် private function (Private function to populate serial ports)
    void populateBaudRates(); // baud rate များကို စာရင်းပြုစုသည့် private function (Private function to populate baud rates)
    void setupPlot(); // plot ကို စတင်သတ်မှတ်သည့် private function (Private function to set up the plot)
    void updatePlot(); // plot ကို update လုပ်သည့် private function (Private function to update the plot)
};

#endif // SERVICE_H