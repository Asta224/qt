#include "service.h"
#include "./ui_service.h"
#include <QRandomGenerator> // ကျပန်းနံပါတ်များ ထုတ်လုပ်ရန်အတွက် QRandomGenerator ကို ထည့်သွင်းခြင်း (Include QRandomGenerator for generating random numbers)
#include <QMessageBox> // သတိပေးချက်များ၊ အမှားအယွင်းများ ပြသရန်အတွက် QMessageBox ကို ထည့်သွင်းခြင်း (Include QMessageBox for displaying warnings, errors)
#include <QDebug> // Debugging messages များကို console တွင် ပြသရန်အတွက် QDebug class ကို ထည့်သွင်းခြင်း (Include QDebug class for printing debugging messages to the console)
#include <QtMath> // သင်္ချာဆိုင်ရာ function များအတွက် QtMath ကို ထည့်သွင်းခြင်း (Include QtMath for mathematical functions)
#include <QScrollBar> // scroll bar များကို ကိုင်တွယ်ရန်အတွက် QScrollBar ကို ထည့်သွင်းခြင်း (Include QScrollBar for handling scroll bars)
#include <QSerialPort> // serial port ဖြင့် ဆက်သွယ်ရန်အတွက် QSerialPort ကို ထည့်သွင်းခြင်း (Include QSerialPort for serial port communication)
#include <QSerialPortInfo> // serial port အချက်အလက်များကို ရယူရန်အတွက် QSerialPortInfo ကို ထည့်သွင်းခြင်း (Include QSerialPortInfo for getting serial port information)
#include <QTextCursor> // QTextEdit ၏ cursor ကို ကိုင်တွယ်ရန်အတွက် QTextCursor ကို ထည့်သွင်းခြင်း (Include QTextCursor for handling the cursor of a QTextEdit)
#include <QMouseEvent> // mouse event များကို ကိုင်တွယ်ရန်အတွက် QMouseEvent ကို ထည့်သွင်းခြင်း (Include QMouseEvent for handling mouse events)

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

service::service(QMainWindow *maindwidow, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::service),
    m_mainwindow(maindwidow)
{
    ui->setupUi(this);
    setWindowTitle("Serial Port Monitor");

    this->setStyleSheet(
        "QDialog {"
        "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1,"
        "                               stop:0 #E0FFFF,"
        "                               stop:1 #00CED1);"
        "}"
    );

   
    serialPort = new QSerialPort(this); // QSerialPort object အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new QSerialPort object)
    populateSerialPorts(); // serial port များကို စာရင်းပြုစုသော function ကို ခေါ်ဆိုခြင်း (Call the function to populate serial ports)
    populateBaudRates(); // baud rate များကို စာရင်းပြုစုသော function ကို ခေါ်ဆိုခြင်း (Call the function to populate baud rates)

    setupPlot(); // plot ကို စတင်သတ်မှတ်သော function ကို ခေါ်ဆိုခြင်း (Call the function to set up the plot)

    plotTimer.start(); // plotTimer ကို စတင်ခြင်း (Start plotTimer)

    // Signals နှင့် slots များကို ချိတ်ဆက်ခြင်း (Connect signals and slots)
    connect(ui->sendButton, &QPushButton::clicked, this, &service::on_sendButton_clicked); // sendButton နှိပ်သောအခါ on_sendButton_clicked ကို ခေါ်ဆိုခြင်း (Connect sendButton clicked to on_sendButton_clicked)
    connect(ui->clearHistoryButton, &QPushButton::clicked, this, &service::on_clearHistoryButton_clicked); // clearHistoryButton နှိပ်သောအခါ on_clearHistoryButton_clicked ကို ခေါ်ဆိုခြင်း (Connect clearHistoryButton clicked to on_clearHistoryButton_clicked)
    connect(ui->pausePlotCheckBox, &QCheckBox::stateChanged, this, &service::on_pausePlotCheckBox_stateChanged); // pausePlotCheckBox state ပြောင်းလဲသောအခါ on_pausePlotCheckBox_stateChanged ကို ခေါ်ဆိုခြင်း (Connect pausePlotCheckBox stateChanged to on_pausePlotCheckBox_stateChanged)
    connect(ui->homebtn, &QPushButton::clicked, this, &service::on_homebtn_clicked); // homebtn နှိပ်သောအခါ on_homebtn_clicked ကို ခေါ်ဆိုခြင်း (Connect homebtn clicked to on_homebtn_clicked)

    ui->connectButton->installEventFilter(this); // connectButton တွင် event filter ကို ထည့်သွင်းခြင်း (Install event filter on connectButton)

    serialPollTimer = new QTimer(this); // QTimer object အသစ်တစ်ခု ဖန်တီးခြင်း (Create a new QTimer object)
    connect(serialPollTimer, &QTimer::timeout, this, &service::checkSerialData); // serialPollTimer timeout ဖြစ်သောအခါ checkSerialData ကို ခေါ်ဆိုခြင်း (Connect serialPollTimer timeout to checkSerialData)
    serialPollTimer->start(10); // serialPollTimer ကို 10 milliseconds တိုင်း လုပ်ဆောင်ရန် စတင်ခြင်း (Start serialPollTimer to trigger every 10 milliseconds)

    ui->connectButton->setText("Connect"); // connectButton ၏ စာသားကို "Connect" အဖြစ် သတ်မှတ်ခြင်း (Set connectButton text to "Connect")
}

// service class ၏ Destructor ကို အကောင်အထည်ဖော်ခြင်း (Implement the destructor of the service class)
service::~service()
{
    if (serialPollTimer->isActive()) { // serialPollTimer အလုပ်လုပ်နေပါက (If serialPollTimer is active)
        serialPollTimer->stop(); // serialPollTimer ကို ရပ်တန့်ခြင်း (Stop serialPollTimer)
    }

    if (serialPort->isOpen()) // serialPort ပွင့်နေပါက (If serialPort is open)
    {
        serialPort->close(); // serialPort ကို ပိတ်ခြင်း (Close serialPort)
    }
    delete ui; // ui object အတွက် ခွဲဝေထားသော memory ကို ပြန်လည်ရှင်းလင်းခြင်း (Delete the ui object to free allocated memory)
}

// ဒေတာများ ပေးပို့သည့် private function ကို အကောင်အထည်ဖော်ခြင်း (Implement the private sendData function)
void service::sendData(const QString& message)
{
    if (message.isEmpty()) // message ဗလာဖြစ်နေပါက (If message is empty)
    {
        QMessageBox::warning(this, "Send Message", "Message field is empty."); // သတိပေးချက် ပြသခြင်း (Show a warning message)
        return; // function မှ ထွက်ခြင်း (Return from function)
    }

    if (serialPort->isOpen() && serialPort->isWritable()) // serialPort ပွင့်နေပြီး ရေးသားနိုင်ပါက (If serialPort is open and writable)
    {
        QByteArray data = message.toUtf8(); // message ကို UTF-8 QByteArray အဖြစ် ပြောင်းလဲခြင်း (Convert message to UTF-8 QByteArray)
        // data.append('\n'); // သင်၏ board က newline ကို မျှော်လင့်ပါက ဤ line ကို uncomment လုပ်ပါ (Uncomment if your board expects a newline after commands)

        qDebug() << "Sending:" << data; // ပေးပို့နေသော ဒေတာကို debug ပြုလုပ်ခြင်း (Debug the data being sent)

        qint64 bytesWritten = serialPort->write(data); // serial port သို့ ဒေတာများ ရေးသားခြင်း (Write data to serial port)
        if (bytesWritten == -1) // ရေးသားခြင်း မအောင်မြင်ပါက (If writing failed)
        {
            QMessageBox::critical(this, "Serial Port Write Error", serialPort->errorString()); // အမှားအယွင်း message ပြသခြင်း (Show an error message)
        }
        else if (bytesWritten != data.size()) // ရေးသားသော byte အရေအတွက် မတူညီပါက (If the number of bytes written is not equal to data size)
        {
             QMessageBox::warning(this, "Serial Port Write Warning", // သတိပေးချက် ပြသခြင်း (Show a warning message)
                                  "Only " + QString::number(bytesWritten) + " of " +
                                  QString::number(data.size()) + " bytes written. Port might be busy.");
        }
        else // အောင်မြင်စွာ ရေးသားနိုင်ပါက (If written successfully)
        {
            ui->receiveDataTextEdit->append("Sent: " + message); // receiveDataTextEdit တွင် "Sent: " ဖြင့် message ကို ထည့်သွင်းခြင်း (Append "Sent: " with message to receiveDataTextEdit)
            ui->sendMessageLineEdit->clear(); // sendMessageLineEdit ကို ရှင်းလင်းခြင်း (Clear sendMessageLineEdit)
        }
    }
    else // serialPort မပွင့်ပါက သို့မဟုတ် ရေးသားနိုင်ခြင်း မရှိပါက (If serialPort is not open or not writable)
    {
        QMessageBox::warning(this, "Serial Port", "Port not open or not writable. Cannot send data."); // သတိပေးချက် ပြသခြင်း (Show a warning message)
    }
}

// event များကို စစ်ထုတ်ရန်အတွက် override လုပ်ထားသော function ကို အကောင်အထည်ဖော်ခြင်း (Implement the overridden eventFilter function)
bool service::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->connectButton) // event ဖြစ်ပေါ်သော object သည် connectButton ဖြစ်ပါက (If the object on which the event occurred is connectButton)
    {
        if (event->type() == QEvent::MouseButtonPress) // event အမျိုးအစားသည် MouseButtonPress ဖြစ်ပါက (If the event type is MouseButtonPress)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event); // event ကို QMouseEvent အဖြစ် ပြောင်းလဲခြင်း (Cast event to QMouseEvent)
            if (mouseEvent->button() == Qt::LeftButton) // နှိပ်လိုက်သော mouse button သည် ဘယ်ဘက် button ဖြစ်ပါက (If the clicked mouse button is the left button)
            {
                qDebug() << "Connect button clicked via Event Filter (Mouse Press)"; // Debug message ပြသခြင်း (Display debug message)
                ui->connectButton->setEnabled(false); // connectButton ကို disable လုပ်ခြင်း (Disable connectButton)

                QString portName = ui->serialPortComboBox->currentText(); // serialPortComboBox မှ လက်ရှိရွေးချယ်ထားသော port name ကို ရယူခြင်း (Get the current selected port name from serialPortComboBox)
                qint32 baudRate = ui->baudRateComboBox->currentText().toInt(); // baudRateComboBox မှ လက်ရှိရွေးချယ်ထားသော baud rate ကို integer အဖြစ် ပြောင်းလဲခြင်း (Convert the current selected baud rate from baudRateComboBox to an integer)
                qDebug() << "Selected Port:" << portName << "Baud Rate:" << baudRate; // ရွေးချယ်ထားသော port နှင့် baud rate ကို debug ပြုလုပ်ခြင်း (Debug the selected port and baud rate)

                if (serialPort->isOpen()) // serialPort ပွင့်နေပါက (If serialPort is open)
                {
                    qDebug() << "Attempting to disconnect from" << serialPort->portName(); // ချိတ်ဆက်မှု ဖြုတ်ရန် ကြိုးစားနေကြောင်း debug ပြုလုပ်ခြင်း (Debug that an attempt to disconnect is being made)
                    serialPort->close(); // serialPort ကို ပိတ်ခြင်း (Close serialPort)
                    ui->receiveDataTextEdit->append("--- Disconnected ---"); // receiveDataTextEdit တွင် "--- Disconnected ---" ကို ထည့်သွင်းခြင်း (Append "--- Disconnected ---" to receiveDataTextEdit)
                    ui->connectButton->setText("Connect"); // connectButton ၏ စာသားကို "Connect" အဖြစ် ပြောင်းလဲခြင်း (Change connectButton text to "Connect")
                    ui->serialPortComboBox->setEnabled(true); // serialPortComboBox ကို enable လုပ်ခြင်း (Enable serialPortComboBox)
                    ui->baudRateComboBox->setEnabled(true); // baudRateComboBox ကို enable လုပ်ခြင်း (Enable baudRateComboBox)

                    qDebug() << "Disconnected successfully. Button text set to 'Connect'."; // ချိတ်ဆက်မှု ဖြုတ်ခြင်း အောင်မြင်ကြောင်း debug ပြုလုပ်ခြင်း (Debug that disconnection was successful)
                }
                else // serialPort မပွင့်ပါက (If serialPort is not open)
                {
                    if (portName == "No Ports Found" || portName.isEmpty()) // portName သည် "No Ports Found" ဖြစ်ပါက သို့မဟုတ် ဗလာဖြစ်ပါက (If portName is "No Ports Found" or empty)
                    {
                        qDebug() << "Error: No valid serial port selected or found in combo box."; // အမှားအယွင်း debug message ပြသခြင်း (Display error debug message)
                        QMessageBox::warning(this, "Serial Port", "Please select a valid serial port."); // သတိပေးချက် ပြသခြင်း (Show a warning message)
                        ui->connectButton->setEnabled(true); // connectButton ကို enable လုပ်ခြင်း (Enable connectButton)
                        return true; // event ကို ကိုင်တွယ်ပြီးဖြစ်ကြောင်း ပြန်ပေးခြင်း (Return true, indicating the event has been handled)
                    }

                    serialPort->setPortName(portName); // serialPort ၏ port name ကို သတ်မှတ်ခြင်း (Set the port name for serialPort)
                    serialPort->setBaudRate(baudRate); // serialPort ၏ baud rate ကို သတ်မှတ်ခြင်း (Set the baud rate for serialPort)
                    serialPort->setDataBits(QSerialPort::Data8); // data bits ကို Data8 အဖြစ် သတ်မှတ်ခြင်း (Set data bits to Data8)
                    serialPort->setParity(QSerialPort::NoParity); // parity ကို NoParity အဖြစ် သတ်မှတ်ခြင်း (Set parity to NoParity)
                    serialPort->setStopBits(QSerialPort::OneStop); // stop bits ကို OneStop အဖြစ် သတ်မှတ်ခြင်း (Set stop bits to OneStop)
                    serialPort->setFlowControl(QSerialPort::NoFlowControl); // flow control ကို NoFlowControl အဖြစ် သတ်မှတ်ခြင်း (Set flow control to NoFlowControl)
                    qDebug() << "Attempting to open port:" << portName << "with baud:" << baudRate; // port ဖွင့်ရန် ကြိုးစားနေကြောင်း debug ပြုလုပ်ခြင်း (Debug that an attempt to open the port is being made)

                    if (serialPort->open(QIODevice::ReadWrite)) // serialPort ကို ReadWrite mode ဖြင့် ဖွင့်နိုင်ပါက (If serialPort can be opened in ReadWrite mode)
                    {
                        qDebug() << "Port opened successfully."; // Port ဖွင့်ခြင်း အောင်မြင်ကြောင်း debug ပြုလုပ်ခြင်း (Debug that port opened successfully)
                        ui->receiveDataTextEdit->append(QString("--- Connected to %1 at %2 baud ---").arg(portName).arg(baudRate)); // receiveDataTextEdit တွင် ချိတ်ဆက်မှု အချက်အလက် ထည့်သွင်းခြင်း (Append connection info to receiveDataTextEdit)
                        ui->connectButton->setText("Disconnect"); // connectButton ၏ စာသားကို "Disconnect" အဖြစ် ပြောင်းလဲခြင်း (Change connectButton text to "Disconnect")
                        ui->connectButton->setEnabled(true); // connectButton ကို enable လုပ်ခြင်း (Enable connectButton)
                        ui->serialPortComboBox->setEnabled(false); // serialPortComboBox ကို disable လုပ်ခြင်း (Disable serialPortComboBox)
                        ui->baudRateComboBox->setEnabled(false); // baudRateComboBox ကို disable လုပ်ခြင်း (Disable baudRateComboBox)

                        ui->receiveDataTextEdit->append("Plot will now show real-time serial data (single line)."); // receiveDataTextEdit တွင် plot အချက်အလက် ထည့်သွင်းခြင်း (Append plot info to receiveDataTextEdit)
                        qDebug() << "Connected successfully. Button text set to 'Disconnect'."; // ချိတ်ဆက်မှု အောင်မြင်ကြောင်း debug ပြုလုပ်ခြင်း (Debug that connection was successful)
                    }
                    else // serialPort ကို ဖွင့်မရပါက (If serialPort cannot be opened)
                    {
                        QString errorString = serialPort->errorString(); // serial port အမှားအယွင်း message ကို ရယူခြင်း (Get serial port error message)
                        qDebug() << "Failed to open port. Error:" << errorString << "Serial Port Error Code:" << serialPort->error(); // Port ဖွင့်ခြင်း မအောင်မြင်ကြောင်း debug ပြုလုပ်ခြင်း (Debug that port opening failed)
                        QMessageBox::critical(this, "Serial Port Error", errorString); // အမှားအယွင်း message ပြသခြင်း (Show an error message)
                        ui->connectButton->setEnabled(true); // connectButton ကို enable လုပ်ခြင်း (Enable connectButton)
                        qDebug() << "Connection failed. Button text remains 'Connect'."; // ချိတ်ဆက်မှု မအောင်မြင်ကြောင်း debug ပြုလုပ်ခြင်း (Debug that connection failed)
                    }
                }
                return true; // event ကို ကိုင်တွယ်ပြီးဖြစ်ကြောင်း ပြန်ပေးခြင်း (Return true, indicating the event has been handled)
            }
        }
    }
    return QDialog::eventFilter(obj, event); // parent class ၏ eventFilter ကို ခေါ်ဆိုခြင်း (Call the eventFilter of the parent class)
}


// --- Serial Data Polling ၏ ပြုပြင်ထားသော အကောင်အထည်ဖော်မှု (တစ်ကြောင်းလျှင် တန်ဖိုးတစ်ခုကို မျှော်လင့်သည်) ---
void service::checkSerialData()
{
    if (serialPort->isOpen() && serialPort->bytesAvailable() > 0) // serialPort ပွင့်နေပြီး bytes များ ရရှိနိုင်ပါက (If serialPort is open and bytes are available)
    {
        QByteArray data = serialPort->readAll(); // serial port မှ ဒေတာအားလုံးကို ဖတ်ခြင်း (Read all data from serial port)
        QString receivedString = QString::fromUtf8(data); // ဒေတာကို UTF-8 string အဖြစ် ပြောင်းလဲခြင်း (Convert data to UTF-8 string)

        QString trimmedString = receivedString.trimmed(); // string ၏ အစနှင့် အဆုံးရှိ နေရာလွတ်များကို ဖယ်ရှားခြင်း (Remove leading and trailing whitespace from the string)
        ui->receiveDataTextEdit->append("Received Raw (Polled): " + trimmedString); // receiveDataTextEdit တွင် လက်ခံရရှိသော raw data ကို ထည့်သွင်းခြင်း (Append received raw data to receiveDataTextEdit)
        qDebug() << "Full Received String (trimmed):" << trimmedString; // လက်ခံရရှိသော string ကို debug ပြုလုပ်ခြင်း (Debug the received string)

        QTextCursor cursor = ui->receiveDataTextEdit->textCursor(); // receiveDataTextEdit ၏ cursor ကို ရယူခြင်း (Get the cursor of receiveDataTextEdit)
        cursor.movePosition(QTextCursor::End); // cursor ကို အဆုံးသို့ ရွှေ့ခြင်း (Move cursor to the end)
        ui->receiveDataTextEdit->setTextCursor(cursor); // receiveDataTextEdit ၏ cursor ကို သတ်မှတ်ခြင်း (Set the cursor of receiveDataTextEdit)

        if (!ui->pausePlotCheckBox->isChecked()) // pausePlotCheckBox ကို check မလုပ်ထားပါက (If pausePlotCheckBox is not checked)
        {
            // အရေးကြီးသည်: ယခုအခါ parsing သည် တစ်ကြောင်းလျှင် ဂဏန်းတန်ဖိုးတစ်ခုကို မျှော်လင့်သည် (IMPORTANT: Now parsing expects a single numerical value per line.)
            bool ok; // ပြောင်းလဲခြင်း အောင်မြင်ခြင်း ရှိမရှိ စစ်ဆေးရန် boolean (Boolean to check if conversion was successful)
            double val = trimmedString.toDouble(&ok); // trimmed string တစ်ခုလုံးကို double အဖြစ် ပြောင်းလဲရန် ကြိုးစားခြင်း (Try to convert the whole trimmed string to a double)

            qDebug() << "Parsed Value:" << val << "Conversion OK:" << ok; // parse လုပ်ထားသော တန်ဖိုးနှင့် ပြောင်းလဲခြင်း အခြေအနေကို debug ပြုလုပ်ခြင်း (Debug the parsed value and conversion status)

            if (ok) { // ပြောင်းလဲခြင်း အောင်မြင်ပါက (If conversion was successful)
                double currentTime = plotTimer.elapsed() / 1000.0; // လက်ရှိအချိန်ကို စက္ကန့်ဖြင့် ရယူခြင်း (Get current time in seconds)
                xData.append(currentTime); // xData တွင် လက်ရှိအချိန်ကို ထည့်သွင်းခြင်း (Append current time to xData)
                yData.append(val); // yData တွင် တန်ဖိုးကို ထည့်သွင်းခြင်း (Append value to yData)

                const int maxDataPoints = 1000; // အများဆုံး ဒေတာမှတ်အရေအတွက် (Maximum number of data points)
                if (xData.size() > maxDataPoints) { // xData ၏ အရွယ်အစားသည် maxDataPoints ထက် ပိုပါက (If the size of xData is greater than maxDataPoints)
                    xData.removeFirst(); // xData မှ ပထမဆုံး အချက်အလက်ကို ဖယ်ရှားခြင်း (Remove the first data point from xData)
                    yData.removeFirst(); // yData မှ ပထမဆုံး အချက်အလက်ကို ဖယ်ရှားခြင်း (Remove the first data point from yData)
                }
                updatePlot(); // plot ကို update လုပ်ခြင်း (Update the plot)
            } else {
                qDebug() << "Failed to parse single numerical value from: " << trimmedString; // ဂဏန်းတန်ဖိုးကို parse လုပ်ခြင်း မအောင်မြင်ကြောင်း debug ပြုလုပ်ခြင်း (Debug that parsing of numerical value failed)
            }
        }
    }
}

// serial port များကို စာရင်းပြုစုသည့် private function ကို အကောင်အထည်ဖော်ခြင်း (Implement the private populateSerialPorts function)
void service::populateSerialPorts()
{
    ui->serialPortComboBox->clear(); // serialPortComboBox ကို ရှင်းလင်းခြင်း (Clear serialPortComboBox)
    const auto serialPortsInfo = QSerialPortInfo::availablePorts(); // ရရှိနိုင်သော serial port အချက်အလက်များကို ရယူခြင်း (Get available serial port information)
    bool foundDesiredPorts = false; // လိုချင်သော port များ တွေ့ရှိခြင်း ရှိမရှိ စစ်ဆေးရန် boolean (Boolean to check if desired ports are found)
    qDebug() << "Populating serial ports..."; // Debug message ပြသခြင်း (Display debug message)
    if (serialPortsInfo.isEmpty()) { // serialPortsInfo ဗလာဖြစ်နေပါက (If serialPortsInfo is empty)
        qDebug() << "No serial ports found by QSerialPortInfo::availablePorts(). Check permissions or hardware."; // Debug message ပြသခြင်း (Display debug message)
    }

    for (const QSerialPortInfo &PortInfo : serialPortsInfo) // serialPortsInfo တစ်ခုချင်းစီကို စစ်ဆေးခြင်း (Iterate through each serial port info)
    {
        QString portName = PortInfo.portName(); // port name ကို ရယူခြင်း (Get port name)
        qDebug() << "Found port:" << portName << "Description:" << PortInfo.description() << "Manufacturer:" << PortInfo.manufacturer(); // တွေ့ရှိသော port အချက်အလက်များကို debug ပြုလုပ်ခြင်း (Debug found port information)
        if (portName.startsWith("ttyUSB") || portName.startsWith("ttyACM") || portName.startsWith("COM")) // port name သည် "ttyUSB"၊ "ttyACM" သို့မဟုတ် "COM" ဖြင့် စတင်ပါက (If port name starts with "ttyUSB", "ttyACM", or "COM")
        {
            ui->serialPortComboBox->addItem(portName); // serialPortComboBox တွင် port name ကို ထည့်သွင်းခြင်း (Add port name to serialPortComboBox)
            foundDesiredPorts = true; // foundDesiredPorts ကို true အဖြစ် သတ်မှတ်ခြင်း (Set foundDesiredPorts to true)
            qDebug() << "Added relevant port:" << portName; // Debug message ပြသခြင်း (Display debug message)
        }
    }
    if (!foundDesiredPorts) // လိုချင်သော port များ မတွေ့ရှိပါက (If desired ports are not found)
    {
        ui->serialPortComboBox->addItem("No Ports Found"); // serialPortComboBox တွင် "No Ports Found" ကို ထည့်သွင်းခြင်း (Add "No Ports Found" to serialPortComboBox)
        ui->connectButton->setEnabled(false); // connectButton ကို disable လုပ်ခြင်း (Disable connectButton)
        qDebug() << "No relevant ports found. Connect button disabled."; // Debug message ပြသခြင်း (Display debug message)
    }
    else // လိုချင်သော port များ တွေ့ရှိပါက (If desired ports are found)
    {
        if (!serialPort->isOpen()) { // serialPort မပွင့်သေးပါက (If serialPort is not yet open)
            ui->connectButton->setEnabled(true); // connectButton ကို enable လုပ်ခြင်း (Enable connectButton)
            qDebug() << "Relevant ports found. Connect button enabled."; // Debug message ပြသခြင်း (Display debug message)
        } else {
            qDebug() << "Relevant ports found, but serial port is already open. Connect button remains enabled."; // Debug message ပြသခြင်း (Display debug message)
        }
    }
}

// baud rate များကို စာရင်းပြုစုသည့် private function ကို အကောင်အထည်ဖော်ခြင်း (Implement the private populateBaudRates function)
void service::populateBaudRates()
{
    ui->baudRateComboBox->clear(); // baudRateComboBox ကို ရှင်းလင်းခြင်း (Clear baudRateComboBox)
    ui->baudRateComboBox->addItem("9600"); // baudRateComboBox တွင် "9600" ကို ထည့်သွင်းခြင်း (Add "9600" to baudRateComboBox)
    ui->baudRateComboBox->addItem("19200"); // baudRateComboBox တွင် "19200" ကို ထည့်သွင်းခြင်း (Add "19200" to baudRateComboBox)
    ui->baudRateComboBox->addItem("38400"); // baudRateComboBox တွင် "38400" ကို ထည့်သွင်းခြင်း (Add "38400" to baudRateComboBox)
    ui->baudRateComboBox->addItem("57600"); // baudRateComboBox တွင် "57600" ကို ထည့်သွင်းခြင်း (Add "57600" to baudRateComboBox)
    ui->baudRateComboBox->addItem("115200"); // baudRateComboBox တွင် "115200" ကို ထည့်သွင်းခြင်း (Add "115200" to baudRateComboBox)
    ui->baudRateComboBox->setCurrentText("9600"); // default အဖြစ် "9600" ကို သတ်မှတ်ခြင်း (Set default to "9600")
}

// sendButton ကို နှိပ်လိုက်သောအခါ လုပ်ဆောင်မည့် slot ကို အကောင်အထည်ဖော်ခြင်း (Implement the slot executed when sendButton is clicked)
void service::on_sendButton_clicked()
{
    ui->sendButton->setEnabled(false); // sendButton ကို disable လုပ်ခြင်း (Disable sendButton)
    sendData(ui->sendMessageLineEdit->text()); // sendMessageLineEdit မှ စာသားကို sendData function ဖြင့် ပေးပို့ခြင်း (Send text from sendMessageLineEdit using sendData function)
    ui->sendButton->setEnabled(true); // sendButton ကို enable လုပ်ခြင်း (Enable sendButton)
}

// clearHistoryButton ကို နှိပ်လိုက်သောအခါ လုပ်ဆောင်မည့် slot ကို အကောင်အထည်ဖော်ခြင်း (Implement the slot executed when clearHistoryButton is clicked)
void service::on_clearHistoryButton_clicked()
{
    ui->receiveDataTextEdit->clear(); // receiveDataTextEdit ကို ရှင်းလင်းခြင်း (Clear receiveDataTextEdit)
    xData.clear(); // xData ကို ရှင်းလင်းခြင်း (Clear xData)
    yData.clear(); // yData ကို ရှင်းလင်းခြင်း (Clear yData)
    updatePlot(); // plot ကို update လုပ်ခြင်း (Update the plot)
    plotTimer.restart(); // plotTimer ကို ပြန်လည်စတင်ခြင်း (Restart plotTimer)
}

// pausePlotCheckBox ၏ state ပြောင်းလဲသောအခါ လုပ်ဆောင်မည့် slot ကို အကောင်အထည်ဖော်ခြင်း (Implement the slot executed when the state of pausePlotCheckBox changes)
void service::on_pausePlotCheckBox_stateChanged(int state)
{
    if (state == Qt::Checked) // state သည် Qt::Checked ဖြစ်ပါက (If state is Qt::Checked)
    {
        ui->receiveDataTextEdit->append("Plotting Paused. Incoming serial data will not be plotted."); // receiveDataTextEdit တွင် message ထည့်သွင်းခြင်း (Append message to receiveDataTextEdit)
    }
    else // state သည် Qt::Unchecked ဖြစ်ပါက (If state is Qt::Unchecked)
    {
        ui->receiveDataTextEdit->append("Plotting Resumed. Incoming serial data will be plotted."); // receiveDataTextEdit တွင် message ထည့်သွင်းခြင်း (Append message to receiveDataTextEdit)
    }
}

// --- plot တစ်ခုတည်းအတွက် setupPlot() ၏ ပြုပြင်ထားသော အကောင်အထည်ဖော်မှု ---
void service::setupPlot()
{
    // Graph 0 (single line) ကို စတင်သတ်မှတ်ခြင်း (Configure Graph 0 (the single line))
    ui->customPlot->addGraph(); // customPlot တွင် graph အသစ်တစ်ခု ထည့်သွင်းခြင်း (Add a new graph to customPlot)
    ui->customPlot->graph(0)->setPen(QPen(Qt::blue, 2)); // graph 0 ၏ pen အရောင်နှင့် အထူကို သတ်မှတ်ခြင်း (Set pen color and width for graph 0)
    ui->customPlot->graph(0)->setName("Serial Data"); // graph 0 ၏ နာမည်ကို သတ်မှတ်ခြင်း (Set name for graph 0)

    ui->customPlot->xAxis->setLabel("Time (s)"); // x-axis ၏ label ကို သတ်မှတ်ခြင်း (Set label for x-axis)
    ui->customPlot->yAxis->setLabel("Value"); // y-axis ၏ label ကို သတ်မှတ်ခြင်း (Set label for y-axis)
    // Y-axis ၏ ကနဦး range ကို သတ်မှတ်ခြင်း။ သင်၏ board မှ ပေးပို့သော တန်ဖိုးများပေါ် မူတည်၍ ချိန်ညှိပါ။ (Initial Y-axis range. Adjust based on the values your board sends.)
    // သင်၏ တန်ဖိုးများသည် 0-1023 (analogRead ကဲ့သို့) ဖြစ်ပါက ဥပမာ- 0, 1024 သို့ ပြောင်းလဲပါ။ (If your values are 0-1023 (like analogRead), change this to e.g., 0, 1024.)
    ui->customPlot->yAxis->setRange(0, 1024); // ဥပမာ- 0-1023 sensor readings အတွက် (Example: for 0-1023 sensor readings)

    ui->customPlot->xAxis2->setVisible(true); // x-axis2 ကို မြင်နိုင်စေရန် သတ်မှတ်ခြင်း (Set x-axis2 to be visible)
    ui->customPlot->xAxis2->setTicks(false); // x-axis2 ၏ ticks များကို ဖျောက်ထားခြင်း (Hide ticks for x-axis2)
    ui->customPlot->yAxis2->setVisible(true); // y-axis2 ကို မြင်နိုင်စေရန် သတ်မှတ်ခြင်း (Set y-axis2 to be visible)
    ui->customPlot->yAxis2->setTicks(false); // y-axis2 ၏ ticks များကို ဖျောက်ထားခြင်း (Hide ticks for y-axis2)

    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables); // plot ၏ interaction များကို သတ်မှတ်ခြင်း (Set interactions for the plot)

    ui->customPlot->legend->setVisible(true); // legend ကို မြင်နိုင်စေရန် သတ်မှတ်ခြင်း (Set legend to be visible)
    ui->customPlot->legend->setFont(QFont("Helvetica", 9)); // legend ၏ font ကို သတ်မှတ်ခြင်း (Set font for legend)
    ui->customPlot->axisRect()->insetLayout()->addElement(ui->customPlot->legend, Qt::AlignBottom | Qt::AlignRight); // legend ကို axisRect ၏ ညာဘက်အောက်ခြေတွင် ထားရှိခြင်း (Place legend at the bottom right of axisRect)
    ui->customPlot->legend->setBrush(QBrush(QColor(255, 255, 255, 150))); // legend ၏ နောက်ခံအရောင်ကို သတ်မှတ်ခြင်း (Set background brush for legend)

    // x-axis ၏ range ပြောင်းလဲသောအခါ x-axis2 ၏ range ကို ပြောင်းလဲရန် ချိတ်ဆက်ခြင်း (Connect x-axis rangeChanged to x-axis2 setRange)
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    // y-axis ၏ range ပြောင်းလဲသောအခါ y-axis2 ၏ range ကို ပြောင်းလဲရန် ချိတ်ဆက်ခြင်း (Connect y-axis rangeChanged to y-axis2 setRange)
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
}

// --- plot တစ်ခုတည်းအတွက် updatePlot() ၏ ပြုပြင်ထားသော အကောင်အထည်ဖော်မှု ---
void service::updatePlot()
{
    ui->customPlot->graph(0)->setData(xData, yData); // graph 0 အတွက် ဒေတာကို သတ်မှတ်ခြင်း (Set data for graph 0)

    if (!xData.isEmpty()) // xData ဗလာမဖြစ်ပါက (If xData is not empty)
    {
        ui->customPlot->xAxis->setRange(xData.last() - 5.0, xData.last()); // x-axis ၏ range ကို သတ်မှတ်ခြင်း (Set x-axis range)
        // ရွေးချယ်နိုင်သည်: သင်၏ တန်ဖိုးများ အပြောင်းအလဲများပြီး တိကျသော (ဥပမာ- 0-1024) မဟုတ်ပါက Y-axis ကို အလိုအလျောက် ပြန်လည်ချိန်ညှိပါ။ (Optional: Auto-rescale Y-axis if your values vary widely and are not fixed (e.g., 0-1024))
        // ui->customPlot->graph(0)->rescaleValueAxis(true, true);
    } else {
        ui->customPlot->xAxis->setRange(0, 5); // xData ဗလာဖြစ်ပါက x-axis ၏ range ကို သတ်မှတ်ခြင်း (Set x-axis range if xData is empty)
    }

    ui->customPlot->replot(); // plot ကို ပြန်လည်ဆွဲခြင်း (Replot the graph)
}


void service::on_homebtn_clicked()
{
    this->close();
    if (m_mainwindow) {
        m_mainwindow->show();
    }
}