#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QRandomGenerator>
#include <QMessageBox>
#include <QDebug>
#include <QtMath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)

{
    ui->setupUi(this);
    serialPort = new QSerialPort(this);
    populateSerialPorts();
    populateBaudRates();

    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    setupPlot();

    plotDataTimer = new QTimer(this);
    connect(plotDataTimer, &QTimer::timeout, this, &MainWindow::generateRandomPlotData);
    plotDataTimer->start(50);

    plotTimer.start();

    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::on_sendButton_clicked);
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::on_connectButton_clicked);
    connect(ui->disconnectButton, &QPushButton::clicked, this, &MainWindow::on_disconnectButton_clicked);
    connect(ui->clearHistoryButton, &QPushButton::clicked, this, &MainWindow::on_clearHistoryButton_clicked);

    connect(ui->pausePlotCheckBox, &QCheckBox::stateChanged, this, &MainWindow::on_pausePlotCheckBox_stateChanged);

    ui->disconnectButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    if (serialPort->isOpen())
    {
        serialPort->close();
    }
    delete ui;
}

void MainWindow::populateSerialPorts()
{
    ui->serialPortComboBox->clear();
    const auto serialPortsInfo = QSerialPortInfo::availablePorts();
    if (serialPortsInfo.isEmpty())
    {
        ui->serialPortComboBox->addItem("No Ports Found");
        ui->connectButton->setEnabled(false);
    }
    else
    {
        ui->connectButton->setEnabled(true);
        for (const QSerialPortInfo &PortInfo : serialPortsInfo)
        {
            ui->serialPortComboBox->addItem(PortInfo.portName());
            qDebug() << "Found serial port:" << PortInfo.portName()<< PortInfo.description();
        }
    }
}

void MainWindow::populateBaudRates()
{
    ui->baudRateComboBox->clear();
    ui->baudRateComboBox->addItem("9600");
    ui->baudRateComboBox->addItem("19200");
    ui->baudRateComboBox->addItem("38400");
    ui->baudRateComboBox->addItem("57600");
    ui->baudRateComboBox->addItem("115200");
    ui->baudRateComboBox->setCurrentText("115200");
}

void MainWindow::on_connectButton_clicked()
{
    if (serialPort->isOpen())
    {
        QMessageBox::warning(this, "Serial Port", "Port is already open!");
        return;
    }
    QString portName = ui->serialPortComboBox->currentText();
    qint32 baudRate = ui -> baudRateComboBox->currentText().toInt();

    if (portName == " No Ports Found" || portName.isEmpty())
    {
        QMessageBox::warning(this, "Serial Port", "Please select a valid serial port.");
        return;
    }
    serialPort->setPortName(portName);
    serialPort->setBaudRate(baudRate);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (serialPort->open(QIODevice::ReadWrite))
    {
        ui->receivedDataTextEdit->append(QString("--- Connected to %1 at %2 baud ---").arg(portName).arg(baudRate));
        ui->connectButton->setEnabled(false);
        ui->disconnectButton->setEnabled(true);
        ui->serialPortComboBox->setEnabled(false); // Disable port selection when connected
        ui->baudRateComboBox->setEnabled(false);   // Disable baud rate selection
    }
    else
    {
        QMessageBox::critical(this, "Serial Port Error", serialPort->errorString());
    }
}

void MainWindow::on_disconnectButton_clicked()
{
    if (serialPort->isOpen())
    {
        serialPort->close();
        ui->receivedDataTextEdit->append("--- Disconnected ---");
        ui->connectButton->setEnabled(true);
        ui->disconnectButton->setEnabled(false);
        ui->serialPortComboBox->setEnabled(true);
        ui->baudRateComboBox->setEnabled(true);
    }
}

void MainWindow::on_sendButton_clicked()
{
    QString message = ui->sendMessageLineEdit->text();
    if (message.isEmpty())
    {
        QMessageBox::warning(this, "Send Message", "Message field is empty.");
        return;
    }

    if (serialPort->isOpen() && serialPort->isWritable())
    {
        QByteArray data = message.toUtf8();
        serialPort->write(data);
        ui->receivedDataTextEdit->append("Sent: " + message);
        ui->sendMessageLineEdit->clear();
    }
    else
    {
        QMessageBox::warning(this, "Serial Port", "Port not open or not writable.");
    }
}

void MainWindow::readSerialData()
{
    // This function will be called when new data is available on the serial port.
    // For the first try, we'll just append it to the received data text edit.
    // Data parsing and plotting from serial will come later.
    QByteArray data = serialPort->readAll();
    QString receivedString = QString::fromUtf8(data);
    ui->receivedDataTextEdit->append("Received Raw: " + receivedString.trimmed());

    // --- IMPORTANT: For actual serial plotting, you'll need robust parsing here ---
    // Example: if your device sends "value1,value2\n"
    // QStringList parts = receivedString.trimmed().split(',');
    // if (parts.size() == 2) {
    //     bool ok1, ok2;
    //     double val1 = parts.at(0).toDouble(&ok1);
    //     double val2 = parts.at(1).toDouble(&ok2);
    //     if (ok1 && ok2) {
    //         double currentTime = plotTimer.elapsed() / 1000.0; // Time in seconds
    //         xData.append(currentTime);
    //         yDataSine1.append(val1);
    //         yDataSine2.append(val2);
    //         // Manage max data points
    //         const int maxDataPoints = 1000;
    //         if (xData.size() > maxDataPoints) {
    //             xData.removeFirst();
    //             yDataSine1.removeFirst();
    //             yDataSine2.removeFirst();
    //         }
    //         updatePlot(); // Replot with new serial data
    //     }
    // }
    // --- End of actual serial plotting parsing example ---
}

// --- UI Interaction Functions ---
void MainWindow::on_clearHistoryButton_clicked()
{
    ui->receivedDataTextEdit->clear();
}

void MainWindow::on_pausePlotCheckBox_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked)
    {
        plotDataTimer->stop();
        ui->receivedDataTextEdit->append("Plotting Paused.");
    }
    else
    {
        plotDataTimer->start(50); // Resume data generation/plotting
        ui->receivedDataTextEdit->append("Plotting Resumed.");
        // When resuming, clear current plot data to avoid jumps
        // Or adjust timeOffset to ensure continuous time
        // For simplicity here, we'll just continue from current elapsed time.
    }
}

// --- Plotting Functions ---
void MainWindow::setupPlot()
{
    // Add two graphs (curves)
    ui->customPlot->addGraph();                          // Graph 0 for Sine 1
    ui->customPlot->graph(0)->setPen(QPen(Qt::blue, 2)); // Blue line, 2 pixels wide
    ui->customPlot->graph(0)->setName("Sine 1");

    ui->customPlot->addGraph();                         // Graph 1 for Sine 2
    ui->customPlot->graph(1)->setPen(QPen(Qt::red, 2)); // Red line, 2 pixels wide
    ui->customPlot->graph(1)->setName("Sine 2");

    // Configure axes
    ui->customPlot->xAxis->setLabel("Time (s)");
    ui->customPlot->yAxis->setLabel("Value");
    ui->customPlot->yAxis->setRange(-1.5, 1.5); // Fixed Y range for sine waves

    // Make top and right axes visible, but without ticks and labels
    ui->customPlot->xAxis2->setVisible(true);
    ui->customPlot->xAxis2->setTicks(false);
    ui->customPlot->yAxis2->setVisible(true);
    ui->customPlot->yAxis2->setTicks(false);

    // Enable user interactions for zooming and dragging
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    // Add a legend to distinguish curves
    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setFont(QFont("Helvetica", 9));
    ui->customPlot->axisRect()->insetLayout()->addElement(ui->customPlot->legend, Qt::AlignBottom | Qt::AlignRight);
    ui->customPlot->legend->setBrush(QBrush(QColor(255, 255, 255, 150)));

    // Connect axis range changed signals to replot.
    // This is good practice if you allow user interaction on axes.
    connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
}

void MainWindow::generateRandomPlotData()
{
    // Generate two noisy sine waves for demonstration
    double currentTime = plotTimer.elapsed() / 1000.0;                                                         // Current time in seconds
    double y1 = qSin(currentTime * 2.0) + (QRandomGenerator::global()->generateDouble() - 0.5) * 0.2;          // Sine 1 with noise
    double y2 = qSin(currentTime * 2.0 + M_PI_2) + (QRandomGenerator::global()->generateDouble() - 0.5) * 0.2; // Sine 2 shifted with noise

    xData.append(currentTime);
    yDataSine1.append(y1);
    yDataSine2.append(y2);

    // Keep only the last N data points for a sliding window effect
    const int maxDataPoints = 1000; // Adjust this value as needed
    if (xData.size() > maxDataPoints)
    {
        xData.removeFirst();
        yDataSine1.removeFirst();
        yDataSine2.removeFirst();
    }

    updatePlot();

    // Optionally update text edit with random data for demonstration
    ui->receivedDataTextEdit->append(QString("Gen: T:%1, S1:%2, S2:%3")
                                         .arg(currentTime, 0, 'f', 2)
                                         .arg(y1, 0, 'f', 2)
                                         .arg(y2, 0, 'f', 2));

    // Auto-scroll the text edit
    QTextCursor cursor = ui->receivedDataTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->receivedDataTextEdit->setTextCursor(cursor);
}

void MainWindow::updatePlot()
{
    // Set data to the QCustomPlot graphs
    ui->customPlot->graph(0)->setData(xData, yDataSine1);
    ui->customPlot->graph(1)->setData(xData, yDataSine2);

    // Auto-rescale the x-axis to show the last N data points
    if (!xData.isEmpty())
    {
        // Option 1: Fixed range for current view (simple sliding window)
        ui->customPlot->xAxis->setRange(xData.last() - 5.0, xData.last()); // Show last 5 seconds of data

        // Option 2: Auto-scale X-axis (if you prefer it to expand)
        // ui->customPlot->xAxis->rescale();
    }

    // Replot the graph to display the new data
    ui->customPlot->replot();
}
