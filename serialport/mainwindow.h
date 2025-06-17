#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include <QElapsedTimer>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_sendButton_clicked();
    void on_connectButton_clicked();
    void on_disconnectButton_clicked();
    void on_clearHistoryButton_clicked();
    void on_pausePlotCheckBox_stateChanged(int arg1);

    void generateRandomPlotData();
    void readSerialData();
    void setupPlot();
    void updatePlot();
private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QTimer *plotDataTimer;
    QElapsedTimer plotTimer;

    QVector<double> xData;
    QVector<double> yDataSine1;
    QVector<double> yDataSine2;

    void populateSerialPorts();
    void populateBaudRates();
};
#endif // MAINWINDOW_H
