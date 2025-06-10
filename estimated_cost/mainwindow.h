#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMap>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


struct ItemData {
    QString imagePath;
    double price;
    QString description;

    ItemData() {}
    ItemData(const QString &img, double pr, const QString &desc)
        : imagePath(img), price(pr), description(desc) {}
};



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;

    QMap<QString, ItemData> itemDataMap;
    void loadPricesFromCSV(const QString &filepath); 
};

#endif // MAINWINDOW_H
