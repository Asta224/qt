#ifndef SERVICE_H
#define SERVICE_H

#include <QDialog>
#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp> // ROS2 client library
#include <memory>
namespace Ui {
class service;
}

class service : public QDialog
{
    Q_OBJECT

public:
    explicit service(QMainWindow *mainwindow,QWidget *parent = nullptr);
    ~service();

private slots:
    void on_homeButton_clicked();
    void on_searchButton_clicked();

private:
    Ui::service *ui;
    QMainWindow *m_mainwindow;
    rclcpp::Node::SharedPtr node_; // ROS2 Node shared pointer
};

#endif // TOPIC_H
