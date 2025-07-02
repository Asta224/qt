#include "service.h"
#include "ui_service.h"

service::service(QMainWindow *mainwindow,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::service),
    m_mainwindow(mainwindow)
{
    ui->setupUi(this);
    if (!rclcpp::ok()){
        rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("service_node");
    this->setWindowState(Qt::WindowMaximized);
    connect(ui->searchButton, &QPushButton::clicked, this , &service::on_searchButton_clicked);
    connect(ui->homeButton, &QPushButton::clicked, this , &service::on_homeButton_clicked);
}

service::~service()
{   
    rclcpp::shutdown();
    delete ui;
}
void service::on_searchButton_clicked()
{
    ui->treeWidget->clear(); auto services = node_->get_service_names_and_types();

    for (const auto &service : services) {
        QString service_name = QString::fromStdString(service.first);
        QString types;
        for (const auto &type : service.second) {
            types += QString::fromStdString(type) + " ";
        }

        QTreeWidgetItem *parentItem = new QTreeWidgetItem(ui->treeWidget);
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsUserCheckable);
        parentItem->setCheckState(0, Qt::Unchecked);
        parentItem->setText(1, service_name);
        parentItem->setText(2, types.trimmed());

        //int hz = QRandomGenerator::global()->bounded(0, 21);
        parentItem->setText(3, "0");

        // 👉 Make Hz column editable
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsEditable);

        // 👉 Add a child item
        QTreeWidgetItem *childItem = new QTreeWidgetItem(parentItem);
        childItem->setText(1, "Editable Request");
        childItem->setFlags(childItem->flags() | Qt::ItemIsEditable);

        parentItem->setExpanded(false);
    }

}
void service::on_homeButton_clicked()
{
    this->close();
    m_mainwindow->show();

}

