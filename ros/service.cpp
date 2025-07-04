#include "service.h"
#include "ui_service.h"

service::service(QMainWindow *mainwindow,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::service),
    m_mainwindow(mainwindow)
{
    ui->setupUi(this);
    this->setWindowState(Qt::WindowMaximized);
    this->setStyleSheet(
        // QDialog background color (similar to centralwidget in MainWindow)
        "QDialog { background-color: #E0E0E0; }"

        // QLabel text color
        "QLabel { color: #333333; }"

        // QPushButton styling
        "QPushButton { "
        "color: white; "
        "background-color: #4CAF50; "
        "border: 1px solid #388E3C; "
        "border-radius: 3px; "
        "padding: 5px 10px; "
        "}"
        "QPushButton:hover { "
        "background-color: #66BB6A; "
        "border: 1px solid #4CAF50; "
        "}"
        "QPushButton:pressed { "
        "background-color: #388E3C; "
        "border: 1px solid #2E7D32; "
        "}"

        // QTreeWidget styling (similar to QLineEdit/loginForm for a clean look)
        "QTreeWidget { "
        "border: 1px solid #A0A0A0; "
        "background-color: white; " // A clean white background for the tree content
        "border-radius: 4px; "
        "color: #333333; "
        "alternate-background-color: #F8F8F8; " // Optional: for zebra striping
        "}"
        "QTreeWidget::item { "
        "padding: 3px; " // Add some padding to items
        "}"
        "QTreeWidget::branch { "
        "image: none; " // Hide branch indicators if you just want a flat list
        "}"
    );
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

