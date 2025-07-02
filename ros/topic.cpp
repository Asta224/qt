#include "topic.h"
#include "ui_topic.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/graph_listener.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <turtlesim/msg/pose.hpp>

#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QDateTime>
#include <QDebug>
#include <QMessageBox>

topic::topic(QMainWindow *maindwindow, QWidget *parent) : QDialog(parent),
                                                          ui(new Ui::topic),
                                                          m_mainwindow(maindwindow)
{
    ui->setupUi(this);
    this->setWindowState(Qt::WindowMaximized);

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    node_ = rclcpp::Node::make_shared("topic_monitor_node");

    resizeTreeWidgetColumns();

    connect(ui->searchButton, &QPushButton::clicked, this, &topic::on_searchButton_clicked);
    connect(ui->homeButton, &QPushButton::clicked, this, &topic::on_homeButton_clicked);

    ros2_timer_ = new QTimer(this);
    connect(ros2_timer_, &QTimer::timeout, this, &topic::ros2_spin_timer_callback);
    ros2_timer_->start(100);

    cmd_vel_movement_timer_ = new QTimer(this);
    connect(cmd_vel_movement_timer_, &QTimer::timeout, this, &topic::publish_cmd_vel_callback);

    connect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
}

topic::~topic()
{
    if (ros2_timer_)
    {
        if (ros2_timer_->isActive())
        {
            ros2_timer_->stop();
        }
        delete ros2_timer_;
        ros2_timer_ = nullptr;
    }
    if (cmd_vel_movement_timer_)
    {
        if (cmd_vel_movement_timer_->isActive())
        {
            cmd_vel_movement_timer_->stop();
        }
        delete cmd_vel_movement_timer_;
        cmd_vel_movement_timer_ = nullptr;
    }
    active_geometry_subscribers_.clear();
    main_type_items_.clear();
    topic_timestamps_.clear();
    topic_data_container_items_.clear(); 

    rclcpp::shutdown();
    delete ui;
}

void topic::ros2_spin_timer_callback()
{
    rclcpp::spin_some(node_);
}

void topic::on_searchButton_clicked()
{
    ui->treeWidget->clear();

    active_geometry_subscribers_.clear();
    main_type_items_.clear();
    topic_timestamps_.clear();
    topic_data_container_items_.clear(); 

    auto topics = node_->get_topic_names_and_types();
    for (const auto &topic_pair : topics)
    {
        QString topic_name = QString::fromStdString(topic_pair.first);
        QStringList topic_types_list;
        for (const auto &type : topic_pair.second)
        {
            topic_types_list += QString::fromStdString(type);
        }
        QString topic_types_str = topic_types_list.join(", ").trimmed();

        QTreeWidgetItem *parentItem = new QTreeWidgetItem(ui->treeWidget);
        parentItem->setFlags(parentItem->flags() | Qt::ItemIsUserCheckable);
        parentItem->setCheckState(0, Qt::Unchecked);
        parentItem->setText(1, topic_name);
        parentItem->setText(2, topic_types_str);
        parentItem->setText(3, "0 Hz");
        parentItem->setFlags(parentItem->flags() & ~Qt::ItemIsEditable);

        main_type_items_[topic_name] = parentItem;
        topic_timestamps_[topic_name] = QVector<rclcpp::Time>();

        auto publishers = node_->get_publishers_info_by_topic(topic_pair.first);
        for (const auto &pub_info : publishers)
        {
            QTreeWidgetItem *pubItem = new QTreeWidgetItem(parentItem);
            pubItem->setText(1, "Node: " + QString::fromStdString(pub_info.node_name()));
            pubItem->setText(2, "Publisher");
            pubItem->setFlags(pubItem->flags() & ~Qt::ItemIsEditable);
        }

        auto subscribers = node_->get_subscriptions_info_by_topic(topic_pair.first);
        for (const auto &sub_info : subscribers)
        {
            QTreeWidgetItem *subItem = new QTreeWidgetItem(parentItem);
            subItem->setText(1, "Node: " + QString::fromStdString(sub_info.node_name()));
            subItem->setText(2, "Subscriber");
            subItem->setFlags(subItem->flags() & ~Qt::ItemIsEditable);
        }

        QString type_for_subscription;
        QTreeWidgetItem *dataContainerItem = nullptr;

        if (topic_types_list.contains("geometry_msgs/msg/PoseStamped") ||
            topic_types_list.contains("geometry_msgs/msg/Twist") ||
            topic_types_list.contains("geometry_msgs/msg/PointStamped") ||
            topic_types_list.contains("turtlesim/msg/Pose"))
        {
            dataContainerItem = new QTreeWidgetItem(parentItem);
            dataContainerItem->setText(1, "");
            dataContainerItem->setText(2, "");
            dataContainerItem->setText(3, "");
            dataContainerItem->setFlags(dataContainerItem->flags() & ~Qt::ItemIsEditable);
            topic_data_container_items_[topic_name] = dataContainerItem; 
        }

        if (topic_types_list.contains("geometry_msgs/msg/PoseStamped")) {
            type_for_subscription = "geometry_msgs/msg/PoseStamped";
            QTreeWidgetItem *posItem = new QTreeWidgetItem(dataContainerItem);
            posItem->setText(1, "Position");
            posItem->setFlags(posItem->flags() & ~Qt::ItemIsEditable);
            QTreeWidgetItem *posX = new QTreeWidgetItem(posItem); posX->setText(1, "x:"); posX->setText(2, "0.000"); posX->setFlags(posX->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *posY = new QTreeWidgetItem(posItem); posY->setText(1, "y:"); posY->setText(2, "0.000"); posY->setFlags(posY->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *posZ = new QTreeWidgetItem(posItem); posZ->setText(1, "z:"); posZ->setText(2, "0.000"); posZ->setFlags(posZ->flags() | Qt::ItemIsEditable);

            QTreeWidgetItem *orientItem = new QTreeWidgetItem(dataContainerItem);
            orientItem->setText(1, "Orientation");
            orientItem->setFlags(orientItem->flags() & ~Qt::ItemIsEditable);
            QTreeWidgetItem *orientX = new QTreeWidgetItem(orientItem); orientX->setText(1, "x:"); orientX->setText(2, "0.000"); orientX->setFlags(orientX->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *orientY = new QTreeWidgetItem(orientItem); orientY->setText(1, "y:"); orientY->setText(2, "0.000"); orientY->setFlags(orientY->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *orientZ = new QTreeWidgetItem(orientItem); orientZ->setText(1, "z:"); orientZ->setText(2, "0.000"); orientZ->setFlags(orientZ->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *orientW = new QTreeWidgetItem(orientItem); orientW->setText(1, "w:"); orientW->setText(2, "1.000"); orientW->setFlags(orientW->flags() | Qt::ItemIsEditable);

        } else if (topic_types_#centralwidget{border-image: url(:image.jpeg) 0 0 0 0 stretch stretch;}"list.contains("geometry_msgs/msg/Twist")) {
            type_for_subscription = "geometry_msgs/msg/Twist";
            QTreeWidgetItem *linearItem = new QTreeWidgetItem(dataContainerItem);
            linearItem->setText(1, "Linear");
            linearItem->setFlags(linearItem->flags() & ~Qt::ItemIsEditable);
            QTreeWidgetItem *linearX = new QTreeWidgetItem(linearItem); linearX->setText(1, "x:"); linearX->setText(2, "0.000"); linearX->setFlags(linearX->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *linearY = new QTreeWidgetItem(linearItem); linearY->setText(1, "y:"); linearY->setText(2, "0.000"); linearY->setFlags(linearY->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *linearZ = new QTreeWidgetItem(linearItem); linearZ->setText(1, "z:"); linearZ->setText(2, "0.000"); linearZ->setFlags(linearZ->flags() | Qt::ItemIsEditable);

            QTreeWidgetItem *angularItem = new QTreeWidgetItem(dataContainerItem);
            angularItem->setText(1, "Angular");
            angularItem->setFlags(angularItem->flags() & ~Qt::ItemIsEditable);
            QTreeWidgetItem *angularX = new QTreeWidgetItem(angularItem); angularX->setText(1, "x:"); angularX->setText(2, "0.000"); angularX->setFlags(angularX->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *angularY = new QTreeWidgetItem(angularItem); angularY->setText(1, "y:"); angularY->setText(2, "0.000"); angularY->setFlags(angularY->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *angularZ = new QTreeWidgetItem(angularItem); angularZ->setText(1, "z:"); angularZ->setText(2, "0.000"); angularZ->setFlags(angularZ->flags() | Qt::ItemIsEditable);

        } else if (topic_types_list.contains("geometry_msgs/msg/PointStamped")) {
            type_for_subscription = "geometry_msgs/msg/PointStamped";
            QTreeWidgetItem *pointItem = new QTreeWidgetItem(dataContainerItem);
            pointItem->setText(1, "Point");
            pointItem->setFlags(pointItem->flags() & ~Qt::ItemIsEditable);
            QTreeWidgetItem *pointX = new QTreeWidgetItem(pointItem); pointX->setText(1, "x:"); pointX->setText(2, "0.000"); pointX->setFlags(pointX->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *pointY = new QTreeWidgetItem(pointItem); pointY->setText(1, "y:"); pointY->setText(2, "0.000"); pointY->setFlags(pointY->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *pointZ = new QTreeWidgetItem(pointItem); pointZ->setText(1, "z:"); pointZ->setText(2, "0.000"); pointZ->setFlags(pointZ->flags() | Qt::ItemIsEditable);

        } else if (topic_types_list.contains("turtlesim/msg/Pose")) {
            type_for_subscription = "turtlesim/msg/Pose";
            QTreeWidgetItem *poseItem = new QTreeWidgetItem(dataContainerItem);
            poseItem->setText(1, "Pose");
            poseItem->setFlags(poseItem->flags() & ~Qt::ItemIsEditable);
            QTreeWidgetItem *posX = new QTreeWidgetItem(poseItem); posX->setText(1, "x:"); posX->setText(2, "0.000"); posX->setFlags(posX->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *posY = new QTreeWidgetItem(poseItem); posY->setText(1, "y:"); posY->setText(2, "0.000"); posY->setFlags(posY->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *theta = new QTreeWidgetItem(poseItem); theta->setText(1, "theta:"); theta->setText(2, "0.000"); theta->setFlags(theta->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *linear_vel = new QTreeWidgetItem(poseItem); linear_vel->setText(1, "linear_velocity:"); linear_vel->setText(2, "0.000"); linear_vel->setFlags(linear_vel->flags() | Qt::ItemIsEditable);
            QTreeWidgetItem *angular_vel = new QTreeWidgetItem(poseItem); angular_vel->setText(1, "angular_velocity:"); angular_vel->setText(2, "0.000"); angular_vel->setFlags(angular_vel->flags() | Qt::ItemIsEditable);
        }

        if (!type_for_subscription.isEmpty())
        {
            if (type_for_subscription == "geometry_msgs/msg/PoseStamped")
            {
                auto sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                    topic_pair.first,
                    rclcpp::QoS(10),
                    [this, topic_name](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                    {
                        if (main_type_items_.contains(topic_name)) 
                        {
                            this->updatePoseStampedData(main_type_items_[topic_name], msg, topic_name);
                        }
                    });
                active_geometry_subscribers_[topic_name] = sub;
            }
            else if (type_for_subscription == "geometry_msgs/msg/Twist")
            {
                auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
                    topic_pair.first,
                    rclcpp::QoS(10),
                    [this, topic_name](const geometry_msgs::msg::Twist::SharedPtr msg)
                    {
                        if (main_type_items_.contains(topic_name)) 
                        {
                            this->updateTwistData(main_type_items_[topic_name], msg, topic_name);
                        }
                    });
                active_geometry_subscribers_[topic_name] = sub;
            }
            else if (type_for_subscription == "geometry_msgs/msg/PointStamped")
            {
                auto sub = node_->create_subscription<geometry_msgs::msg::PointStamped>(
                    topic_pair.first,
                    rclcpp::QoS(10),
                    [this, topic_name](const geometry_msgs::msg::PointStamped::SharedPtr msg)
                    {
                        if (main_type_items_.contains(topic_name)) 
                        {
                            this->updatePointStampedData(main_type_items_[topic_name], msg, topic_name);
                        }
                    });
                active_geometry_subscribers_[topic_name] = sub;
            } else if (type_for_subscription == "turtlesim/msg/Pose") {
                auto sub = node_->create_subscription<turtlesim::msg::Pose>(
                    topic_pair.first,
                    rclcpp::QoS(10),
                    [this, topic_name](const turtlesim::msg::Pose::SharedPtr msg) {
                        if (main_type_items_.contains(topic_name)) { 
                            this->updateTurtlePoseData(main_type_items_[topic_name], msg, topic_name);
                        }
                    }
                );
                active_geometry_subscribers_[topic_name] = sub;
            }
        }
        parentItem->setExpanded(false);
    }

    resizeTreeWidgetColumns();
}

void topic::on_treeWidgetItem_changed(QTreeWidgetItem *item, int column){
    // Disconnect to prevent re-triggering during programmatic updates
    disconnect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);

    if (column == 0) { // Checkbox column
        if (item->parent() == nullptr) { // Top-level topic item
            QString topic_name = item->text(1);
            if (topic_name == "/turtle1/cmd_vel") {
                if (item->checkState(0) == Qt::Checked) {
                    qDebug() << "Checkbox for" << topic_name << "CHECKED. Starting turtle movement.";
                    double fixed_frequency_hz = 10.0;
                    int interval_ms = static_cast<int>(1000.0 / fixed_frequency_hz);
                    if (interval_ms < 1) interval_ms = 1;

                    if (!cmd_vel_publisher_) {
                        cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
                    }
                    cmd_vel_movement_timer_->start(interval_ms);
                    qDebug() << "Publishing /turtle1/cmd_vel at fixed" << fixed_frequency_hz << "Hz (interval:" << interval_ms << "ms)";

                } else { // Checkbox UNCHECKED
                    qDebug() << "Checkbox for" << topic_name << "UNCHECKED. Stopping turtle movement and resetting values.";
                    cmd_vel_movement_timer_->stop(); // Stop the publishing timer
                    if (cmd_vel_publisher_) {
                        // Publish a zero Twist message to ensure the turtle stops immediately
                        auto stop_msg = geometry_msgs::msg::Twist();
                        stop_msg.linear.x = 0.0;
                        stop_msg.angular.z = 0.0;
                        cmd_vel_publisher_->publish(stop_msg);
                    }

                    // Reset internal current_cmd_vel variables to 0.0
                    current_linear_x_ = 0.0;
                    current_linear_y_ = 0.0; 
                    current_linear_z_ = 0.0;
                    current_angular_x_ = 0.0; 
                    current_angular_y_ = 0.0;
                    current_angular_z_ = 0.0;

                    // Explicitly update the QTreeWidget UI to show "0.000" for all cmd_vel components
                    QTreeWidgetItem *dataContainerItem = topic_data_container_items_.value(topic_name);
                    if (dataContainerItem) {
                        QTreeWidgetItem *linearItem = findChildByText(dataContainerItem, "Linear");
                        if (linearItem) {
                            QTreeWidgetItem *linearX = findChildByText(linearItem, "x:");
                            if (linearX) linearX->setText(2, "0.000"); // Force "0.000"
                            QTreeWidgetItem *linearY = findChildByText(linearItem, "y:");
                            if (linearY) linearY->setText(2, "0.000");
                            QTreeWidgetItem *linearZ = findChildByText(linearItem, "z:");
                            if (linearZ) linearZ->setText(2, "0.000");
                        }
                        QTreeWidgetItem *angularItem = findChildByText(dataContainerItem, "Angular");
                        if (angularItem) {
                            QTreeWidgetItem *angularX = findChildByText(angularItem, "x:");
                            if (angularX) angularX->setText(2, "0.000");
                            QTreeWidgetItem *angularY = findChildByText(angularItem, "y:");
                            if (angularY) angularY->setText(2, "0.000");
                            QTreeWidgetItem *angularZ = findChildByText(angularItem, "z:");
                            if (angularZ) angularZ->setText(2, "0.000");
                        }
                    }
                }
            }
        }
    }
    else if (column == 2) { // Value column (editable)
        bool ok;
        double newValue = item->text(2).toDouble(&ok);

        if(ok){
            QTreeWidgetItem *dataFieldItem = item;
            QTreeWidgetItem *dataGroupItem = dataFieldItem->parent();
            QTreeWidgetItem *dataContainerItem = dataGroupItem ? dataGroupItem->parent() : nullptr;
            QTreeWidgetItem *topicParentItem = dataContainerItem ? dataContainerItem->parent() : nullptr;

            // Check if the changed item belongs to "/turtle1/cmd_vel" and if its checkbox is checked
            if (topicParentItem && topicParentItem->text(1) == "/turtle1/cmd_vel" && topicParentItem->checkState(0) == Qt::Checked) {
                QString groupText = dataGroupItem->text(1);
                QString fieldText = dataFieldItem->text(1);

                // Update the corresponding internal member variable
                if (groupText == "Linear") {
                    if (fieldText == "x:") current_linear_x_ = newValue;
                    else if (fieldText == "y:") current_linear_y_ = newValue;
                    else if (fieldText == "z:") current_linear_z_ = newValue;
                    qDebug() << "Updated linear " << fieldText << "to:" << newValue;
                } else if (groupText == "Angular") {
                    if (fieldText == "x:") current_angular_x_ = newValue;
                    else if (fieldText == "y:") current_angular_y_ = newValue;
                    else if (fieldText == "z:") current_angular_z_ = newValue;
                    qDebug() << "Updated angular " << fieldText << "to:" << newValue;
                }
            } else {
                qDebug() << "Value changed for non-cmd_vel item or unchecked cmd_vel topic:" << item->text(1) << "New value:" << newValue;
            }
        } else {
            qWarning() << "Invalid numerical value entered for item:" << item->text(1) <<"Text:"<<item->text(2);
        }
    }
    // Reconnect after programmatic updates
    connect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
}

void topic::publish_cmd_vel_callback() {
    if (cmd_vel_publisher_) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = current_linear_x_;
        twist_msg.linear.y = current_linear_y_;
        twist_msg.linear.z = current_linear_z_;
        twist_msg.angular.x = current_angular_x_;
        twist_msg.angular.y = current_angular_y_;
        twist_msg.angular.z = current_angular_z_;
           qDebug() << "DEBUG: Publishing Twist - Linear.x:" << twist_msg.linear.x
                 << " Linear.y:" << twist_msg.linear.y
                 << " Linear.z:" << twist_msg.linear.z
                 << " Angular.x:" << twist_msg.angular.x
                 << " Angular.y:" << twist_msg.angular.y
                 << " Angular.z:" << twist_msg.angular.z;
        cmd_vel_publisher_->publish(twist_msg);
    }
}

void topic::calculateAndDisplayFrequency(const QString &topic_name, const rclcpp::Time &msg_timestamp, QTreeWidgetItem *main_topic_parent_item)
{
    if (!main_topic_parent_item) return;

    QVector<rclcpp::Time> &timestamps = topic_timestamps_[topic_name];
    timestamps.push_back(msg_timestamp);

    while (timestamps.size() > MAX_TIMESTAMP_HISTORY)
    {
        timestamps.removeFirst();
    }

    if (timestamps.size() >= 2)
    {
        double total_duration_sec = (timestamps.last() - timestamps.first()).seconds();
        int num_intervals = timestamps.size() - 1;

        if (num_intervals > 0 && total_duration_sec > 0.0)
        {
            double frequency_hz = (double)num_intervals / total_duration_sec;
            
            main_topic_parent_item->setText(3, QString::number(frequency_hz, 'f', 2) + " Hz");
        }
        else
        {
            main_topic_parent_item->setText(3, "0.00 Hz");
        }
    }
    else
    {
        main_topic_parent_item->setText(3, "0.00 Hz");
    }
}

QTreeWidgetItem *topic::findChildByText(QTreeWidgetItem *parent, const QString &text)
{
    if (!parent)
        return nullptr;
    for (int i = 0; i < parent->childCount(); ++i)
    {
        if (parent->child(i)->text(1) == text)
        {
            return parent->child(i);
        }
    }
    return nullptr;
}

void topic::updatePoseStampedData(QTreeWidgetItem *main_topic_parent_item, const geometry_msgs::msg::PoseStamped::SharedPtr msg, const QString &topic_name)
{
    if (!main_topic_parent_item) return;

    QTreeWidgetItem *dataContainerItem = topic_data_container_items_.value(topic_name);
    if (!dataContainerItem) {
        qWarning() << "Error: Blank data container item not found for PoseStamped under topic" << topic_name;
        return;
    }

    QTreeWidgetItem *posItem = findChildByText(dataContainerItem, "Position");
    QTreeWidgetItem *orientItem = findChildByText(dataContainerItem, "Orientation");

    if (!posItem || !orientItem)
    {
        qWarning() << "Error: 'Position/Orientation' item not found for PoseStamped under data container for topic" << topic_name;
        return;
    }

    disconnect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
    findChildByText(posItem, "x:")->setText(2, QString::number(msg->pose.position.x, 'f', 3));
    findChildByText(posItem, "y:")->setText(2, QString::number(msg->pose.position.y, 'f', 3));
    findChildByText(posItem, "z:")->setText(2, QString::number(msg->pose.position.z, 'f', 3));
    findChildByText(orientItem, "x:")->setText(2, QString::number(msg->pose.orientation.x, 'f', 3));
    findChildByText(orientItem, "y:")->setText(2, QString::number(msg->pose.orientation.y, 'f', 3));
    findChildByText(orientItem, "z:")->setText(2, QString::number(msg->pose.orientation.z, 'f', 3));
    findChildByText(orientItem, "w:")->setText(2, QString::number(msg->pose.orientation.w, 'f', 3));
    connect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);

    calculateAndDisplayFrequency(topic_name, msg->header.stamp, main_topic_parent_item);
}

void topic::updateTwistData(QTreeWidgetItem *main_topic_parent_item, const geometry_msgs::msg::Twist::SharedPtr msg, const QString &topic_name)
{
    if (!main_topic_parent_item) return;

    QTreeWidgetItem *dataContainerItem = topic_data_container_items_.value(topic_name);
    if (!dataContainerItem) {
        qWarning() << "Error: Blank data container item not found for Twist under topic" << topic_name;
        return;
    }

    QTreeWidgetItem *linearItem = findChildByText(dataContainerItem, "Linear");
    QTreeWidgetItem *angularItem = findChildByText(dataContainerItem, "Angular");

    if (!linearItem || !angularItem)
    {
        qWarning() << "Error: 'Linear/Angular' item not found for Twist under data container for topic" << topic_name;
        return;
    }
   
    disconnect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
    findChildByText(linearItem, "x:")->setText(2, QString::number(msg->linear.x, 'f', 3));
    findChildByText(linearItem, "y:")->setText(2, QString::number(msg->linear.y, 'f', 3));
    findChildByText(linearItem, "z:")->setText(2, QString::number(msg->linear.z, 'f', 3));
    findChildByText(angularItem, "x:")->setText(2, QString::number(msg->angular.x, 'f', 3));
    findChildByText(angularItem, "y:")->setText(2, QString::number(msg->angular.y, 'f', 3));
    findChildByText(angularItem, "z:")->setText(2, QString::number(msg->angular.z, 'f', 3));
    connect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);

    calculateAndDisplayFrequency(topic_name, node_->now(), main_topic_parent_item);
}

void topic::updatePointStampedData(QTreeWidgetItem *main_topic_parent_item, const geometry_msgs::msg::PointStamped::SharedPtr msg, const QString &topic_name)
{
    if (!main_topic_parent_item) return;

    QTreeWidgetItem *dataContainerItem = topic_data_container_items_.value(topic_name);
    if (!dataContainerItem) {
        qWarning() << "Error: Blank data container item not found for PointStamped under topic" << topic_name;
        return;
    }

    QTreeWidgetItem *pointItem = findChildByText(dataContainerItem, "Point");

    if (!pointItem)
    {
        qWarning() << "Error: 'Point' item not found for PointStamped under data container for topic" << topic_name;
        return;
    }

    disconnect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
    findChildByText(pointItem, "x:")->setText(2, QString::number(msg->point.x, 'f', 3));
    findChildByText(pointItem, "y:")->setText(2, QString::number(msg->point.y, 'f', 3));
    findChildByText(pointItem, "z:")->setText(2, QString::number(msg->point.z, 'f', 3));
    connect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
    
    calculateAndDisplayFrequency(topic_name, msg->header.stamp, main_topic_parent_item);
}

void topic::updateTurtlePoseData(QTreeWidgetItem* main_topic_parent_item, const turtlesim::msg::Pose::SharedPtr msg, const QString& topic_name) {
    if (!main_topic_parent_item) return;

    QTreeWidgetItem *dataContainerItem = topic_data_container_items_.value(topic_name);
    if (!dataContainerItem) {
        qWarning() << "Error: Blank data container item not found for turtlesim/Pose under topic" << topic_name;
        return;
    }

    QTreeWidgetItem *poseItem = findChildByText(dataContainerItem, "Pose");

    if (!poseItem) { 
        qWarning() << "Error: 'Pose' item not found for turtlesim/Pose under data container for topic" << topic_name; 
        return; 
    }
    disconnect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
    findChildByText(poseItem, "x:")->setText(2, QString::number(msg->x, 'f', 3));
    findChildByText(poseItem, "y:")->setText(2, QString::number(msg->y, 'f', 3));
    findChildByText(poseItem, "theta:")->setText(2, QString::number(msg->theta, 'f', 3));
    findChildByText(poseItem, "linear_velocity:")->setText(2, QString::number(msg->linear_velocity, 'f', 3));
    findChildByText(poseItem, "angular_velocity:")->setText(2, QString::number(msg->angular_velocity, 'f', 3));
    connect(ui->treeWidget, &QTreeWidget::itemChanged, this, &topic::on_treeWidgetItem_changed);
    
    calculateAndDisplayFrequency(topic_name, node_->now(), main_topic_parent_item); 
}

void topic::on_homeButton_clicked()
{
    this->close();
    m_mainwindow->show();
}

void topic::resizeTreeWidgetColumns()
{
    ui->treeWidget->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->treeWidget->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->treeWidget->header()->setSectionResizeMode(2, QHeaderView::Stretch);
    ui->treeWidget->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
}