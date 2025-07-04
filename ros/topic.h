#ifndef TOPIC_H
#define TOPIC_H

#include <QDialog>
#include <rclcpp/rclcpp.hpp>
#include <QMap>
#include <QVector>
#include <QTimer>
#include <QTreeWidgetItem>
#include <QMainWindow>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <turtlesim/msg/pose.hpp>


QT_BEGIN_NAMESPACE
namespace Ui
{
    class topic;
}
QT_END_NAMESPACE

class topic : public QDialog
{
    Q_OBJECT

public:
    explicit topic(QMainWindow *maindwindow, QWidget *parent = nullptr);
    ~topic();


private slots:
    void on_searchButton_clicked();
    void on_homeButton_clicked();
    void ros2_spin_timer_callback();
    void publish_cmd_vel_callback();
    void on_treeWidgetItem_changed(QTreeWidgetItem *item, int column);
    

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    void resizeTreeWidgetColumns();
    void calculateAndDisplayFrequency(const QString &topic_name, const rclcpp::Time &msg_timestamp, QTreeWidgetItem *main_topic_parent_item);
    QTreeWidgetItem *findChildByText(QTreeWidgetItem *parent, const QString &text);

    void updatePoseStampedData(QTreeWidgetItem *main_topic_parent_item, const geometry_msgs::msg::PoseStamped::SharedPtr msg, const QString &topic_name);
    void updateTwistData(QTreeWidgetItem *main_topic_parent_item, const geometry_msgs::msg::Twist::SharedPtr msg, const QString &topic_name);
    void updatePointStampedData(QTreeWidgetItem *main_topic_parent_item, const geometry_msgs::msg::PointStamped::SharedPtr msg, const QString &topic_name);
    void updateTurtlePoseData(QTreeWidgetItem *main_topic_parent_item, const turtlesim::msg::Pose::SharedPtr msg, const QString &topic_name);

    Ui::topic *ui;
    QMainWindow *m_mainwindow;
    rclcpp::Node::SharedPtr node_;

    QMap<QString, rclcpp::SubscriptionBase::SharedPtr> active_geometry_subscribers_;
    QMap<QString, QTreeWidgetItem *> main_type_items_;
    QMap<QString, QVector<rclcpp::Time>> topic_timestamps_;
    QMap<QString, QTreeWidgetItem *> topic_data_container_items_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    QTimer *cmd_vel_movement_timer_;

    double current_linear_x_ = 0.0;
    double current_linear_y_ = 0.0;
    double current_linear_z_ = 0.0;
    double current_angular_x_ = 0.0;
    double current_angular_y_ = 0.0;
    double current_angular_z_ = 0.0;

    QTimer *ros2_timer_;

    static constexpr int MAX_TIMESTAMP_HISTORY = 100;
};

#endif // TOPIC_H