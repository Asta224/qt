#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <turtlesim/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <turtlesim/msg/color.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
    void checkROS2Connection();
    void onAnyButtonClicked();
    void showProcessMessage(const QString &buttonName);

private:
    Ui::MainWindow *ui;
    QList<QPushButton*> allButtons;
    QTimer *timer;
    std::thread ros_thread;
    std::shared_ptr<rclcpp::Node> ros_node;
    bool ros_ok = false;
    QString lastError;

    // Subscribers and data for 25 topics
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> scan_sub;
    std::shared_ptr<rclcpp::Subscription<turtlesim::msg::Pose>> turtle_pose_sub;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_states_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> image_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::BatteryState>> battery_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_sub;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>> map_sub;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> goal_pose_sub;
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> tf_sub;
    std::shared_ptr<rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>> diag_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserEcho>> laser_echo_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::FluidPressure>> fluid_pressure_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::MagneticField>> magnetic_field_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> navsatfix_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Temperature>> temperature_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>> humidity_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Range>> range_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;
    std::shared_ptr<rclcpp::Subscription<turtlesim::msg::Color>> color_sub;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> nav_path_sub;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_filtered_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_sub;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::BatteryState>> battery_voltage_sub;
   
    // Data and flags
    QString last_cmd_vel, last_scan, last_turtle_pose, last_odom, last_joint_states, last_image, last_battery;
    QString last_imu, last_map, last_goal_pose, last_tf, last_diag, last_laser_echo, last_fluid_pressure;
    QString last_magnetic_field, last_navsatfix, last_temperature, last_humidity, last_range, last_joy, last_color;
    QString last_nav_path, last_odom_filtered, last_pointcloud, last_battery_voltage;
    bool cmd_vel_received = false, scan_received = false, turtle_pose_received = false, odom_received = false;
    bool joint_states_received = false, image_received = false, battery_received = false, imu_received = false;
    bool map_received = false, goal_pose_received = false, tf_received = false, diag_received = false;
    bool laser_echo_received = false, fluid_pressure_received = false, magnetic_field_received = false;
    bool navsatfix_received = false, temperature_received = false, humidity_received = false, range_received = false;
    bool joy_received = false, color_received = false, nav_path_received = false, odom_filtered_received = false;
    bool pointcloud_received = false, battery_voltage_received = false;
};
