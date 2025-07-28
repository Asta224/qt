#pragma once

#include <QMainWindow> 
#include <QPushButton> 
#include <QTimer> // Timed events တွေအတွက် ထည့်သွင်းခြင်း။
#include <thread> // C++ thread support အတွက် ထည့်သွင်းခြင်း။
#include <memory> // Smart pointers (ဥပမာ: std::shared_ptr) အတွက် ထည့်သွင်းခြင်း။
#include <rclcpp/rclcpp.hpp> // ROS2 client library အတွက် ထည့်သွင်းခြင်း။
#include <geometry_msgs/msg/twist.hpp> // ROS2 Twist မက်ဆေ့ချ် အမျိုးအစား (robot velocity) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/laser_scan.hpp> // ROS2 LaserScan မက်ဆေ့ချ် အမျိုးအစား (Lidar data) အတွက် ထည့်သွင်းခြင်း။
#include <turtlesim/msg/pose.hpp> // ROS2 Turtlesim Pose မက်ဆေ့ချ် အမျိုးအစား (robot position/orientation) အတွက် ထည့်သွင်းခြင်း။
#include <nav_msgs/msg/odometry.hpp> // ROS2 Odometry မက်ဆေ့ချ် အမျိုးအစား (robot position/velocity estimate) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/joint_state.hpp> // ROS2 JointState မက်ဆေ့ချ် အမျိုးအစား (robot joint positions) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/image.hpp> // ROS2 Image မက်ဆေ့ချ် အမျိုးအစား (camera image data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/battery_state.hpp> // ROS2 BatteryState မက်ဆေ့ချ် အမျိုးအစား (battery info) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/imu.hpp> // ROS2 Imu မက်ဆေ့ချ် အမျိုးအစား (IMU sensor data) အတွက် ထည့်သွင်းခြင်း။
#include <nav_msgs/msg/occupancy_grid.hpp> // ROS2 OccupancyGrid မက်ဆေ့ချ် အမျိုးအစား (map data) အတွက် ထည့်သွင်းခြင်း။
#include <geometry_msgs/msg/pose_stamped.hpp> // ROS2 PoseStamped မက်ဆေ့ချ် အမျိုးအစား (pose with timestamp) အတွက် ထည့်သွင်းခြင်း။
#include <tf2_msgs/msg/tf_message.hpp> // ROS2 TFMessage အမျိုးအစား (coordinate frame transformations) အတွက် ထည့်သွင်းခြင်း။
#include <diagnostic_msgs/msg/diagnostic_array.hpp> // ROS2 DiagnosticArray မက်ဆေ့ချ် အမျိုးအစား (system diagnostics) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/laser_echo.hpp> // ROS2 LaserEcho မက်ဆေ့ချ် အမျိုးအစား (laser echo data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/fluid_pressure.hpp> // ROS2 FluidPressure မက်ဆေ့ချ် အမျိုးအစား (pressure sensor data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/magnetic_field.hpp> // ROS2 MagneticField မက်ဆေ့ချ် အမျိုးအစား (magnetometer data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/nav_sat_fix.hpp> // ROS2 NavSatFix မက်ဆေ့ချ် အမျိုးအစား (GPS data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/temperature.hpp> // ROS2 Temperature မက်ဆေ့ချ် အမျိုးအစား (temperature sensor data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/relative_humidity.hpp> // ROS2 RelativeHumidity မက်ဆေ့ချ် အမျိုးအစား (humidity sensor data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/range.hpp> // ROS2 Range မက်ဆေ့ချ် အမျိုးအစား (range sensor data) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/joy.hpp> // ROS2 Joy မက်ဆေ့ချ် အမျိုးအစား (joystick input) အတွက် ထည့်သွင်းခြင်း။
#include <turtlesim/msg/color.hpp> // ROS2 Turtlesim Color မက်ဆေ့ချ် အမျိုးအစား (turtlesim color) အတွက် ထည့်သွင်းခြင်း။
#include <nav_msgs/msg/path.hpp> // ROS2 Path မက်ဆေ့ချ် အမျိုးအစား (robot path) အတွက် ထည့်သွင်းခြင်း။
#include <sensor_msgs/msg/point_cloud2.hpp> // ROS2 PointCloud2 မက်ဆေ့ချ် အမျိုးအစား (3D point cloud data) အတွက် ထည့်သွင်းခြင်း။

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

    void checkROS2Connection(); // ROS2 connection ရှိမရှိ စစ်ဆေးတဲ့ slot ကို ကြေညာမယ်။

    void onAnyButtonClicked(); // မည်သည့် button ကိုမဆို click လုပ်တဲ့အခါ ခေါ်မယ့် slot ကို ကြေညာခြင်း။

    void showProcessMessage(const QString &buttonName); // Button နာမည်ကိုပြပြီး လုပ်ဆောင်ချက် မက်ဆေ့ချ်ပြသတဲ့ slot ကို ကြေညာခြင်း။

private:
    Ui::MainWindow *ui; // UI object ကို pointer အနေနဲ့ ကြေညာခြင်း။
    //
    QList<QPushButton*> allButtons; // Button အားလုံးကို သိမ်းထားမယ့် QList ကို ကြေညာခြင်း။
    QTimer *timer; // QTimer object ကို pointer အနေနဲ့ ကြေညာခြင်း။
    std::thread ros_thread; // ROS2 spin ကို run ဖို့အတွက် thread ကို ကြေညာခြင်း။
    std::shared_ptr<rclcpp::Node> ros_node; // ROS2 node ကို shared pointer အနေနဲ့ ကြေညာခြင်း။
    bool ros_ok = false; // ROS2 initialization အခြေအနေကို ပြတဲ့ flag ကို ကြေညာခြင်း။
    QString lastError; // နောက်ဆုံးပေါ်လာတဲ့ error ကို သိမ်းထားဖို့ ကြေညာခြင်း။

    // ROS2 topic များကို subscribe လုပ်ဖို့အတွက် shared pointers ကို ကြေညာခြင်း။
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
    QString last_cmd_vel, last_scan, last_turtle_pose, last_odom, last_joint_states, last_image, last_battery; // နောက်ဆုံးရရှိခဲ့တဲ့ topic data တွေကို သိမ်းဖို့ QString variable တွေ ကြေညာခြင်း။
    QString last_imu, last_map, last_goal_pose, last_tf, last_diag, last_laser_echo, last_fluid_pressure; // ကျန် topic data တွေကို သိမ်းဖို့ QString variable တွေ ကြေညာခြင်း။
    QString last_magnetic_field, last_navsatfix, last_temperature, last_humidity, last_range, last_joy, last_color; // ကျန် topic data တွေကို သိမ်းဖို့ QString variable တွေ ကြေညာခြင်း။
    QString last_nav_path, last_odom_filtered, last_pointcloud, last_battery_voltage; // ကျန် topic data တွေကို သိမ်းဖို့ QString variable တွေ ကြေညာခြင်း။
    bool cmd_vel_received = false, scan_received = false, turtle_pose_received = false, odom_received = false; // Topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
    bool joint_states_received = false, image_received = false, battery_received = false, imu_received = false; // ကျန် topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
    bool map_received = false, goal_pose_received = false, tf_received = false, diag_received = false; // ကျန် topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
    bool laser_echo_received = false, fluid_pressure_received = false, magnetic_field_received = false; // ကျန် topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
    bool navsatfix_received = false, temperature_received = false, humidity_received = false, range_received = false; // ကျန် topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
    bool joy_received = false, color_received = false, nav_path_received = false, odom_filtered_received = false; // ကျန် topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
    bool pointcloud_received = false, battery_voltage_received = false; // ကျန် topic data လက်ခံရရှိကြောင်း ပြတဲ့ boolean flag တွေ ကြေညာခြင်း။
};
