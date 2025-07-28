#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox> // Message box ပြသဖို့အတွက် ထည့်သွင်းခြင်း။
#include <QProcess> // External process တွေ run ဖို့အတွက် (ဒီ code မှာတော့ တိုက်ရိုက်မသုံးထားပါ) ထည့်သွင်းခြင်း။
#include <QMap> // Key-value pair တွေကို သိမ်းဖို့ QMap ကို ထည့်သွင်းခြင်း။
#include <QStringList> // String list တွေအတွက် QStringList ကို ထည့်သွင်းခြင်း။

// Topic info struct and map
// Topic တစ်ခုချင်းစီရဲ့ အချက်အလက်တွေကို သိမ်းဆည်းဖို့ TopicInfo struct ကို ကြေညာခြင်း။
struct TopicInfo {
    QString user_name;    // User ကို ပြသမယ့် Topic နာမည်။
    QString topic_name;   // ROS2 topic အပြည့်အစုံ နာမည်။
    QString type;         // Topic ရဲ့ Message Type။
    QString description;  // Topic ရဲ့ ဖော်ပြချက်။
};

// Button နာမည် (ObjectName) နဲ့ သက်ဆိုင်ရာ TopicInfo ကို တွဲမှတ်ထားတဲ့ QMap ကို ကြေညာခြင်း။
const QMap<QString, TopicInfo> topicInfoMap = {
    { "cmd_velButton",      { "Robot Velocity", "/cmd_vel", "geometry_msgs/msg/Twist", "Robot velocity command input" } }, // robot ရဲ့ အရှိန်နှုန်း ထိန်းချုပ်တဲ့ topic။
    { "scanButton",         { "Laser Scan", "/scan", "sensor_msgs/msg/LaserScan", "Laser scanner data" } }, // Laser scanner ကနေ ရတဲ့ အချက်အလက်တွေ။
    { "turtlePoseButton",   { "Turtle Pose", "/turtle1/pose", "turtlesim/msg/Pose", "Turtlesim pose feedback" } }, // Turtlesim ထဲက လိပ်ရဲ့ အနေအထား။
    { "odomButton",         { "Odometry", "/odom", "nav_msgs/msg/Odometry", "Odometry feedback" } }, // robot ရဲ့ အနေအထား ခန့်မှန်းချက်။
    { "jointStatesButton",  { "Joint States", "/joint_states", "sensor_msgs/msg/JointState", "Joint state feedback" } }, // robot ရဲ့ အဆစ်အဆက်တွေရဲ့ အနေအထား။
    { "imageButton",        { "Camera Image", "/camera/image_raw", "sensor_msgs/msg/Image", "Camera image stream" } }, // ကင်မရာကနေ ရတဲ့ ပုံရိပ်အချက်အလက်။
    { "batteryButton",      { "Battery State", "/battery_state", "sensor_msgs/msg/BatteryState", "Battery status" } }, // ဘက်ထရီ အခြေအနေ။
    { "imuButton",          { "IMU Data", "/imu/data", "sensor_msgs/msg/Imu", "IMU sensor data" } }, // IMU (Inertial Measurement Unit) ကနေ ရတဲ့ အချက်အလက်။
    { "mapButton",          { "Map", "/map", "nav_msgs/msg/OccupancyGrid", "Occupancy grid map" } }, // robot ပတ်ဝန်းကျင်မြေပုံ။
    { "goalPoseButton",     { "Goal Pose", "/goal_pose", "geometry_msgs/msg/PoseStamped", "Navigation goal pose" } }, // လမ်းကြောင်းသွားမယ့် ပန်းတိုင် အနေအထား။
    { "tfButton",           { "TF", "/tf", "tf2_msgs/msg/TFMessage", "Transform tree" } }, // Coordinate frames တွေကြားက ပြောင်းလဲမှု (transformation) အချက်အလက်။
    { "diagButton",         { "Diagnostics", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", "System diagnostics" } }, // စနစ်ရဲ့ လုပ်ဆောင်မှု အခြေအနေ စစ်ဆေးချက်များ။
    { "laserEchoButton",    { "Laser Echo", "/echo", "sensor_msgs/msg/LaserEcho", "Laser echo data" } }, // Laser echo sensor data (ရှားပါး)။
    { "fluidPressureButton",{ "Fluid Pressure", "/fluid_pressure", "sensor_msgs/msg/FluidPressure", "Fluid pressure sensor" } }, // အရည်ဖိအား sensor data။
    { "magneticFieldButton",{ "Magnetic Field", "/magnetic_field", "sensor_msgs/msg/MagneticField", "Magnetic field sensor" } }, // သံလိုက်စက်ကွင်း sensor data။
    { "navSatFixButton",    { "GPS Fix", "/gps/fix", "sensor_msgs/msg/NavSatFix", "GPS position fix" } }, // GPS နေရာသတ်မှတ်မှု အချက်အလက်။
    { "temperatureButton",  { "Temperature", "/temperature", "sensor_msgs/msg/Temperature", "Temperature sensor" } }, // အပူချိန် sensor data။
    { "relativeHumidityButton", { "Humidity", "/humidity", "sensor_msgs/msg/RelativeHumidity", "Humidity sensor" } }, // စိုထိုင်းဆ sensor data။
    { "rangeButton",        { "Range", "/range", "sensor_msgs/msg/Range", "Range sensor" } }, // အကွာအဝေး sensor data။
    { "joyButton",          { "Joystick", "/joy", "sensor_msgs/msg/Joy", "Joystick/gamepad input" } }, // Joystick သို့မဟုတ် gamepad ကနေ ရတဲ့ input။
    { "colorButton",        { "Turtle Color", "/turtle1/color_sensor", "turtlesim/msg/Color", "Turtlesim color sensor" } }, // Turtlesim လိပ်ရဲ့ color sensor။
    { "navPathButton",      { "Path", "/plan", "nav_msgs/msg/Path", "Planned navigation path" } }, // စီစဉ်ထားသော လမ်းကြောင်း။
    { "odomFilteredButton", { "Filtered Odometry", "/odometry/filtered", "nav_msgs/msg/Odometry", "Filtered odometry (e.g. EKF)" } }, // စစ်ထုတ်ထားသော Odometry (ဥပမာ EKF မှ)။
    { "pointCloudButton",   { "Point Cloud", "/points_raw", "sensor_msgs/msg/PointCloud2", "Raw point cloud data" } }, // 3D point cloud data။
    { "batteryVoltageButton", { "Battery Voltage", "/battery/voltage", "sensor_msgs/msg/BatteryState", "Battery voltage" } } // ဘက်ထရီ ဗို့အား (battery_state message ကို ပြန်သုံးထား)။
};

// MainWindow class ရဲ့ constructor ကို အစပျိုးခြင်း။
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), ros_ok(false) // Parent widget ကို ပေးပို့ပြီး UI ကို အစပျိုးကာ ros_ok ကို false ပေးခြင်း။
{
    ui->setupUi(this); // UI component တွေကို setup လုပ်ခြင်း။

    // Minimalist theme (no animation)
    // UI အတွက် CSS style sheet ကို သတ်မှတ်ခြင်း။
    this->setStyleSheet(R"(
    QWidget {
        background: rgba(117, 238, 157, 1); // နောက်ခံအရောင်ကို ဖော်ပြခြင်း။
        font-family: 'Segoe UI', 'Arial', sans-serif; // font အမျိုးအစားကို သတ်မှတ်ခြင်း။
        font-size: 13px; // font အရွယ်အစားကို သတ်မှတ်ခြင်း။
    }
    QPushButton {
        background: #ffffff; // Button နောက်ခံအရောင်ကို သတ်မှတ်ခြင်း။
        border: 1px solid #dddddd; // Button ဘောင်အရောင်နှင့် အထူကို သတ်မှတ်ခြင်း။
        border-radius: 8px; // Button ဘောင်အဝိုင်းကို သတ်မှတ်ခြင်း။
        padding: 8px 0; // Button padding ကို သတ်မှတ်ခြင်း။
        color: #222; // Button စာလုံးအရောင်ကို သတ်မှတ်ခြင်း။
    }
    QPushButton:hover {
        background: #e6f0fa; // Button ပေါ် mouse တင်တဲ့အခါ နောက်ခံအရောင်ကို သတ်မှတ်ခြင်း။
        border: 1.5px solid #7bb6f2; // Button ပေါ် mouse တင်တဲ့အခါ ဘောင်အရောင်နှင့် အထူကို သတ်မှတ်ခြင်း။
        color: #222; // Button ပေါ် mouse တင်တဲ့အခါ စာလုံးအရောင်ကို သတ်မှတ်ခြင်း။
    }
    QPushButton:pressed {
        background: #d0e6fa; // Button နှိပ်ထားစဉ် နောက်ခံအရောင်ကို သတ်မှတ်ခြင်း။
        border: 1.5px solid #1976d2; // Button နှိပ်ထားစဉ် ဘောင်အရောင်နှင့် အထူကို သတ်မှတ်ခြင်း။
        color: #222; // Button နှိပ်ထားစဉ် စာလုံးအရောင်ကို သတ်မှတ်ခြင်း။
    }
)");

    // Collect all buttons
    allButtons = findChildren<QPushButton *>(); // UI ထဲရှိ QPushButton အားလုံးကို ရှာပြီး allButtons ထဲ ထည့်ခြင်း။
    for (auto btn : allButtons) // Button တစ်ခုချင်းစီကို စစ်ဆေးခြင်း။
        btn->setStyleSheet("background-color: grey;"); // အစအဦးတွင် button ၏ နောက်ခံအရောင်ကို မီးခိုးရောင် သတ်မှတ်ခြင်း။
    for (auto btn : allButtons) // Button တစ်ခုချင်းစီကို စစ်ဆေးခြင်း။
        connect(btn, &QPushButton::clicked, this, &MainWindow::onAnyButtonClicked); // Button နှိပ်လိုက်တိုင်း onAnyButtonClicked slot ကို ချိတ်ဆက်ခြင်း။

    // Initialize received flags
    // Topic data လက်ခံရရှိခြင်း ရှိမရှိပြသော flags အားလုံးကို false အဖြစ် အစပျိုးခြင်း။
    cmd_vel_received = scan_received = turtle_pose_received = odom_received = false;
    joint_states_received = image_received = battery_received = imu_received = false;
    map_received = goal_pose_received = tf_received = diag_received = false;
    laser_echo_received = fluid_pressure_received = magnetic_field_received = false;
    navsatfix_received = temperature_received = humidity_received = range_received = false;
    joy_received = color_received = nav_path_received = odom_filtered_received = false;
    pointcloud_received = battery_voltage_received = false;

    // Start ROS2 in a thread
    // ROS2 spin ကို သီးခြား thread တစ်ခုမှာ စတင်ခြင်း။
    ros_thread = std::thread([this]()
    {
        try {
            int argc = 0; // ROS2 init အတွက် argument count ကို သတ်မှတ်ခြင်း။
            char **argv = nullptr; // ROS2 init အတွက် argument vector ကို သတ်မှတ်ခြင်း။
            rclcpp::init(argc, argv); // ROS2 library ကို အစပျိုးခြင်း။
            ros_node = std::make_shared<rclcpp::Node>("qt_ros2_monitor"); // "qt_ros2_monitor" နာမည်နဲ့ ROS2 node အသစ် ဖန်တီးခြင်း။

            // ROS2 Subscribers များကို ဖန်တီးခြင်း။
            // တစ်ခုချင်းစီအတွက် topic နာမည်၊ QoS history depth (10) နှင့် callback function များကို သတ်မှတ်ပေးခြင်း။
            cmd_vel_sub = ros_node->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10,
                [this](geometry_msgs::msg::Twist::SharedPtr msg) { // cmd_vel topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_cmd_vel = QString("linear.x: %1, angular.z: %2") // နောက်ဆုံးရရှိခဲ့သော linear.x နှင့် angular.z data ကို သိမ်းခြင်း။
                        .arg(msg->linear.x)
                        .arg(msg->angular.z);
                    cmd_vel_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            scan_sub = ros_node->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10,
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { // scan topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_scan = msg->ranges.empty() ? "No scan data." : QString("ranges[0]: %1").arg(msg->ranges[0]); // scan data ကို သိမ်းခြင်း။
                    scan_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            turtle_pose_sub = ros_node->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                [this](turtlesim::msg::Pose::SharedPtr msg) { // turtle_pose topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_turtle_pose = QString("x: %1, y: %2, theta: %3") // turtle pose data ကို သိမ်းခြင်း။
                        .arg(msg->x)
                        .arg(msg->y)
                        .arg(msg->theta);
                    turtle_pose_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            odom_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                [this](nav_msgs::msg::Odometry::SharedPtr msg) { // odom topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_odom = QString("x: %1, y: %2") // odometry position data ကို သိမ်းခြင်း။
                        .arg(msg->pose.pose.position.x)
                        .arg(msg->pose.pose.position.y);
                    odom_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            joint_states_sub = ros_node->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg) { // joint_states topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_joint_states = QString("name[0]: %1, position[0]: %2") // joint state data ကို သိမ်းခြင်း။
                        .arg(msg->name.empty() ? "none" : QString::fromStdString(msg->name[0]))
                        .arg(msg->position.empty() ? 0.0 : msg->position[0]);
                    joint_states_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            image_sub = ros_node->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10,
                [this](sensor_msgs::msg::Image::SharedPtr msg) { // image topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_image = QString("width: %1, height: %2") // image width နှင့် height ကို သိမ်းခြင်း။
                        .arg(msg->width)
                        .arg(msg->height);
                    image_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            battery_sub = ros_node->create_subscription<sensor_msgs::msg::BatteryState>(
                "/battery_state", 10,
                [this](sensor_msgs::msg::BatteryState::SharedPtr msg) { // battery_state topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_battery = QString("voltage: %1, percentage: %2") // battery voltage နှင့် percentage ကို သိမ်းခြင်း။
                        .arg(msg->voltage)
                        .arg(msg->percentage);
                    battery_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            imu_sub = ros_node->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 10,
                [this](sensor_msgs::msg::Imu::SharedPtr msg) { // imu topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_imu = QString("Orientation: x=%1, y=%2, z=%3, w=%4") // IMU orientation data ကို သိမ်းခြင်း။
                        .arg(msg->orientation.x).arg(msg->orientation.y)
                        .arg(msg->orientation.z).arg(msg->orientation.w);
                    imu_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            map_sub = ros_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10,
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { // map topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_map = QString("Map: %1x%2").arg(msg->info.width).arg(msg->info.height); // map dimension ကို သိမ်းခြင်း။
                    map_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            goal_pose_sub = ros_node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10,
                [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { // goal_pose topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_goal_pose = QString("Goal: x=%1, y=%2") // goal pose data ကို သိမ်းခြင်း။
                        .arg(msg->pose.position.x).arg(msg->pose.position.y);
                    goal_pose_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            tf_sub = ros_node->create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf", 10,
                [this](tf2_msgs::msg::TFMessage::SharedPtr msg) { // tf topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_tf = QString("Transforms: %1").arg(msg->transforms.size()); // tf message ရှိ transformation အရေအတွက်ကို သိမ်းခြင်း။
                    tf_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            diag_sub = ros_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", 10,
                [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { // diagnostics topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_diag = QString("Diagnostics: %1 status").arg(msg->status.size()); // diagnostic status အရေအတွက်ကို သိမ်းခြင်း။
                    diag_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            laser_echo_sub = ros_node->create_subscription<sensor_msgs::msg::LaserEcho>(
                "/echo", 10,
                [this](sensor_msgs::msg::LaserEcho::SharedPtr msg) { // laser_echo topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_laser_echo = QString("Echo: %1").arg(msg->echoes.size()); // laser echoes အရေအတွက်ကို သိမ်းခြင်း။
                    laser_echo_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            fluid_pressure_sub = ros_node->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/fluid_pressure", 10,
                [this](sensor_msgs::msg::FluidPressure::SharedPtr msg) { // fluid_pressure topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_fluid_pressure = QString("Pressure: %1").arg(msg->fluid_pressure); // fluid pressure data ကို သိမ်းခြင်း။
                    fluid_pressure_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            magnetic_field_sub = ros_node->create_subscription<sensor_msgs::msg::MagneticField>(
                "/magnetic_field", 10,
                [this](sensor_msgs::msg::MagneticField::SharedPtr msg) { // magnetic_field topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_magnetic_field = QString("Magnetic: x=%1, y=%2, z=%3") // magnetic field data ကို သိမ်းခြင်း။
                        .arg(msg->magnetic_field.x).arg(msg->magnetic_field.y).arg(msg->magnetic_field.z);
                    magnetic_field_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            navsatfix_sub = ros_node->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix", 10,
                [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) { // gps/fix topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_navsatfix = QString("Lat: %1, Lon: %2").arg(msg->latitude).arg(msg->longitude); // latitude နှင့် longitude ကို သိမ်းခြင်း။
                    navsatfix_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            temperature_sub = ros_node->create_subscription<sensor_msgs::msg::Temperature>(
                "/temperature", 10,
                [this](sensor_msgs::msg::Temperature::SharedPtr msg) { // temperature topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_temperature = QString("Temperature: %1").arg(msg->temperature); // temperature data ကို သိမ်းခြင်း။
                    temperature_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            humidity_sub = ros_node->create_subscription<sensor_msgs::msg::RelativeHumidity>(
                "/humidity", 10,
                [this](sensor_msgs::msg::RelativeHumidity::SharedPtr msg) { // humidity topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_humidity = QString("Humidity: %1").arg(msg->relative_humidity); // humidity data ကို သိမ်းခြင်း။
                    humidity_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            range_sub = ros_node->create_subscription<sensor_msgs::msg::Range>(
                "/range", 10,
                [this](sensor_msgs::msg::Range::SharedPtr msg) { // range topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_range = QString("Range: %1").arg(msg->range); // range data ကို သိမ်းခြင်း။
                    range_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            joy_sub = ros_node->create_subscription<sensor_msgs::msg::Joy>(
                "/joy", 10,
                [this](sensor_msgs::msg::Joy::SharedPtr msg) { // joy topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_joy = QString("Axes: %1, Buttons: %2").arg(msg->axes.size()).arg(msg->buttons.size()); // joystick axes နှင့် buttons အရေအတွက်ကို သိမ်းခြင်း။
                    joy_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            color_sub = ros_node->create_subscription<turtlesim::msg::Color>(
                "/turtle1/color_sensor", 10,
                [this](turtlesim::msg::Color::SharedPtr msg) { // color topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_color = QString("R: %1, G: %2, B: %3").arg(msg->r).arg(msg->g).arg(msg->b); // color RGB တန်ဖိုးများကို သိမ်းခြင်း။
                    color_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            nav_path_sub = ros_node->create_subscription<nav_msgs::msg::Path>(
                "/plan", 10,
                [this](nav_msgs::msg::Path::SharedPtr msg) { // plan topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_nav_path = QString("Path points: %1").arg(msg->poses.size()); // လမ်းကြောင်းရှိ point အရေအတွက်ကို သိမ်းခြင်း။
                    nav_path_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            odom_filtered_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered", 10,
                [this](nav_msgs::msg::Odometry::SharedPtr msg) { // odometry/filtered topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_odom_filtered = QString("x: %1, y: %2") // filtered odometry position data ကို သိမ်းခြင်း။
                        .arg(msg->pose.pose.position.x)
                        .arg(msg->pose.pose.position.y);
                    odom_filtered_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            pointcloud_sub = ros_node->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/points_raw", 10,
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { // points_raw topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_pointcloud = QString("Width: %1, Height: %2").arg(msg->width).arg(msg->height); // point cloud width နှင့် height ကို သိမ်းခြင်း။
                    pointcloud_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });
            battery_voltage_sub = ros_node->create_subscription<sensor_msgs::msg::BatteryState>(
                "/battery/voltage", 10,
                [this](sensor_msgs::msg::BatteryState::SharedPtr msg) { // battery/voltage topic မှ message လက်ခံရရှိသောအခါ ခေါ်မည့် callback။
                    last_battery_voltage = QString("Voltage: %1").arg(msg->voltage); // battery voltage ကို သိမ်းခြင်း။
                    battery_voltage_received = true; // data လက်ခံရရှိကြောင်း flag ကို true ပေးခြင်း။
                });

            ros_ok = true; // ROS2 အစပျိုးခြင်း အောင်မြင်ပါက ros_ok ကို true ပေးခြင်း။
            rclcpp::spin(ros_node); // ROS2 node ကို spin လုပ်ခြင်း (callback များကို လုပ်ဆောင်ရန်)။
            rclcpp::shutdown(); // ROS2 ကို ပိတ်သိမ်းခြင်း။
        } catch (const std::exception &e) { // Exception ဖမ်းမိပါက။
            lastError = e.what(); // error message ကို သိမ်းခြင်း။
            ros_ok = false; // ros_ok ကို false ပေးခြင်း။
        } catch (...) { // အခြား မသိသော error ဖမ်းမိပါက။
            lastError = "Unknown error"; // "Unknown error" message ကို သိမ်းခြင်း။
            ros_ok = false; // ros_ok ကို false ပေးခြင်း။
        }
    });

    // Timer to check connection and update button color
    timer = new QTimer(this); // QTimer object အသစ် ဖန်တီးခြင်း။
    connect(timer, &QTimer::timeout, this, &MainWindow::checkROS2Connection); // timer timeout ဖြစ်တိုင်း checkROS2Connection slot ကို ခေါ်ရန် ချိတ်ဆက်ခြင်း။
    timer->start(1000); // timer ကို 1000 ms (1 second) တိုင်း timeout ဖြစ်စေရန် စတင်ခြင်း။
}

// MainWindow class ရဲ့ destructor ကို ကြေညာခြင်း။
MainWindow::~MainWindow()
{
    if (ros_node) // ros_node ရှိနေသေးပါက။
        ros_node->get_node_base_interface()->get_context()->shutdown("GUI closed"); // ROS2 node ကို ပိတ်သိမ်းရန် အမိန့်ပေးခြင်း။
    if (ros_thread.joinable()) // ros_thread ကို join လုပ်လို့ရပါက။
        ros_thread.join(); // ros_thread ပြီးဆုံးအောင် စောင့်ခြင်း။
    delete ui; // UI object ကို delete လုပ်ခြင်း။
}

// ROS2 connection ရှိမရှိ စစ်ဆေးပြီး button အရောင်များကို update လုပ်သည့် function။
void MainWindow::checkROS2Connection()
{
    for (auto btn : allButtons) { // button တစ်ခုချင်းစီကို စစ်ဆေးခြင်း။
        QString color = "grey"; // default အရောင်ကို မီးခိုးရောင် ပေးခြင်း။
        if (!topicInfoMap.contains(btn->objectName())) { // button ၏ objectName ကို topicInfoMap တွင် မတွေ့ပါက။
            btn->setStyleSheet("background-color: grey;"); // နောက်ခံအရောင်ကို မီးခိုးရောင် ပေးခြင်း။
            continue; // နောက် button ကို ဆက်သွားခြင်း။
        }
        const TopicInfo &info = topicInfoMap[btn->objectName()]; // သက်ဆိုင်ရာ TopicInfo ကို ရယူခြင်း။
        int publisher_count = 0; // publisher အရေအတွက်ကို 0 အဖြစ် အစပျိုးခြင်း။
        if (ros_ok && rclcpp::ok()) // ROS2 အလုပ်လုပ်နေပြီး rclcpp လည်း ok ဖြစ်ပါက။
            publisher_count = ros_node->get_publishers_info_by_topic(info.topic_name.toStdString()).size(); // သက်ဆိုင်ရာ topic ၏ publisher အရေအတွက်ကို ရယူခြင်း။

        if (!ros_ok || !rclcpp::ok()) { // ROS2 အလုပ်မလုပ်သေးပါက သို့မဟုတ် rclcpp ok မဖြစ်ပါက။
            color = "red"; // အရောင်ကို အနီရောင် ပေးခြင်း (Error/Disconnected)။
        } else if (publisher_count == 0) { // publisher မရှိပါက။
            color = "grey"; // အရောင်ကို မီးခိုးရောင် ပေးခြင်း (No publisher)။
        } else {
            // Publisher ရှိပြီး data လက်ခံရရှိခြင်းရှိမရှိ စစ်ဆေးခြင်း။
            if (btn->objectName() == "cmd_velButton") // cmd_velButton ဖြစ်ပါက။
                color = cmd_vel_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "scanButton") // scanButton ဖြစ်ပါက။
                color = scan_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "turtlePoseButton") // turtlePoseButton ဖြစ်ပါက။
                color = turtle_pose_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "odomButton") // odomButton ဖြစ်ပါက။
                color = odom_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "jointStatesButton") // jointStatesButton ဖြစ်ပါက။
                color = joint_states_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "imageButton") // imageButton ဖြစ်ပါက။
                color = image_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "batteryButton") // batteryButton ဖြစ်ပါက။
                color = battery_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "imuButton") // imuButton ဖြစ်ပါက။
                color = imu_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "mapButton") // mapButton ဖြစ်ပါက။
                color = map_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "goalPoseButton") // goalPoseButton ဖြစ်ပါက။
                color = goal_pose_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "tfButton") // tfButton ဖြစ်ပါက။
                color = tf_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "diagButton") // diagButton ဖြစ်ပါက။
                color = diag_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "laserEchoButton") // laserEchoButton ဖြစ်ပါက။
                color = laser_echo_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "fluidPressureButton") // fluidPressureButton ဖြစ်ပါက။
                color = fluid_pressure_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "magneticFieldButton") // magneticFieldButton ဖြစ်ပါက။
                color = magnetic_field_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "navSatFixButton") // navSatFixButton ဖြစ်ပါက။
                color = navsatfix_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "temperatureButton") // temperatureButton ဖြစ်ပါက။
                color = temperature_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "relativeHumidityButton") // relativeHumidityButton ဖြစ်ပါက။
                color = humidity_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "rangeButton") // rangeButton ဖြစ်ပါက။
                color = range_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "joyButton") // joyButton ဖြစ်ပါက။
                color = joy_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "colorButton") // colorButton ဖြစ်ပါက။
                color = color_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "navPathButton") // navPathButton ဖြစ်ပါက။
                color = nav_path_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "odomFilteredButton") // odomFilteredButton ဖြစ်ပါက။
                color = odom_filtered_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "pointCloudButton") // pointCloudButton ဖြစ်ပါက။
                color = pointcloud_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else if (btn->objectName() == "batteryVoltageButton") // batteryVoltageButton ဖြစ်ပါက။
                color = battery_voltage_received ? "green" : "blue"; // data ရပြီဆို green၊ မရသေးရင် blue ပေးခြင်း။
            else
                color = "blue"; // အခြား button များအတွက် blue ပေးခြင်း။
        }
        btn->setStyleSheet(QString("background-color: %1;").arg(color)); // Button ၏ နောက်ခံအရောင်ကို သတ်မှတ်ခြင်း။
    }
}

// မည်သည့် button ကိုမဆို နှိပ်လိုက်သောအခါ ခေါ်မည့် slot function။
void MainWindow::onAnyButtonClicked()
{
    QPushButton *btn = qobject_cast<QPushButton *>(sender()); // နှိပ်လိုက်သော button ကို ရယူခြင်း။
    if (!btn) // button မရှိပါက။
        return; // function ကနေ ပြန်ထွက်ခြင်း။
    showProcessMessage(btn->objectName()); // button ရဲ့ objectName နဲ့ showProcessMessage function ကို ခေါ်ခြင်း။
}

// Topic အချက်အလက်နှင့် လက်ရှိ status ကို Message Box ဖြင့် ပြသသော function။
void MainWindow::showProcessMessage(const QString &buttonName)
{
    if (!topicInfoMap.contains(buttonName)) { // buttonName ကို topicInfoMap တွင် မတွေ့ပါက။
        QMessageBox::information(this, "ROS 2 Topic Info", "Unknown button/topic."); // မသိသော button/topic ဖြစ်ကြောင်း message ပြသခြင်း။
        return; // function ကနေ ပြန်ထွက်ခြင်း။
    }
    const TopicInfo &info = topicInfoMap[buttonName]; // သက်ဆိုင်ရာ TopicInfo ကို ရယူခြင်း။

    QString status; // status string ကို ကြေညာခြင်း။
    QString data; // data string ကို ကြေညာခြင်း။
    int publisher_count = ros_node->get_publishers_info_by_topic(info.topic_name.toStdString()).size(); // သက်ဆိုင်ရာ topic ၏ publisher အရေအတွက်ကို ရယူခြင်း။

    // Status and data logic
    if (!ros_ok || !rclcpp::ok()) { // ROS2 အလုပ်မလုပ်သေးပါက သို့မဟုတ် rclcpp ok မဖြစ်ပါက။
        status = "Error: ROS 2 is not running!"; // Error message ကို သတ်မှတ်ခြင်း။
    } else if (publisher_count == 0) { // publisher မရှိပါက။
        status = "No publisher detected for this topic."; // Publisher မရှိကြောင်း status ကို သတ်မှတ်ခြင်း။
    } else {
        // Button နာမည်အလိုက် သက်ဆိုင်ရာ data နှင့် status ကို သတ်မှတ်ခြင်း။
        if (buttonName == "cmd_velButton") {
            status = cmd_vel_received ? "Receiving data." : "Waiting for data...";
            data = last_cmd_vel;
        } else if (buttonName == "scanButton") {
            status = scan_received ? "Receiving data." : "Waiting for data...";
            data = last_scan;
        } else if (buttonName == "turtlePoseButton") {
            status = turtle_pose_received ? "Receiving data." : "Waiting for data...";
            data = last_turtle_pose;
        } else if (buttonName == "odomButton") {
            status = odom_received ? "Receiving data." : "Waiting for data...";
            data = last_odom;
        } else if (buttonName == "jointStatesButton") {
            status = joint_states_received ? "Receiving data." : "Waiting for data...";
            data = last_joint_states;
        } else if (buttonName == "imageButton") {
            status = image_received ? "Receiving data." : "Waiting for data...";
            data = last_image;
        } else if (buttonName == "batteryButton") {
            status = battery_received ? "Receiving data." : "Waiting for data...";
            data = last_battery;
        } else if (buttonName == "imuButton") {
            status = imu_received ? "Receiving data." : "Waiting for data...";
            data = last_imu;
        } else if (buttonName == "mapButton") {
            status = map_received ? "Receiving data." : "Waiting for data...";
            data = last_map;
        } else if (buttonName == "goalPoseButton") {
            status = goal_pose_received ? "Receiving data." : "Waiting for data...";
            data = last_goal_pose;
        } else if (buttonName == "tfButton") {
            status = tf_received ? "Receiving data." : "Waiting for data...";
            data = last_tf;
        } else if (buttonName == "diagButton") {
            status = diag_received ? "Receiving data." : "Waiting for data...";
            data = last_diag;
        } else if (buttonName == "laserEchoButton") {
            status = laser_echo_received ? "Receiving data." : "Waiting for data...";
            data = last_laser_echo;
        } else if (buttonName == "fluidPressureButton") {
            status = fluid_pressure_received ? "Receiving data." : "Waiting for data...";
            data = last_fluid_pressure;
        } else if (buttonName == "magneticFieldButton") {
            status = magnetic_field_received ? "Receiving data." : "Waiting for data...";
            data = last_magnetic_field;
        } else if (buttonName == "navSatFixButton") {
            status = navsatfix_received ? "Receiving data." : "Waiting for data...";
            data = last_navsatfix;
        } else if (buttonName == "temperatureButton") {
            status = temperature_received ? "Receiving data." : "Waiting for data...";
            data = last_temperature;
        } else if (buttonName == "relativeHumidityButton") {
            status = humidity_received ? "Receiving data." : "Waiting for data...";
            data = last_humidity;
        } else if (buttonName == "rangeButton") {
            status = range_received ? "Receiving data." : "Waiting for data...";
            data = last_range;
        } else if (buttonName == "joyButton") {
            status = joy_received ? "Receiving data." : "Waiting for data...";
            data = last_joy;
        } else if (buttonName == "colorButton") {
            status = color_received ? "Receiving data." : "Waiting for data...";
            data = last_color;
        } else if (buttonName == "navPathButton") {
            status = nav_path_received ? "Receiving data." : "Waiting for data...";
            data = last_nav_path;
        } else if (buttonName == "odomFilteredButton") {
            status = odom_filtered_received ? "Receiving data." : "Waiting for data...";
            data = last_odom_filtered;
        } else if (buttonName == "pointCloudButton") {
            status = pointcloud_received ? "Receiving data." : "Waiting for data...";
            data = last_pointcloud;
        } else if (buttonName == "batteryVoltageButton") {
            status = battery_voltage_received ? "Receiving data." : "Waiting for data...";
            data = last_battery_voltage;
        }
    }

    // Message box တွင် ပြသရန် string ကို format လုပ်ခြင်း။
    QString msg = QString(
        "<b>%1</b><br>" // User-friendly topic name ကို bold နဲ့ပြ။
        "Topic: <code>%2</code><br>" // ROS2 topic name ကို code block နဲ့ပြ။
        "Type: <code>%3</code><br>" // Message type ကို code block နဲ့ပြ။
        "Role: Subscriber<br>" // Role က Subscriber ဖြစ်ကြောင်းပြ။
        "Description: %4<br>" // Topic ဖော်ပြချက်။
        "Publishers: %5<br>" // လက်ရှိ publisher အရေအတွက်။
        "Status: %6" // Topic status (Receiving data, Waiting for data, No publisher, Error)။
    ).arg(info.user_name)
     .arg(info.topic_name)
     .arg(info.type)
     .arg(info.description)
     .arg(ros_node->get_publishers_info_by_topic(info.topic_name.toStdString()).size())
     .arg(status);

    if (!data.isEmpty()) // data ရှိပါက။
        msg += QString("<br><br><b>Latest Data:</b><br><pre>%1</pre>").arg(data); // နောက်ဆုံးရ data ကို bold နှင့် pre-formatted text အနေဖြင့် ထည့်ခြင်း။

    QMessageBox box(this); // QMessageBox object အသစ် ဖန်တီးခြင်း။
    box.setWindowTitle("ROS 2 Topic Info"); // Message Box ခေါင်းစဉ်ကို သတ်မှတ်ခြင်း။
    box.setTextFormat(Qt::RichText); // Text format ကို RichText (HTML) အနေနဲ့ သတ်မှတ်ခြင်း။
    box.setText(msg); // Message Box ရဲ့ text ကို သတ်မှတ်ခြင်း။
    box.exec(); // Message Box ကို ပြသခြင်း။
}
