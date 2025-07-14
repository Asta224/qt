#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QProcess>
#include <QMap>
#include <QStringList>

// Topic info struct and map
struct TopicInfo {
    QString user_name;
    QString topic_name;
    QString type;
    QString description;
};
 
const QMap<QString, TopicInfo> topicInfoMap = {
    { "cmd_velButton",      { "Robot Velocity", "/cmd_vel", "geometry_msgs/msg/Twist", "Robot velocity command input" } },
    { "scanButton",         { "Laser Scan", "/scan", "sensor_msgs/msg/LaserScan", "Laser scanner data" } },
    { "turtlePoseButton",   { "Turtle Pose", "/turtle1/pose", "turtlesim/msg/Pose", "Turtlesim pose feedback" } },
    { "odomButton",         { "Odometry", "/odom", "nav_msgs/msg/Odometry", "Odometry feedback" } },
    { "jointStatesButton",  { "Joint States", "/joint_states", "sensor_msgs/msg/JointState", "Joint state feedback" } },
    { "imageButton",        { "Camera Image", "/camera/image_raw", "sensor_msgs/msg/Image", "Camera image stream" } },
    { "batteryButton",      { "Battery State", "/battery_state", "sensor_msgs/msg/BatteryState", "Battery status" } },
    { "imuButton",          { "IMU Data", "/imu/data", "sensor_msgs/msg/Imu", "IMU sensor data" } },
    { "mapButton",          { "Map", "/map", "nav_msgs/msg/OccupancyGrid", "Occupancy grid map" } },
    { "goalPoseButton",     { "Goal Pose", "/goal_pose", "geometry_msgs/msg/PoseStamped", "Navigation goal pose" } },
    { "tfButton",           { "TF", "/tf", "tf2_msgs/msg/TFMessage", "Transform tree" } },
    { "diagButton",         { "Diagnostics", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", "System diagnostics" } },
    { "laserEchoButton",    { "Laser Echo", "/echo", "sensor_msgs/msg/LaserEcho", "Laser echo data" } },
    { "fluidPressureButton",{ "Fluid Pressure", "/fluid_pressure", "sensor_msgs/msg/FluidPressure", "Fluid pressure sensor" } },
    { "magneticFieldButton",{ "Magnetic Field", "/magnetic_field", "sensor_msgs/msg/MagneticField", "Magnetic field sensor" } },
    { "navSatFixButton",    { "GPS Fix", "/gps/fix", "sensor_msgs/msg/NavSatFix", "GPS position fix" } },
    { "temperatureButton",  { "Temperature", "/temperature", "sensor_msgs/msg/Temperature", "Temperature sensor" } },
    { "relativeHumidityButton", { "Humidity", "/humidity", "sensor_msgs/msg/RelativeHumidity", "Humidity sensor" } },
    { "rangeButton",        { "Range", "/range", "sensor_msgs/msg/Range", "Range sensor" } },
    { "joyButton",          { "Joystick", "/joy", "sensor_msgs/msg/Joy", "Joystick/gamepad input" } },
    { "colorButton",        { "Turtle Color", "/turtle1/color_sensor", "turtlesim/msg/Color", "Turtlesim color sensor" } },
    { "navPathButton",      { "Path", "/plan", "nav_msgs/msg/Path", "Planned navigation path" } },
    { "odomFilteredButton", { "Filtered Odometry", "/odometry/filtered", "nav_msgs/msg/Odometry", "Filtered odometry (e.g. EKF)" } },
    { "pointCloudButton",   { "Point Cloud", "/points_raw", "sensor_msgs/msg/PointCloud2", "Raw point cloud data" } },
    { "batteryVoltageButton", { "Battery Voltage", "/battery/voltage", "sensor_msgs/msg/BatteryState", "Battery voltage" } }
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), ros_ok(false)
{
    ui->setupUi(this);

    // Minimalist theme (no animation)
    this->setStyleSheet(R"(
    QWidget {
        background: rgba(117, 238, 157, 1);
        font-family: 'Segoe UI', 'Arial', sans-serif;
        font-size: 13px;
    }
    QPushButton {
        background: #ffffff;
        border: 1px solid #dddddd;
        border-radius: 8px;
        padding: 8px 0;
        color: #222;
//        min-width: 90px;
//        min-height: 32px;
//        max-width: 160px;
//        max-height: 48px;
    }
    QPushButton:hover {
        background: #e6f0fa;
        border: 1.5px solid #7bb6f2;
        color: #222;
    }
    QPushButton:pressed {
        background: #d0e6fa;
        border: 1.5px solid #1976d2;
        color: #222;
    }
)");

    // Collect all buttons
    allButtons = findChildren<QPushButton *>();
    for (auto btn : allButtons)
        btn->setStyleSheet("background-color: grey;");
    for (auto btn : allButtons)
        connect(btn, &QPushButton::clicked, this, &MainWindow::onAnyButtonClicked);

    // Initialize received flags
    cmd_vel_received = scan_received = turtle_pose_received = odom_received = false;
    joint_states_received = image_received = battery_received = imu_received = false;
    map_received = goal_pose_received = tf_received = diag_received = false;
    laser_echo_received = fluid_pressure_received = magnetic_field_received = false;
    navsatfix_received = temperature_received = humidity_received = range_received = false;
    joy_received = color_received = nav_path_received = odom_filtered_received = false;
    pointcloud_received = battery_voltage_received = false;

    // Start ROS2 in a thread
    ros_thread = std::thread([this]()
    {
        try {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
            ros_node = std::make_shared<rclcpp::Node>("qt_ros2_monitor");

            cmd_vel_sub = ros_node->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10,
                [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                    last_cmd_vel = QString("linear.x: %1, angular.z: %2")
                        .arg(msg->linear.x)
                        .arg(msg->angular.z);
                    cmd_vel_received = true;
                });
            scan_sub = ros_node->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10,
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    last_scan = msg->ranges.empty() ? "No scan data." : QString("ranges[0]: %1").arg(msg->ranges[0]);
                    scan_received = true;
                });
            turtle_pose_sub = ros_node->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10,
                [this](turtlesim::msg::Pose::SharedPtr msg) {
                    last_turtle_pose = QString("x: %1, y: %2, theta: %3")
                        .arg(msg->x)
                        .arg(msg->y)
                        .arg(msg->theta);
                    turtle_pose_received = true;
                });
            odom_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                    last_odom = QString("x: %1, y: %2")
                        .arg(msg->pose.pose.position.x)
                        .arg(msg->pose.pose.position.y);
                    odom_received = true;
                });
            joint_states_sub = ros_node->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                    last_joint_states = QString("name[0]: %1, position[0]: %2")
                        .arg(msg->name.empty() ? "none" : QString::fromStdString(msg->name[0]))
                        .arg(msg->position.empty() ? 0.0 : msg->position[0]);
                    joint_states_received = true;
                });
            image_sub = ros_node->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10,
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    last_image = QString("width: %1, height: %2")
                        .arg(msg->width)
                        .arg(msg->height);
                    image_received = true;
                });
            battery_sub = ros_node->create_subscription<sensor_msgs::msg::BatteryState>(
                "/battery_state", 10,
                [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
                    last_battery = QString("voltage: %1, percentage: %2")
                        .arg(msg->voltage)
                        .arg(msg->percentage);
                    battery_received = true;
                });
            imu_sub = ros_node->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 10,
                [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                    last_imu = QString("Orientation: x=%1, y=%2, z=%3, w=%4")
                        .arg(msg->orientation.x).arg(msg->orientation.y)
                        .arg(msg->orientation.z).arg(msg->orientation.w);
                    imu_received = true;
                });
            map_sub = ros_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10,
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                    last_map = QString("Map: %1x%2").arg(msg->info.width).arg(msg->info.height);
                    map_received = true;
                });
            goal_pose_sub = ros_node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10,
                [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    last_goal_pose = QString("Goal: x=%1, y=%2")
                        .arg(msg->pose.position.x).arg(msg->pose.position.y);
                    goal_pose_received = true;
                });
            tf_sub = ros_node->create_subscription<tf2_msgs::msg::TFMessage>(
                "/tf", 10,
                [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
                    last_tf = QString("Transforms: %1").arg(msg->transforms.size());
                    tf_received = true;
                });
            diag_sub = ros_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", 10,
                [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
                    last_diag = QString("Diagnostics: %1 status").arg(msg->status.size());
                    diag_received = true;
                });
            laser_echo_sub = ros_node->create_subscription<sensor_msgs::msg::LaserEcho>(
                "/echo", 10,
                [this](sensor_msgs::msg::LaserEcho::SharedPtr msg) {
                    last_laser_echo = QString("Echo: %1").arg(msg->echoes.size());
                    laser_echo_received = true;
                });
            fluid_pressure_sub = ros_node->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/fluid_pressure", 10,
                [this](sensor_msgs::msg::FluidPressure::SharedPtr msg) {
                    last_fluid_pressure = QString("Pressure: %1").arg(msg->fluid_pressure);
                    fluid_pressure_received = true;
                });
            magnetic_field_sub = ros_node->create_subscription<sensor_msgs::msg::MagneticField>(
                "/magnetic_field", 10,
                [this](sensor_msgs::msg::MagneticField::SharedPtr msg) {
                    last_magnetic_field = QString("Magnetic: x=%1, y=%2, z=%3")
                        .arg(msg->magnetic_field.x).arg(msg->magnetic_field.y).arg(msg->magnetic_field.z);
                    magnetic_field_received = true;
                });
            navsatfix_sub = ros_node->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix", 10,
                [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                    last_navsatfix = QString("Lat: %1, Lon: %2").arg(msg->latitude).arg(msg->longitude);
                    navsatfix_received = true;
                });
            temperature_sub = ros_node->create_subscription<sensor_msgs::msg::Temperature>(
                "/temperature", 10,
                [this](sensor_msgs::msg::Temperature::SharedPtr msg) {
                    last_temperature = QString("Temperature: %1").arg(msg->temperature);
                    temperature_received = true;
                });
            humidity_sub = ros_node->create_subscription<sensor_msgs::msg::RelativeHumidity>(
                "/humidity", 10,
                [this](sensor_msgs::msg::RelativeHumidity::SharedPtr msg) {
                    last_humidity = QString("Humidity: %1").arg(msg->relative_humidity);
                    humidity_received = true;
                });
            range_sub = ros_node->create_subscription<sensor_msgs::msg::Range>(
                "/range", 10,
                [this](sensor_msgs::msg::Range::SharedPtr msg) {
                    last_range = QString("Range: %1").arg(msg->range);
                    range_received = true;
                });
            joy_sub = ros_node->create_subscription<sensor_msgs::msg::Joy>(
                "/joy", 10,
                [this](sensor_msgs::msg::Joy::SharedPtr msg) {
                    last_joy = QString("Axes: %1, Buttons: %2").arg(msg->axes.size()).arg(msg->buttons.size());
                    joy_received = true;
                });
            color_sub = ros_node->create_subscription<turtlesim::msg::Color>(
                "/turtle1/color_sensor", 10,
                [this](turtlesim::msg::Color::SharedPtr msg) {
                    last_color = QString("R: %1, G: %2, B: %3").arg(msg->r).arg(msg->g).arg(msg->b);
                    color_received = true;
                });
            nav_path_sub = ros_node->create_subscription<nav_msgs::msg::Path>(
                "/plan", 10,
                [this](nav_msgs::msg::Path::SharedPtr msg) {
                    last_nav_path = QString("Path points: %1").arg(msg->poses.size());
                    nav_path_received = true;
                });
            odom_filtered_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered", 10,
                [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                    last_odom_filtered = QString("x: %1, y: %2")
                        .arg(msg->pose.pose.position.x)
                        .arg(msg->pose.pose.position.y);
                    odom_filtered_received = true;
                });
            pointcloud_sub = ros_node->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/points_raw", 10,
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    last_pointcloud = QString("Width: %1, Height: %2").arg(msg->width).arg(msg->height);
                    pointcloud_received = true;
                });
            battery_voltage_sub = ros_node->create_subscription<sensor_msgs::msg::BatteryState>(
                "/battery/voltage", 10,
                [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
                    last_battery_voltage = QString("Voltage: %1").arg(msg->voltage);
                    battery_voltage_received = true;
                });

            ros_ok = true;
            rclcpp::spin(ros_node);
            rclcpp::shutdown();
        } catch (const std::exception &e) {
            lastError = e.what();
            ros_ok = false;
        } catch (...) {
            lastError = "Unknown error";
            ros_ok = false;
        }
    });

    // Timer to check connection and update button color
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::checkROS2Connection);
    timer->start(1000);
}

MainWindow::~MainWindow()
{
    if (ros_node)
        ros_node->get_node_base_interface()->get_context()->shutdown("GUI closed");
    if (ros_thread.joinable())
        ros_thread.join();
    delete ui;
}

void MainWindow::checkROS2Connection()
{
    for (auto btn : allButtons) {
        QString color = "grey";
        if (!topicInfoMap.contains(btn->objectName())) {
            btn->setStyleSheet("background-color: grey;");
            continue;
        }
        const TopicInfo &info = topicInfoMap[btn->objectName()];
        int publisher_count = 0;
        if (ros_ok && rclcpp::ok())
            publisher_count = ros_node->get_publishers_info_by_topic(info.topic_name.toStdString()).size();

        if (!ros_ok || !rclcpp::ok()) {
            color = "red";
        } else if (publisher_count == 0) {
            color = "grey";
        } else {
            // Publisher exists, check if data received
            if (btn->objectName() == "cmd_velButton")
                color = cmd_vel_received ? "green" : "blue";
            else if (btn->objectName() == "scanButton")
                color = scan_received ? "green" : "blue";
            else if (btn->objectName() == "turtlePoseButton")
                color = turtle_pose_received ? "green" : "blue";
            else if (btn->objectName() == "odomButton")
                color = odom_received ? "green" : "blue";
            else if (btn->objectName() == "jointStatesButton")
                color = joint_states_received ? "green" : "blue";
            else if (btn->objectName() == "imageButton")
                color = image_received ? "green" : "blue";
            else if (btn->objectName() == "batteryButton")
                color = battery_received ? "green" : "blue";
            else if (btn->objectName() == "imuButton")
                color = imu_received ? "green" : "blue";
            else if (btn->objectName() == "mapButton")
                color = map_received ? "green" : "blue";
            else if (btn->objectName() == "goalPoseButton")
                color = goal_pose_received ? "green" : "blue";
            else if (btn->objectName() == "tfButton")
                color = tf_received ? "green" : "blue";
            else if (btn->objectName() == "diagButton")
                color = diag_received ? "green" : "blue";
            else if (btn->objectName() == "laserEchoButton")
                color = laser_echo_received ? "green" : "blue";
            else if (btn->objectName() == "fluidPressureButton")
                color = fluid_pressure_received ? "green" : "blue";
            else if (btn->objectName() == "magneticFieldButton")
                color = magnetic_field_received ? "green" : "blue";
            else if (btn->objectName() == "navSatFixButton")
                color = navsatfix_received ? "green" : "blue";
            else if (btn->objectName() == "temperatureButton")
                color = temperature_received ? "green" : "blue";
            else if (btn->objectName() == "relativeHumidityButton")
                color = humidity_received ? "green" : "blue";
            else if (btn->objectName() == "rangeButton")
                color = range_received ? "green" : "blue";
            else if (btn->objectName() == "joyButton")
                color = joy_received ? "green" : "blue";
            else if (btn->objectName() == "colorButton")
                color = color_received ? "green" : "blue";
            else if (btn->objectName() == "navPathButton")
                color = nav_path_received ? "green" : "blue";
            else if (btn->objectName() == "odomFilteredButton")
                color = odom_filtered_received ? "green" : "blue";
            else if (btn->objectName() == "pointCloudButton")
                color = pointcloud_received ? "green" : "blue";
            else if (btn->objectName() == "batteryVoltageButton")
                color = battery_voltage_received ? "green" : "blue";
            else
                color = "blue";
        }
        btn->setStyleSheet(QString("background-color: %1;").arg(color));
    }
}

void MainWindow::onAnyButtonClicked()
{
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    if (!btn)
        return;
    showProcessMessage(btn->objectName());
}

void MainWindow::showProcessMessage(const QString &buttonName)
{
    if (!topicInfoMap.contains(buttonName)) {
        QMessageBox::information(this, "ROS 2 Topic Info", "Unknown button/topic.");
        return;
    }
    const TopicInfo &info = topicInfoMap[buttonName];

    QString status;
    QString data;
    int publisher_count = ros_node->get_publishers_info_by_topic(info.topic_name.toStdString()).size();

    // Status and data logic
    if (!ros_ok || !rclcpp::ok()) {
        status = "Error: ROS 2 is not running!";
    } else if (publisher_count == 0) {
        status = "No publisher detected for this topic.";
    } else {
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

    QString msg = QString(
        "<b>%1</b><br>"
        "Topic: <code>%2</code><br>"
        "Type: <code>%3</code><br>"
        "Role: Subscriber<br>"
        "Description: %4<br>"
        "Publishers: %5<br>"
        "Status: %6"
    ).arg(info.user_name)
     .arg(info.topic_name)
     .arg(info.type)
     .arg(info.description)
     .arg(ros_node->get_publishers_info_by_topic(info.topic_name.toStdString()).size())
     .arg(status);

    if (!data.isEmpty())
        msg += QString("<br><br><b>Latest Data:</b><br><pre>%1</pre>").arg(data);

    QMessageBox box(this);
    box.setWindowTitle("ROS 2 Topic Info");
    box.setTextFormat(Qt::RichText);
    box.setText(msg);
    box.exec();
}
