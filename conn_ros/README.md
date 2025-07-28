<div align="center">
  <h1>🌟 ROS2 Qt Topic Monitor 🌟</h1>
  <p>A Real-time Graphical User Interface for ROS2 Topic Monitoring</p>
</div>

---

## 🚀 Overview

This project delivers a **powerful and intuitive Graphical User Interface (GUI)** application, meticulously crafted with **Qt5** and **C++**, designed to be your go-to **real-time monitor for diverse ROS2 topics**. It empowers users with the ability to swiftly inspect the operational status of various ROS2 topics, understand their message types and descriptions, and view the very latest data being published. The application features dynamic, color-coded buttons that provide instant visual feedback on connection status and data flow, making debugging and system oversight a breeze!

## ✨ Key Features

* ✅ **Extensive Topic Monitoring:** Subscribes to a comprehensive range of common ROS2 message types, including `geometry_msgs/Twist` (Robot Velocity), `sensor_msgs/LaserScan` (Laser Scan), `nav_msgs/Odometry` (Odometry), `sensor_msgs/Image` (Camera Images), `sensor_msgs/BatteryState` (Battery State), `sensor_msgs/Imu` (IMU Data), `nav_msgs/OccupancyGrid` (Maps), `sensor_msgs/NavSatFix` (GPS Fixes), and many more!
* 🖥️ **Intuitive User Interface:** Boasts a clean, responsive, and user-friendly Qt-based GUI, featuring dedicated, clearly labeled buttons for each monitored topic.
* 🚦 **Real-time Status Indicators:** Experience immediate visual cues with dynamically changing button colors:
    * 🟢 **Green:** Data is actively and consistently being received from the topic.
    * 🔵 **Blue:** The topic has an active publisher, but the application is currently awaiting new data.
    * ⚪ **Grey:** No publisher is detected for the topic, indicating it might not be active.
    * 🔴 **Red:** Indicates a critical state where ROS2 is either not running, disconnected, or an error has occurred.
* 📊 **Detailed Topic Information:** A single click on any topic button reveals a pop-up message box displaying:
    * User-friendly Topic Name
    * Full ROS2 Topic Name
    * Message Type
    * Detailed Description
    * Number of Active Publishers
    * Current Topic Status
    * **Latest Received Data** (if available), presented in a clear, pre-formatted block.
* ⚡ **Responsive Design:** ROS2 spinning and data processing are efficiently handled in a separate thread, ensuring the GUI remains fluid and responsive without freezing.

## 🛠️ Dependencies

This project is built upon the robust foundations of the following libraries and frameworks:

* **ROS2 Humble (or compatible distribution):**
    * `rclcpp` (ROS2 Client Library for C++)
    * `std_msgs`
    * `geometry_msgs`
    * `sensor_msgs`
    * `nav_msgs`
    * `tf2_msgs`
    * `diagnostic_msgs`
    * `turtlesim` (for demonstration purposes like `turtle_pose`)
* **Qt5:**
    * `QtWidgets`
* **C++ Standard Libraries:**
    * `std::thread`
    * `std::memory`

## 📂 Project Structure

    conn_ros/
    ├── CMakeLists.txt         # CMake build configuration for the project
    ├── main.cpp               # Entry point for the Qt application
    ├── mainwindow.h           # Header file for the MainWindow class (declarations)
    ├── mainwindow.cpp         # Source file for the MainWindow class (implementations)
    ├── mainwindow.ui          # Qt Designer UI file for the main window layout
    └── image.qrc              # Qt Resource file for images (e.g., application logo   


## 🚀 Building and Running

### Prerequisites

Before you can build and run this application, ensure you have the following installed and configured:

1.  **ROS2 Humble Installation:** Make sure ROS2 Humble is correctly installed on your system and its environment variables are sourced.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
2.  **Qt5 Development Libraries:** Install the necessary Qt5 development packages.
    On Ubuntu/Debian-based systems, you can install them using:
    ```bash
    sudo apt install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
    ```
3.  **ROS2 Qt Bindings (if required by your setup):** This project leverages `rclcpp` directly for ROS2 communication. Standard ROS2 setup should generally suffice, but ensure your environment supports Qt-ROS2 integration if you encounter linking issues.

### Build Steps

1.  **Clone the repository:** Begin by cloning this project to your local machine:
    ```bash
    git clone <your-repository-url>
    cd <your-repository-name>
    ```
2.  **Build with Colcon (Recommended for ROS2 Workspaces):**
    Navigate to your ROS2 workspace root (e.g., `~/ros2_ws`) and build using `colcon`:
    ```bash
    colcon build --packages-select conn_ros
    ```
    *Alternatively, for a standalone Qt project build (less common in a ROS2 context):*
    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

### Running the Application

After a successful compilation, follow these steps:

1.  **Source your ROS2 workspace:**
    ```bash
    source install/setup.bash # If built with colcon
    # Or if built with CMake/Make:
    # export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/your/build/directory
    # ./conn_ros # Or the specific name of your executable
    ```
2.  **Launch the application:**
    ```bash
    ros2 run conn_ros conn_ros
    ```

To observe meaningful data on the topic buttons, ensure you have active ROS2 nodes publishing to the respective topics. For instance, to see `Robot Velocity` or `Turtle Pose` data being updated:

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

>⚙️ Customization

This project is designed with extensibility in mind!

    
  Adding New Topics: Want to monitor more ROS2 topics? Here's how:
* Include the necessary message type header in `mainwindow.h` (e.g., `#include <your_package/msg/YourMessage.hpp>`).
* Declare a new `rclcpp::Subscription` and a `QString` variable (e.g., `QString last_your_topic_data;`) along with a boolean flag (e.g., `bool your_topic_received = false;`) in `mainwindow.h`.
* Add a new entry to `topicInfoMap` in `mainwindow.cpp` following the existing pattern.
* Create a new `create_subscription` call within the `MainWindow` constructor in `mainwindow.cpp`, implementing a lambda callback to update your `last_` variable and `_received` flag.
* Add a new `QPushButton` in your `mainwindow.ui` (using Qt Designer) and ensure its `objectName` precisely matches the key you used in `topicInfoMap`.
* Update the `checkROS2Connection` and `showProcessMessage` functions in `mainwindow.cpp` to correctly handle the new topic's status and data display logic.
* If your new topic requires new ROS2 package dependencies, update your `CMakeLists.txt` accordingly.

UI Styling: Personalize the application's aesthetics by modifying the CSS (Cascading Style Sheets) directly within the `MainWindow::MainWindow` constructor in `mainwindow.cpp`.

Error Handling: The application currently logs errors to a `lastError` variable. For more robust applications, consider implementing advanced logging mechanisms or a dedicated error display interface.

>🤝 Contributing

Contributions are warmly welcomed! If you have suggestions, bug reports, or wish to enhance the project, please feel free to:

    🍴 Fork this repository.
    🐛 Submit issues.
    ➡️ Create pull requests.
Your input makes this project better!