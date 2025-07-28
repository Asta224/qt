# ROS2 Qt Monitor

This project is a Qt-based GUI application designed to monitor various ROS2 topics in real-time. It provides a user-friendly interface to visualize the status of different sensors, robot states, and navigation data.

## Features

* **Real-time Topic Monitoring:** Displays the status of 25 different ROS2 topics.
* **Visual Feedback:** Buttons change color based on the ROS2 connection status and whether data is being received for a specific topic (Red: ROS2 not running, Grey: No publisher, Blue: Publisher present but no data received yet, Green: Data actively being received).
* **Detailed Topic Information:** Clicking on a button displays a message box with comprehensive details about the topic, including its name, type, role (subscriber), description, number of active publishers, current status, and the latest received data.
* **Minimalist UI:** A clean and simple user interface built with Qt widgets and custom CSS for a modern look.

## Prerequisites

Before building and running this application, ensure you have the following installed:

* **ROS2 Humble (or compatible distribution):** This project is built against ROS2 Humble.
* **Qt5 Development Libraries:** Specifically, `Qt5Widgets`.
* **ament_cmake:** ROS2 build system.
* **ROS2 Message Types:** The project depends on various ROS2 message packages. These are listed in `CMakeLists.txt` and include:
    * `rclcpp`
    * `std_msgs`
    * `geometry_msgs`
    * `sensor_msgs`
    * `nav_msgs`
    * `turtlesim`
    * `tf2_msgs`
    * `diagnostic_msgs`

## Project Structure

* `CMakeLists.txt`: CMake build configuration for the project. It defines the executable, links libraries, and specifies ROS2 dependencies.
* `main.cpp`: The entry point of the Qt application. Initializes the Qt application and creates the `MainWindow`.
* `mainwindow.h`: The header file for the `MainWindow` class, declaring its members, including ROS2 subscribers, data storage, and UI elements.
* `mainwindow.cpp`: The implementation file for the `MainWindow` class. It sets up the UI, initializes ROS2 in a separate thread, creates subscribers for various topics, handles button clicks, and updates the UI based on ROS2 topic status.
* `mainwindow.ui`: The Qt Designer UI file defining the layout and widgets of the main window.
* `image.qrc`: Qt Resource file, likely containing the `logo2.jpg` image used in the UI.

## Building the Project

1.  **Source ROS2:**
    Ensure your ROS2 environment is sourced. For Humble, it would typically be:

    ```bash
    source /opt/ros/humble/setup.bash
    ```

    If you have a custom workspace, source your workspace's `install/setup.bash` instead.

2.  **Navigate to your workspace (if applicable) and create the package:**
    If you're building this within a ROS2 workspace, place the `conn_ros` directory inside your `src` folder.

3.  **Build using Colcon:**

    ```bash
    cd <your_ros2_workspace>
    colcon build --packages-select conn_ros
    ```

    If you are building it as a standalone CMake project, navigate to the project root and run:

    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

## Running the Application

After successfully building the project, you can run the executable.

1.  **Source your ROS2 workspace (if not already sourced):**

    ```bash
    source install/setup.bash # Or /opt/ros/humble/setup.bash if standalone
    ```

2.  **Run the executable:**

    ```bash
    ros2 run conn_ros conn_ros
    ```

    (Replace `conn_ros` with your executable name if it's different, as defined in `CMakeLists.txt`).

## Usage

Once the application starts:

* The buttons will initially be grey or red, indicating no publishers or no ROS2 connection.
* Start publishing data to the respective ROS2 topics (e.g., run `turtlesim_node` for `/turtle1/pose` and `/turtle1/color_sensor`).

The buttons will change color:

* **Red:** ROS2 is not running or disconnected.
* **Grey:** ROS2 is running, but no publisher is detected for that specific topic.
* **Blue:** A publisher is detected, but no data has been received yet.
* **Green:** Data is actively being received from the topic.

Click any button to view a detailed information box about that topic, including the latest received data.

## Extending the Project

To add more topics or functionalities:

### Update `mainwindow.h`:

* Include the new ROS2 message type header.
* Declare a new `std::shared_ptr<rclcpp::Subscription<...>>` for the new topic.
* Declare a `QString` variable to store the `last_` data and a `bool` flag to track if data has been `_received`.

### Update `mainwindow.ui`:

* Add a new `QPushButton` for your topic. Ensure its `objectName` is unique and descriptive (e.g., `myNewTopicButton`).

### Update `mainwindow.cpp`:

* Add an entry to the `topicInfoMap` with the button's `objectName` and the topic's details.
* Initialize the new `_received` flag to `false` in the `MainWindow` constructor.
* Create a new subscriber using `ros_node->create_subscription` in the ROS2 thread, providing the topic name, QoS, and a lambda function to update the `last_` data and set the `_received` flag to `true`.
* Add an `else if` condition in `checkROS2Connection()` to update the button's color based on the new topic's `_received` flag.
* Add an `else if` condition in `showProcessMessage()` to display the `last_` data for the new topic.

### Update `CMakeLists.txt`:

* Add `find_package()` for any new ROS2 message packages required.
* Add the new message package to `ament_target_dependencies()`.


>  ⚠️ Troubleshooting

>"Error: ROS 2 is not running!": Ensure you have sourced your ROS2 environment and that roscore (or ros2 daemon) is running.

>Buttons remain grey/blue: Verify that a ROS2 node is actively publishing to the topic you are trying to monitor. Use ros2 topic list and ros2 topic echo <topic_name> to confirm.

>Build errors: Check your CMakeLists.txt for correct package dependencies and ensure all required ROS2 and Qt libraries are installed.

>Runtime errors: Check the console output for any ROS2 or Qt-related error messages.