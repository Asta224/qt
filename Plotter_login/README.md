# 📊 Serial Port Monitor Application 🚀

This is a Qt-based desktop application designed to monitor serial port data and visualize it in real-time, along with providing basic serial communication functionalities.

---

## ✨ Features ✨

* **User Authentication:** 🔒 Simple login system with predefined usernames and passwords for different access levels (topic, service, function).
* **Real-time Clock Display:** ⏰ Shows the current date and time on the main login window.
* **Styling:** 🎨 Custom stylesheets applied for a modern and visually appealing user interface.
* **Animated Login Form:** 💫 The login form appears with a bounce animation, adding a touch of magic!
* **Serial Port Communication:** 🔌
    * **Auto-detection of Serial Ports:** 🔍 Automatically lists available serial ports (e.g., COM ports on Windows, ttyUSB/ttyACM on Linux).
    * **Baud Rate Selection:** ⚡️ Allows selection of common baud rates (9600, 19200, 38400, 57600, 115200).
    * **Connect/Disconnect Functionality:** 🔄 Button to establish and terminate serial port connections with ease.
    * **Send Data:** ➡️ Send text messages over the serial port.
    * **Receive Data:** ⬅️ Display incoming serial data in a clear text log.
    * **Clear History:** 🗑️ Clear the received data log and plot with a single click.
* **Real-time Plotting:** 📈
    * **Live Data Visualization:** 👁️ Plots a single line graph of numerical data received from the serial port, updating in real-time.
    * **Time-based X-axis:** ⏱️ The X-axis represents elapsed time in seconds, giving you clear context.
    * **Scrollable Plot:** ↔️ Automatically scrolls the plot to show the latest data, keeping you up-to-date.
    * **Pause Plot:** ⏸️ Option to pause real-time plotting while still receiving data in the log.
    * **Zoom and Drag:** ➕➖🖐️ Interactive plot for effortless zooming and dragging to explore your data.
* **Navigation:** 🏠 "Home" button to return to the main login window from the serial service window.

---

## 🏗️ Project Structure 🏗️

        Plotter_login/
        ├── main.cpp                 # 🚀 Application Entry Point
        ├── mainwindow.h             # 🔒 Core Login UI & App Flow Declarations
        ├── mainwindow.cpp           # 🔑 Login UI Logic & Authentication
        ├── mainwindow.ui            # 🖼️ Qt Designer UI Definition for Login Screen
        ├── service.h                # ⚡ Serial Communication & Plotting Declarations
        ├── service.cpp              # 📊 Serial Communication & Plotting Logic
        ├── service.ui               # 📈 Qt Designer UI Definition for Serial Monitor
        ├── topic.h                  # ✨ Placeholder: Potential 'Topic' Module/Window (Expandable!)
        ├── function.h               # 💡 Placeholder: Potential 'Function' Module/Window (Extendable!)
        ├── serial_monitor.pro       # ⚙️ QMake Project Configuration File
        ├── resources/               # 📂 Directory for Static Assets
        │   └── logo1.jpg            # 🖼️ Application Logo/Background Image
        └── ThirdParty/              # 📦 External Libraries Go Here
        └── qcustomplot.h        # 📈 QCustomPlot: Powerful Plotting Widget Header
        └── qcustomplot.cpp      # 🔧 QCustomPlot: Powerful Plotting Widget Implementation
This clean, modular structure ensures clarity and maintainability. Each component has a defined role, making it easy to understand, extend, and debug the application. It's built for future growth! 🌱

---

## 🚀 Get Started 🚀

### 🔑 Log In

1.  **Run** the application.
2.  On the login screen, use one of these quick credentials:
    * **Username:** `topic` / **Password:** `topic`
    * **Username:** `service` / **Password:** `service` (This takes you directly to the **Serial Port Monitor**! 🤩)
    * **Username:** `function` / **Password:** `function`
3.  Click the **"Login"** button.

### 📈 Use the Serial Port Monitor

1.  After logging in with `service` credentials, the **Serial Port Monitor** window will appear.
2.  **Select Port:** 🔌 Choose your device's serial port from the `Serial Port` dropdown list. (No ports? Check your connection and drivers! 🧐)
3.  **Set Baud Rate:** ⚡️ Choose the `Baud Rate` that matches your device (9600 is default).
4.  **Connect/Disconnect:** 🔄 Hit **"Connect"** to open the port. It'll change to **"Disconnect"** when active. Click it again to close.
5.  **Send Data:** ➡️ Type your message in the `Send Message` box and click **"Send"**. You'll see it reflected in the `Received Data` area.
6.  **Watch Data Flow:** 📊 Incoming serial data populates the `Received Data` log. Numerical values will also be charted in real-time on the plot!
7.  **Reset:** 🗑️ The **"Clear History"** button wipes both the log and the plot clean.
8.  **Pause Plotting:** ⏸️ Check **"Pause Plot"** to freeze the graph while data continues to stream into the log. Uncheck to resume.
9.  **Explore Data:** 🖐️ Drag to pan and 🖱️ scroll to zoom within the plot for detailed analysis.
10. **Go Home:** 🏠 Click **"Home"** to close the serial monitor and return to the main login screen.

---

## 🛠️ Dependencies 🛠️

* **Qt 6 (or compatible version)**: The powerful framework powering this amazing GUI application.
* **QCustomPlot library**: 📈 An indispensable Qt plotting widget for all the fantastic real-time data visualization. Ensure this library is properly included in your Qt project!

---

## 🚀 Building and Running 🚀

1.  **Install Qt:** ⬇️ Download and install Qt (e.g., via Qt Maintenance Tool).
2.  **Include QCustomPlot:** ➕ Add `QCustomPlot` to your project. This typically involves adding its source files to your `.pro` file or using CMake.
    * **Crucial for Serial:** If using QMake, remember to add `QT += serialport` to your `.pro` file!
3.  **Open Project:** 📂 Open the `.pro` file (if using QMake) or `CMakeLists.txt` (if using CMake) in Qt Creator.
4.  **Build:** 🔨 Build the project.
5.  **Run:** ▶️ Execute the compiled application and let the data flow!

---

Feel free to explore the code, contribute, and adapt it to your specific serial communication needs! Happy plotting! ✨