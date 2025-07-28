# 📈 QtPlotter: Integrate Powerful Plotting into Your Qt Projects

<p align="center">
  A simple and modern guide to incorporating QCustomPlot for stunning visualizations in your C++ Qt applications.
</p>

---

## 🌟 Overview

**QtPlotter** provides a straightforward way to integrate `QCustomPlot`, a high-performance Qt C++ plotting library, into your projects. Whether you need to visualize real-time data, create scientific graphs, or add dynamic charts to your application, QtPlotter makes it easy to get started with powerful plotting capabilities.

This guide will walk you through setting up `QCustomPlot` in your existing Qt C++ projects, ensuring a smooth and efficient integration process.

## 🚀 Key Features of QCustomPlot

`QCustomPlot` is renowned for its:

* **High Performance:** Optimized for real-time plotting of large datasets.
* **Rich Feature Set:** Supports various plot types (line, scatter, bar, etc.), multiple axes, legends, annotations, and more.
* **Interactive Capabilities:** Enables zooming, dragging, selection, and custom interactions.
* **Customizable Appearance:** Extensive options for styling plots, axes, lines, and data points.
* **Export Options:** Easily save plots to various image formats (PNG, JPG, BMP) and PDF.

## 🛠️ Prerequisites

Before you begin, ensure you have:

* **Qt Framework:** Installed on your system (Qt5 or Qt6).
* **C++ Compiler:** A C++11 compatible compiler (e.g., GCC, Clang, MSVC).
* **CMake:** Version 3.16 or higher (as indicated in `CMakeLists.txt`).

## 📂 Project Structure

To use QtPlotter, you'll typically place the `qcustomplot.h` and `qcustomplot.cpp` files directly into your project's source directory, or manage them as a static library.
    
    YourQtProject/
    ├── CMakeLists.txt         # Your project's CMake build configuration
    ├── main.cpp               # Your application's main source file
    ├── mainwindow.h           # Header for your main window (if using GUI)
    ├── mainwindow.cpp         # Source for your main window
    ├── qcustomplot.h          # QCustomPlot header file
    └── qcustomplot.cpp        # QCustomPlot source file

## 🏗️ How to Integrate QtPlotter into Your Project

Follow these steps to add `QCustomPlot` to your Qt C++ project using CMake:

### Step 1: Add QCustomPlot Files to Your Project

Place `qcustomplot.h` and `qcustomplot.cpp` into your project's source directory (e.g., alongside `main.cpp`, `mainwindow.h`, `mainwindow.cpp`).

### Step 2: Modify Your `CMakeLists.txt`

You need to tell CMake about the `qcustomplot` source files and link against the necessary Qt modules.

Here's how your `CMakeLists.txt` should look, based on the provided `CMakeLists.txt` for `QtPlotter`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(YourProjectName LANGUAGES CXX) # Replace YourProjectName with your actual project name

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required Qt modules, including Widgets and PrintSupport
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets PrintSupport REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets PrintSupport REQUIRED)

# Add QCustomPlot as a static library (recommended)
add_library(QtPlotter STATIC
    qcustomplot.cpp
)

# Specify include directories for QtPlotter
target_include_directories(QtPlotter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link QtPlotter to Qt modules
target_link_libraries(QtPlotter PUBLIC Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::PrintSupport)

# Add your main executable
add_executable(YourExecutableName
    main.cpp
    mainwindow.cpp
    mainwindow.h
    # Add any other source/header files for your project
)

# Link your executable to QtPlotter and other Qt modules
target_link_libraries(YourExecutableName PRIVATE QtPlotter Qt${QT_VERSION_MAJOR}::Widgets)

# Optional: If you want to enable OpenGL for QCustomPlot (for performance)
# You need to define QCUSTOMPLOT_USE_OPENGL before including qcustomplot.h
# For CMake, you can add this:
# target_compile_definitions(YourExecutableName PRIVATE QCUSTOMPLOT_USE_OPENGL)
# If using Qt5, you might also need to find_package(Qt5 COMPONENTS OpenGL REQUIRED) and link to Qt5::OpenGL
```
Explanation of Changes:

* `add_library(QtPlotter STATIC ...)`: This creates a static library named QtPlotter from qcustomplot.cpp. This is a good practice as it encapsulates QCustomPlot and makes your main executable cleaner.
* `target_include_directories(QtPlotter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})`: Ensures that the compiler can find qcustomplot.h.
* `target_link_libraries(QtPlotter PUBLIC ...)`: Links the QtPlotter static library to the necessary Qt modules (Widgets, PrintSupport).
* `add_executable(YourExecutableName ...)`: Defines your main application executable.
* `target_link_libraries(YourExecutableName PRIVATE QtPlotter Qt${QT_VERSION_MAJOR}::Widgets)`: Crucially, this links your main executable to the QtPlotter static library, making QCustomPlot available to your application.

### Step 3: Integrate QCustomPlot into Your UI (Optional, but common)

If you're using Qt Designer (`.ui` files):

* **Promote a QWidget:** In your `mainwindow.ui` (or any other `.ui` file where you want the plot), drag a `QWidget` onto your form.
* **Right-click and "Promote to..."**:
    * **Promoted class name:** `QCustomPlot`
    * **Header file:** `qcustomplot.h`
* Click "Add" and then "Promote". This will allow you to use QCustomPlot directly in your UI.

### Step 4: Write Plotting Code in Your C++ Files

Now you can use QCustomPlot in your `mainwindow.h` and `mainwindow.cpp` (or other C++ files).

Example in mainwindow.h:

    C++

    #ifndef MAINWINDOW_H
    #define MAINWINDOW_H

    #include <QMainWindow>
    #include "qcustomplot.h" // Include the QCustomPlot header

    QT_BEGIN_NAMESPACE
    namespace Ui { class MainWindow; }
    QT_END_NAMESPACE

    class MainWindow : public QMainWindow
    {
        Q_OBJECT

    public:
        MainWindow(QWidget *parent = nullptr);
        ~MainWindow();

    private:
        Ui::MainWindow *ui;
        QCustomPlot *customPlot; // Declare a pointer to QCustomPlot
    };
    #endif // MAINWINDOW_H

Example in mainwindow.cpp:
```
C++

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Assuming you promoted a QWidget named 'plotWidget' in your .ui file to QCustomPlot
    customPlot = ui->plotWidget; // Assign the promoted widget

    // Basic QCustomPlot setup:
    customPlot->addGraph(); // Add a graph to the plot
    customPlot->graph(0)->setPen(QPen(Qt::blue)); // Set graph line color

    // Generate some data
    QVector<double> x(101), y(101); // x and y will be plotted in 101 points
    for (int i=0; i<101; ++i)
    {
      x[i] = i/50.0 - 1; // x goes from -1 to 1
      y[i] = x[i]*x[i];  // y = x^2
    }
    customPlot->graph(0)->setData(x, y); // Set data for the graph

    // Set axis ranges
    customPlot->xAxis->setLabel("X Axis");
    customPlot->yAxis->setLabel("Y Axis");
    customPlot->xAxis->setRange(-1.1, 1.1);
    customPlot->yAxis->setRange(0, 1.1);

    // Make plot interactive (zoom and drag)
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    customPlot->replot(); // Replot to show the changes
}

MainWindow::~MainWindow()
{
    delete ui;
}
```

## 🚀 Building Your Project
Navigate to your build directory (e.g., build/ inside your project root) and run:
```
Bash

cmake ..
make
```
This will compile your project, linking it with the QtPlotter static library.

## 💡 Tips for Modern Usage

* **Smart Pointers:** Consider using `QSharedPointer` or `std::unique_ptr` for managing `QCustomPlot` objects and other dynamically allocated resources to prevent memory leaks.
* **Layering System:** Explore `QCustomPlot`'s powerful layering system (`QCPLayer`) to control the drawing order of different plot elements.
* **Interactions:** Leverage `QCP::Interaction` flags to enable various user interactions like dragging, zooming, and selecting elements.
* **Performance Hints:** For very demanding plots, investigate `QCP::PlottingHint` and `QCustomPlot::setOpenGl` for potential performance improvements.
* **Error Handling:** Always check return values of functions like `addGraph()` to ensure objects are created successfully. Use `qDebug()` for logging.


