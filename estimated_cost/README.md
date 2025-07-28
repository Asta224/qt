# ✨ Robot Cost Estimator (Qt/C++) ✨

<p align="center">
  A meticulously crafted desktop GUI application to streamline your robot bill of materials and cost estimation!
</p>

---

## 📚 Table of Contents
* [🌟 Overview](#-overview)
* [🚀 Features](#-features)
* [🛠️ Prerequisites](#️-prerequisites)
* [📂 Project Structure](#-project-structure)
* [🏗️ Building the Project](#️-building-the-project)
* [▶️ Usage](#️-usage)
* [💡 Extending the Project](#-extending-the-project)
    * [➕ Adding/Updating Items via CSV](#-addingupdating-items-via-csv)
    * [🆕 Adding New Component Categories to the UI](#-adding-new-component-categories-to-the-ui)
* [⚠️ Troubleshooting](#️-troubleshooting)
* [🤝 Contributing](#-contributing)

---

## 🌟 Overview

Dive into the world of efficient robot design with the **Robot Cost Estimator**! This robust desktop GUI application, built with the power of **Qt** and **C++**, is your ultimate companion for estimating robot costs. It provides a dynamic and interactive platform where you can effortlessly select various robot components, specify quantities, and watch as the total cost is calculated in real-time. All item data, including prices, detailed descriptions, and associated images, are seamlessly loaded from a structured CSV file, making inventory management and updates incredibly simple.

## 🚀 Features

Experience a suite of features designed to enhance your cost estimation workflow:

* ✅ **Intuitive Component Selection:** Navigate and choose from a wide array of robot components (e.g., Motors, LIDARs, Cameras) using user-friendly dropdown menus integrated into a clear table format.
* 🔢 **Effortless Quantity Adjustment:** Precisely control the quantity for each selected component using convenient spin boxes, ensuring accurate estimations.
* 💰 **Dynamic Pricing & Calculation:** Witness real-time updates! The application automatically displays the unit price and instantly calculates the total cost for each item as you adjust quantities.
* 📈 **Grand Total at a Glance:** A live-updating grand total provides an immediate overview of the cumulative cost of all selected components, empowering quick decision-making.
* 📄 **Flexible CSV Data Loading:** All item details—from image paths and prices to comprehensive descriptions—are loaded from a well-structured CSV file. This design makes updating your inventory data a breeze, without needing code changes.
* 🖼️ **Rich Visual Feedback:** Enhance your selection process with visual aids! An associated image and a detailed description are dynamically displayed for the currently selected item in each row, providing all necessary context.

## 🛠️ Prerequisites

To embark on your robot cost estimation journey, ensure you have the following essentials:

* **C++ Compiler:** A modern C++11 compatible compiler (e.g., GCC, Clang, MSVC).
* **CMake:** Version 3.5 or higher, for seamless project configuration and build management.
* **Qt Development Libraries:** Specifically, the `Widgets` module from Qt5 or Qt6, which powers the stunning graphical interface.

## 📂 Project Structure

A clear and organized project directory ensures maintainability and ease of navigation:

Robot Cost Estimator/
    ├── CMakeLists.txt
    ├── main.cpp
    ├── mainwindow.h
    ├── mainwindow.cpp
    ├── mainwindow.ui
    ├── image.qrc
    └── data/
        └── Robot Estimate Cost - Sheet1.csv
## 🏗️ Building the Project

Follow these straightforward steps to get your Robot Cost Estimator up and running:

1.  **Clone the repository:** If you haven't already, fetch the project from its source:
    ```bash
    git clone <repository_url>
    cd <project_directory>
    ```

2.  **Create a build directory:** A clean build environment is always recommended:
    ```bash
    mkdir build
    cd build
    ```

3.  **Run CMake to configure the project:** This step generates the necessary build files for your system:
    ```bash
    cmake ..
    ```

4.  **Build the application:** Compile the source code into an executable:
    ```bash
    make
    ```
    *(For Windows users with Visual Studio, you might open the generated `.sln` file and build directly from the IDE).*

## ▶️ Usage

Once built, interacting with the application is intuitive and efficient:

1.  **Launch the executable:** Locate the generated executable within your `build` directory (e.g., `./robot_estimate` on Linux/macOS, `robot_estimate.exe` on Windows) and run it:
    ```bash
    ./robot_estimate
    ```

2.  **Interact with the UI:**
    * **Select Components:** Utilize the dropdown menus in the "Item Name" column to pick the desired robot parts.
    * **Adjust Quantity:** Fine-tune the number of units for each item using the spin boxes in the "Quantity" column.
    * **View Details:** Observe the "Description" column update dynamically with detailed information about your selected item, accompanied by its corresponding image.
    * **Monitor Costs:** Watch as the "Unit Price," "Total" for each item, and the "Total Cost" (the grand total at the bottom) update automatically with every selection and quantity change.

## 💡 Extending the Project

This project is designed to be highly extensible, allowing you to easily adapt it to your evolving needs!

### ➕ Adding/Updating Items via CSV

The core item data resides in a simple CSV file, making updates incredibly straightforward:

1.  **Locate the CSV file:** Find the CSV file specified in `mainwindow.cpp` (e.g., `/home/thukha/qt/estimated_cost/data/Robot Estimate Cost - Sheet1.csv`).
2.  **Edit the CSV:** Open the CSV file using any text editor or spreadsheet software.
3.  **Add/Modify Entries:** Add new rows or modify existing ones, strictly adhering to this format:
    `Category,ItemName,Description,ImagePath,Price`
    * `Category`: The broad classification of the item (e.g., "Motor").
    * `ItemName`: The *unique* and specific name of the item (e.g., "Motor 1 (120kg)"). This is crucial for matching.
    * `Description`: A concise yet informative description of the item.
    * `ImagePath`: The path to the image file for this item. Use `:/path/to/image.jpg` for Qt resources or the full file system path for direct access.
    * `Price`: The numerical cost of the item (e.g., `120.50`).
4.  **Save & Relaunch:** Save your changes to the CSV file. The application will load the updated data on its next run.

### 🆕 Adding New Component Categories to the UI

To introduce entirely new categories of items beyond the existing ones:

* **Update `mainwindow.cpp`:**
    * Modify the `comboOptions` QMap to seamlessly integrate your new category along with its initial set of items.
    * Increment the `itemRows` variable to accommodate the new row in the table.
    * You might also need to adjust `ui->tableWidget->setRowCount()` and the loop boundaries within the `MainWindow` constructor to reflect the expanded table size.

* **Update `mainwindow.ui` (Optional):**
    * While the code handles row counts, you can visually extend the table in Qt Designer if you prefer a more accurate representation within the UI design file.

## ⚠️ Troubleshooting

Encountering issues? Here are some common solutions:

> * **CSV File Not Found:** Double-check that the path to your CSV file specified in `mainwindow.cpp` (e.g., `loadPricesFromCSV("/home/thukha/qt/estimated_cost/data/Robot Estimate Cost - Sheet1.csv");`) is absolutely correct and that the file exists at that exact location.
> * **Image Loading Issues:**
>     * Verify that your `image.qrc` file is correctly configured for resource paths (e.g., `:/image/logo2.jpg`).
>     * Ensure the actual image files exist at the specified paths within the `.qrc` file or direct file system paths.
>     * Confirm that you are using supported image formats (e.g., PNG, JPG).
> * **Build Errors:**
>     * Ensure all necessary Qt development libraries (`Qt5Widgets` or `Qt6Widgets`) are properly installed on your system.
>     * Carefully review your `CMakeLists.txt` for any syntax errors or missing dependencies.
>     * Verify that your C++ compiler is functioning correctly.
> * **Runtime Errors/Crashes:** Always check the console output for any error messages originating from Qt or your application. Ensure that your CSV data is well-formed and perfectly matches the expected format to prevent parsing issues.

## 🤝 Contributing

Your contributions are not just welcomed, they're celebrated! If you have innovative suggestions, discover a pesky bug, or wish to enhance the project in any way, please feel free to:

* 🍴 **Fork this repository.**
* 🐛 **Submit detailed issues** describing any problems or ideas.
* ➡️ **Create pull requests** with your fantastic improvements.

Your input is invaluable and makes this project shine brighter! Thank you for being a part of it.