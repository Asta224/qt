# Robot Cost Estimator (Qt/C++)

This project is a desktop GUI application built with Qt and C++ designed to help estimate the cost of a robot by allowing users to select various components and specify quantities. It loads item data, including prices, descriptions, and associated images, from a CSV file, and dynamically calculates the total cost.

## Features

* **Component Selection:** Select different robot components (Motors, LIDARs, Cameras, etc.) using dropdown menus in a table format.
* **Quantity Adjustment:** Easily adjust the quantity for each selected component using spin boxes.
* **Dynamic Pricing:** Automatically displays the unit price and calculates the total cost for each item based on the selected quantity.
* **Grand Total Calculation:** Provides a real-time grand total of all selected components.
* **CSV Data Loading:** Loads all item details (image path, price, description) from a structured CSV file, making it easy to update the inventory.
* **Visual Feedback:** Displays an image and a detailed description for the currently selected item in each row.

## Prerequisites

To build and run this application, you need the following:

* **C++ Compiler:** A C++11 compatible compiler (e.g., GCC, Clang, MSVC).
* **CMake:** Version 3.5 or higher.
* **Qt Development Libraries:** Qt5 or Qt6, specifically the `Widgets` module.

## Project Structure

* `CMakeLists.txt`: The CMake build configuration file. It defines the project, specifies source files, finds Qt libraries, and configures the executable.
* `main.cpp`: The application's entry point. It initializes the Qt application and creates the main window.
* `mainwindow.h`: The header file for the `MainWindow` class. It declares the `ItemData` struct, `itemDataMap` (for storing CSV data), and the `loadPricesFromCSV` function, along with UI-related members.
* `mainwindow.cpp`: The implementation of the `MainWindow` class. It handles UI setup, loads data from the CSV, dynamically creates table widgets (combo boxes, spin boxes, labels), and connects signals/slots for real-time calculations and updates.
* `mainwindow.ui`: The Qt Designer UI definition file that describes the layout and components of the main application window.
* `image.qrc`: A Qt Resource Collection file, used to embed images (like `logo2.jpg`) directly into the executable.

Robot Cost Estimator/
├── CMakeLists.txt
├── main.cpp
├── mainwindow.h
├── mainwindow.cpp
├── mainwindow.ui
├── image.qrc
└── data/
    └── Robot Estimate Cost - Sheet1.csv

## Building the Project

1.  **Clone the repository (if applicable):**
    ```bash
    git clone <repository_url>
    cd <project_directory>
    ```

2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

3.  **Run CMake to configure the project:**
    ```bash
    cmake ..
    ```

4.  **Build the application:**
    ```bash
    make
    ```
    (On Windows with Visual Studio, you might open the generated `.sln` file and build from there).

## Usage

1.  **Run the executable:**
    After building, you will find the executable in the `build` directory (e.g., `./robot_estimate` on Linux/macOS, `robot_estimate.exe` on Windows).

    ```bash
    ./robot_estimate
    ```

2.  **Interact with the UI:**
    * **Select Components:** Use the dropdown menus in the "Item Name" column to choose different robot parts.
    * **Adjust Quantity:** Use the spin boxes in the "Quantity" column to set the number of units for each item.
    * **View Details:** The "Description" column will update with details about the selected item, and an associated image will appear.
    * **Monitor Costs:** The "Unit Price", "Total", and "Total Cost" (grand total at the bottom) will update automatically as you make selections and change quantities.

## Extending the Project

### Adding/Updating Items via CSV

The application loads its item data from a CSV file. To add new items or update existing ones:

1.  Locate the CSV file specified in `mainwindow.cpp` (e.g., `/home/thukha/qt/estimated_cost/data/Robot Estimate Cost - Sheet1.csv`).
2.  Open the CSV file with a text editor or spreadsheet software.
3.  Add new rows or modify existing ones following this format:
    `Category,ItemName,Description,ImagePath,Price`
    * `Category`: The category of the item (e.g., "Motor").
    * `ItemName`: The specific name of the item (e.g., "Motor 1 (120kg)"). This is used for matching.
    * `Description`: A brief description of the item.
    * `ImagePath`: The path to the image file for this item. If it's a Qt resource, use `:/path/to/image.jpg`. If it's a direct file path, use the full path.
    * `Price`: The numerical price of the item (e.g., `120.50`).
4.  Save the CSV file. The application will load the updated data on next run.

### Adding New Component Categories to the UI

If you want to add a completely new category of items that isn't in `comboOptions`:

1.  **Update `mainwindow.cpp`:**
    * Modify the `comboOptions` QMap to include your new category and its initial items.
    * Increase the `itemRows` variable to account for the new row.
    * You might need to adjust `ui->tableWidget->setRowCount()` and the loop bounds in the `MainWindow` constructor.

2.  **Update `mainwindow.ui` (Optional, if you want more rows in the UI designer):**
    * You can extend the table in Qt Designer if you prefer, though the `setRowCount` in code overrides it.

## Troubleshooting

> ⚠️ **Troubleshooting**
>
> * **CSV File Not Found:** Ensure the path to your CSV file in `mainwindow.cpp` (`loadPricesFromCSV("/home/thukha/qt/estimated_cost/data/Robot Estimate Cost - Sheet1.csv");`) is correct and the file exists at that location.
> * **Image Loading Issues:**
>     * Verify that the `image.qrc` file is correctly configured for resource paths (e.g., `:/image/logo2.jpg`).
>     * Ensure the image files themselves exist at the specified paths within the `.qrc` file or direct file system paths.
>     * Check for correct image formats (e.g., PNG, JPG).
> * **Build Errors:**
>     * Ensure all Qt development libraries (`Qt5Widgets` or `Qt6Widgets`) are installed.
>     * Check your `CMakeLists.txt` for correct syntax and dependencies.
>     * Verify your C++ compiler is working correctly.
> * **Runtime Errors/Crashes:** Check the console output for any error messages from Qt or your application. Ensure the CSV data is well-formed and matches the expected format.
