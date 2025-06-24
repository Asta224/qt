#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QDebug>
#include <QVBoxLayout> 

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) 
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this); // 'this' is now correctly a QMainWindow*
    ui->userName->setStyleSheet("color: #333; font-weight: bold; font-size: 16px;");
    ui->passwordLabel->setStyleSheet("color: #333; font-weight: bold; font-size: 16px;");
    ui->stackedWidget->setStyleSheet("QStackedWidget { background-color:hsl(0, 0.00%, 43.10%); padding: 10px; }"
                                      "QStackedWidget::pane { border: 1px solid #ddd; border-radius: 10px; padding: 10px;" "}");
    this->setStyleSheet("QMainWindow#MainWindow { background-color: #f2f2f2; }" // Target the main window
                        "QLabel { color: #333; font-weight: bold; font-size: 16px; }"
                        "QLineEdit { border: 1px solid #ccc; border-radius: 5px; padding: 5px; }"
                        "QPushButton { "
                            "background-color: #4CAF50; color: white; border-radius: 8px; padding: 10px 20px; "
                            "font-weight: bold; font-size: 16px; "
                        "}"
                        "QPushButton:hover { background-color: #45a049; }"
                        "QPushButton:pressed { background-color: #367c39; }"
                        "QFrame#loginFormContainer { " // Style for the container frame
                            "border: 1px solid #ddd; border-radius: 10px; "
                            "background-color: white; padding: 20px; "
                            "box-shadow: 0px 0px 15px rgba(0, 0, 0, 0.1);"
                            "}"
                        );

    // For QMainWindow, the actual UI elements are placed on a central widget.
    // The main layout should be applied to the central widget.
    // ui->centralwidget is the standard objectName for the central widget in QMainWindow forms.
    if (auto *layout = dynamic_cast<QVBoxLayout*>(ui->centralwidget->layout())) {
        layout->setAlignment(Qt::AlignCenter);
    }

    if (auto *loginLayout = dynamic_cast<QVBoxLayout*>(ui->loginFormContainer->layout())) {
        loginLayout->setAlignment(Qt::AlignCenter);
    }

    ui->statusLabel->setText("<span style='color: #333;'>Please log in</span>");
    ui->statusLabel->setAlignment(Qt::AlignCenter);

    ui->usernameInput->setText("");
    ui->passwordInput->setText("");
    ui->passwordInput->setEchoMode(QLineEdit::Password);

    
    ui->welcomeMessage->setAlignment(Qt::AlignCenter);
    ui->welcomeMessage->hide();

    setWindowTitle("Qt WebAssembly Login");
    resize(400, 350);

    // Animation Setup
    m_loginOpacityEffect = new QGraphicsOpacityEffect(ui->loginFormContainer);
    ui->loginFormContainer->setGraphicsEffect(m_loginOpacityEffect);
    m_loginFadeOutAnimation = new QPropertyAnimation(m_loginOpacityEffect, "opacity", this);
    m_loginFadeOutAnimation->setDuration(600);
    m_loginFadeOutAnimation->setStartValue(1.0);
    m_loginFadeOutAnimation->setEndValue(0.0);

    QGraphicsOpacityEffect *welcomeOpacityEffect = new QGraphicsOpacityEffect(ui->welcomeMessage);
    ui->welcomeMessage->setGraphicsEffect(welcomeOpacityEffect);
    m_welcomeFadeInAnimation = new QPropertyAnimation(welcomeOpacityEffect, "opacity", this);
    m_welcomeFadeInAnimation->setDuration(600);
    m_welcomeFadeInAnimation->setStartValue(0.0);
    m_welcomeFadeInAnimation->setEndValue(1.0);

    // Connections
    connect(ui->loginButton, &QPushButton::clicked, this, &MainWindow::onLoginClicked);
    connect(ui->passwordInput, &QLineEdit::returnPressed, this, &MainWindow::onLoginClicked);
    connect(m_loginFadeOutAnimation, &QPropertyAnimation::finished, this, &MainWindow::onLoginFormFadeOutFinished);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onLoginClicked()
{
    QString username = ui->usernameInput->text();
    QString password = ui->passwordInput->text();

    qDebug() << "Attempting login with Username:" << username << " Password:" << password;

    if (username == "testuser" && password == "testpass") {
        ui->statusLabel->setText("<span style='color: green;'>Login Successful!</span>");
        qDebug() << "Login Successful.";
        m_loginFadeOutAnimation->start();
        ui->welcomeMessage->graphicsEffect()->setProperty("opacity", 0.0); 
        ui->welcomeMessage->show();
    } else {
        ui->statusLabel->setText("<span style='color: red;'>Invalid username or password.</span>");
        qDebug() << "Login Failed: Invalid credentials.";
        ui->passwordInput->clear();
    }
}

void MainWindow::onLoginFormFadeOutFinished()
{   
    ui->loginFormContainer->hide();
    ui->stackedWidget->setCurrentIndex(1); // Switch to the welcome page
     m_welcomeFadeInAnimation->start();
    this->setFixedSize(this->size());
    this->setWindowTitle("Welcome");
}
