/********************************************************************************
** Form generated from reading UI file 'service.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SERVICE_H
#define UI_SERVICE_H

#include </home/thukha/qtcreator/login/QtPlotter/qcustomplot.h>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_service
{
public:
    QVBoxLayout *verticalLayout_2;
    QFrame *frame;
    QHBoxLayout *horizontalLayout_2;
    QCustomPlot *customPlot;
    QTextEdit *receiveDataTextEdit;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_3;
    QLineEdit *sendMessageLineEdit;
    QPushButton *sendButton;
    QPushButton *clearHistoryButton;
    QCheckBox *pausePlotCheckBox;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QComboBox *serialPortComboBox;
    QComboBox *baudRateComboBox;
    QLabel *label;
    QPushButton *connectButton;
    QPushButton *homebtn;

    void setupUi(QDialog *service)
    {
        if (service->objectName().isEmpty())
            service->setObjectName(QString::fromUtf8("service"));
        service->resize(1125, 645);
        verticalLayout_2 = new QVBoxLayout(service);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        frame = new QFrame(service);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(frame);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        customPlot = new QCustomPlot(frame);
        customPlot->setObjectName(QString::fromUtf8("customPlot"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(customPlot->sizePolicy().hasHeightForWidth());
        customPlot->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(customPlot);

        receiveDataTextEdit = new QTextEdit(frame);
        receiveDataTextEdit->setObjectName(QString::fromUtf8("receiveDataTextEdit"));
        QSizePolicy sizePolicy1(QSizePolicy::Maximum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(receiveDataTextEdit->sizePolicy().hasHeightForWidth());
        receiveDataTextEdit->setSizePolicy(sizePolicy1);

        horizontalLayout_2->addWidget(receiveDataTextEdit);


        verticalLayout_2->addWidget(frame);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(-1, 0, -1, -1);
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(-1, 0, -1, 0);
        groupBox_2 = new QGroupBox(service);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        layoutWidget = new QWidget(groupBox_2);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(22, 22, 258, 241));
        verticalLayout_3 = new QVBoxLayout(layoutWidget);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        sendMessageLineEdit = new QLineEdit(layoutWidget);
        sendMessageLineEdit->setObjectName(QString::fromUtf8("sendMessageLineEdit"));

        verticalLayout_3->addWidget(sendMessageLineEdit);

        sendButton = new QPushButton(layoutWidget);
        sendButton->setObjectName(QString::fromUtf8("sendButton"));
        QSizePolicy sizePolicy2(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(sendButton->sizePolicy().hasHeightForWidth());
        sendButton->setSizePolicy(sizePolicy2);

        verticalLayout_3->addWidget(sendButton);

        clearHistoryButton = new QPushButton(layoutWidget);
        clearHistoryButton->setObjectName(QString::fromUtf8("clearHistoryButton"));
        sizePolicy2.setHeightForWidth(clearHistoryButton->sizePolicy().hasHeightForWidth());
        clearHistoryButton->setSizePolicy(sizePolicy2);

        verticalLayout_3->addWidget(clearHistoryButton);

        pausePlotCheckBox = new QCheckBox(layoutWidget);
        pausePlotCheckBox->setObjectName(QString::fromUtf8("pausePlotCheckBox"));

        verticalLayout_3->addWidget(pausePlotCheckBox);


        horizontalLayout_4->addWidget(groupBox_2);

        groupBox = new QGroupBox(service);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 2);

        serialPortComboBox = new QComboBox(groupBox);
        serialPortComboBox->setObjectName(QString::fromUtf8("serialPortComboBox"));

        gridLayout->addWidget(serialPortComboBox, 0, 1, 1, 2);

        baudRateComboBox = new QComboBox(groupBox);
        baudRateComboBox->setObjectName(QString::fromUtf8("baudRateComboBox"));

        gridLayout->addWidget(baudRateComboBox, 1, 2, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        connectButton = new QPushButton(groupBox);
        connectButton->setObjectName(QString::fromUtf8("connectButton"));

        gridLayout->addWidget(connectButton, 2, 0, 1, 3);


        horizontalLayout_4->addWidget(groupBox);


        verticalLayout->addLayout(horizontalLayout_4);


        verticalLayout_2->addLayout(verticalLayout);

        homebtn = new QPushButton(service);
        homebtn->setObjectName(QString::fromUtf8("homebtn"));
        sizePolicy2.setHeightForWidth(homebtn->sizePolicy().hasHeightForWidth());
        homebtn->setSizePolicy(sizePolicy2);

        verticalLayout_2->addWidget(homebtn);


        retranslateUi(service);

        QMetaObject::connectSlotsByName(service);
    } // setupUi

    void retranslateUi(QDialog *service)
    {
        service->setWindowTitle(QCoreApplication::translate("service", "Dialog", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("service", "Control Section", nullptr));
        sendButton->setText(QCoreApplication::translate("service", "Send", nullptr));
        clearHistoryButton->setText(QCoreApplication::translate("service", "Clear History", nullptr));
        pausePlotCheckBox->setText(QCoreApplication::translate("service", "Pause Plot", nullptr));
        groupBox->setTitle(QCoreApplication::translate("service", "Serial Port Settings", nullptr));
        label_2->setText(QCoreApplication::translate("service", "Baud Rate:", nullptr));
        label->setText(QCoreApplication::translate("service", "Port:", nullptr));
        connectButton->setText(QString());
        homebtn->setText(QCoreApplication::translate("service", "Home", nullptr));
    } // retranslateUi

};

namespace Ui {
    class service: public Ui_service {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SERVICE_H
