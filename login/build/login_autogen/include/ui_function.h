/********************************************************************************
** Form generated from reading UI file 'function.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FUNCTION_H
#define UI_FUNCTION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_function
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout;
    QPushButton *homeBtn;
    QSpacerItem *horizontalSpacer;
    QPushButton *searchBtn;

    void setupUi(QDialog *function)
    {
        if (function->objectName().isEmpty())
            function->setObjectName(QString::fromUtf8("function"));
        function->resize(400, 300);
        function->setStyleSheet(QString::fromUtf8("font-color:black;"));
        verticalLayout_2 = new QVBoxLayout(function);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(-1, 0, -1, -1);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(function);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        homeBtn = new QPushButton(function);
        homeBtn->setObjectName(QString::fromUtf8("homeBtn"));

        horizontalLayout->addWidget(homeBtn);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        searchBtn = new QPushButton(function);
        searchBtn->setObjectName(QString::fromUtf8("searchBtn"));

        horizontalLayout->addWidget(searchBtn);


        verticalLayout->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(function);

        QMetaObject::connectSlotsByName(function);
    } // setupUi

    void retranslateUi(QDialog *function)
    {
        function->setWindowTitle(QCoreApplication::translate("function", "Dialog", nullptr));
        label->setText(QCoreApplication::translate("function", "FUNCTION", nullptr));
        homeBtn->setText(QCoreApplication::translate("function", "Home", nullptr));
        searchBtn->setText(QCoreApplication::translate("function", "Search", nullptr));
    } // retranslateUi

};

namespace Ui {
    class function: public Ui_function {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FUNCTION_H
