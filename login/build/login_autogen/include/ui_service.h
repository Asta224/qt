/********************************************************************************
** Form generated from reading UI file 'service.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SERVICE_H
#define UI_SERVICE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_service
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout;
    QPushButton *homebtn;
    QSpacerItem *horizontalSpacer;
    QPushButton *searchBtn;

    void setupUi(QDialog *service)
    {
        if (service->objectName().isEmpty())
            service->setObjectName(QString::fromUtf8("service"));
        service->resize(400, 300);
        service->setStyleSheet(QString::fromUtf8("text-color: black;\n"
""));
        verticalLayout_2 = new QVBoxLayout(service);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(-1, 0, -1, -1);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(service);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        homebtn = new QPushButton(service);
        homebtn->setObjectName(QString::fromUtf8("homebtn"));

        horizontalLayout->addWidget(homebtn);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        searchBtn = new QPushButton(service);
        searchBtn->setObjectName(QString::fromUtf8("searchBtn"));

        horizontalLayout->addWidget(searchBtn);


        verticalLayout->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(service);

        QMetaObject::connectSlotsByName(service);
    } // setupUi

    void retranslateUi(QDialog *service)
    {
        service->setWindowTitle(QCoreApplication::translate("service", "Dialog", nullptr));
        label->setText(QCoreApplication::translate("service", "SERVICE", nullptr));
        homebtn->setText(QCoreApplication::translate("service", "Home", nullptr));
        searchBtn->setText(QCoreApplication::translate("service", "Search", nullptr));
    } // retranslateUi

};

namespace Ui {
    class service: public Ui_service {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SERVICE_H
