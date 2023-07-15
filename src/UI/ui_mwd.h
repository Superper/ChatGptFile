/********************************************************************************
** Form generated from reading UI file 'mwd.ui'
**
** Created by: Qt User Interface Compiler version 6.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MWD_H
#define UI_MWD_H

#include "MyCheckBox.h"
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QLabel *label;
    QLabel *sar_c;
    QLabel *label_2;
    QLabel *ligth_c;
    QLabel *label_3;
    QLabel *hw_c;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *car_c;
    QLabel *plane_c;
    QLabel *ship_c;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    MyCheckBox *checkBox_4;
    MyCheckBox *checkBox;
    MyCheckBox *checkBox_2;
    MyCheckBox *checkBox_3;
    QPlainTextEdit *plainTextEdit;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton;
    QSpacerItem *horizontalSpacer_2;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(430, 710);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName("gridLayout_2");
        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName("groupBox_2");
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setObjectName("gridLayout");
        label = new QLabel(groupBox_2);
        label->setObjectName("label");

        gridLayout->addWidget(label, 0, 0, 1, 1);

        sar_c = new QLabel(groupBox_2);
        sar_c->setObjectName("sar_c");

        gridLayout->addWidget(sar_c, 0, 1, 1, 1);

        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName("label_2");

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        ligth_c = new QLabel(groupBox_2);
        ligth_c->setObjectName("ligth_c");

        gridLayout->addWidget(ligth_c, 1, 1, 1, 1);

        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName("label_3");

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        hw_c = new QLabel(groupBox_2);
        hw_c->setObjectName("hw_c");

        gridLayout->addWidget(hw_c, 2, 1, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName("label_4");

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName("label_5");

        gridLayout->addWidget(label_5, 4, 0, 1, 1);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName("label_6");

        gridLayout->addWidget(label_6, 5, 0, 1, 1);

        car_c = new QLabel(groupBox_2);
        car_c->setObjectName("car_c");

        gridLayout->addWidget(car_c, 3, 1, 1, 1);

        plane_c = new QLabel(groupBox_2);
        plane_c->setObjectName("plane_c");

        gridLayout->addWidget(plane_c, 4, 1, 1, 1);

        ship_c = new QLabel(groupBox_2);
        ship_c->setObjectName("ship_c");

        gridLayout->addWidget(ship_c, 5, 1, 1, 1);


        gridLayout_2->addWidget(groupBox_2, 0, 0, 1, 1);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName("groupBox");
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName("verticalLayout");
        checkBox_4 = new MyCheckBox(groupBox);
        checkBox_4->setObjectName("checkBox_4");

        verticalLayout->addWidget(checkBox_4);

        checkBox = new MyCheckBox(groupBox);
        checkBox->setObjectName("checkBox");

        verticalLayout->addWidget(checkBox);

        checkBox_2 = new MyCheckBox(groupBox);
        checkBox_2->setObjectName("checkBox_2");

        verticalLayout->addWidget(checkBox_2);

        checkBox_3 = new MyCheckBox(groupBox);
        checkBox_3->setObjectName("checkBox_3");

        verticalLayout->addWidget(checkBox_3);


        gridLayout_2->addWidget(groupBox, 0, 1, 1, 1);

        plainTextEdit = new QPlainTextEdit(centralwidget);
        plainTextEdit->setObjectName("plainTextEdit");

        gridLayout_2->addWidget(plainTextEdit, 1, 0, 1, 2);

        widget = new QWidget(centralwidget);
        widget->setObjectName("widget");
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer = new QSpacerItem(159, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButton = new QPushButton(widget);
        pushButton->setObjectName("pushButton");

        horizontalLayout->addWidget(pushButton);

        horizontalSpacer_2 = new QSpacerItem(158, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);


        gridLayout_2->addWidget(widget, 2, 0, 1, 2);

        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "ZNDDS", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\350\256\260\345\275\225", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "SAR\345\233\276\345\203\217\346\216\245\346\224\266\346\225\260\351\207\217", nullptr));
        sar_c->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "\345\217\257\350\247\201\345\205\211\345\233\276\345\203\217\346\216\245\346\224\266\346\225\260\351\207\217", nullptr));
        ligth_c->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "\347\272\242\345\244\226\345\233\276\345\203\217\346\216\245\346\224\266\346\225\260\351\207\217", nullptr));
        hw_c->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "\350\275\246\350\276\206\347\233\256\346\240\207\346\243\200\346\265\213\344\270\252\346\225\260", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "\351\243\236\346\234\272\347\233\256\346\240\207\346\243\200\346\265\213\344\270\252\346\225\260", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "\350\210\271\345\217\252\347\233\256\346\240\207\346\243\200\346\265\213\344\270\252\346\225\260", nullptr));
        car_c->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        plane_c->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        ship_c->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "\350\277\236\346\216\245\347\212\266\346\200\201", nullptr));
        checkBox_4->setText(QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\346\272\220", nullptr));
        checkBox->setText(QCoreApplication::translate("MainWindow", "\345\217\230\345\214\226\346\243\200\346\265\213\346\217\222\344\273\266", nullptr));
        checkBox_2->setText(QCoreApplication::translate("MainWindow", "\347\233\256\346\240\207\346\243\200\346\265\213\346\217\222\344\273\266", nullptr));
        checkBox_3->setText(QCoreApplication::translate("MainWindow", "\345\244\232\346\272\220\344\277\241\346\201\257\350\236\215\345\220\210\346\217\222\344\273\266", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "\347\273\223\346\236\234\346\230\276\347\244\272", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MWD_H
