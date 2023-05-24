/********************************************************************************
** Form generated from reading UI file 'mwd.ui'
**
** Created by: Qt User Interface Compiler version 6.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MWD_H
#define UI_MWD_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton;
    QPushButton *pushButton_3;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QCheckBox *checkBox_4;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox_3;
    QPlainTextEdit *plainTextEdit;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(418, 536);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName("gridLayout");
        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName("groupBox_2");
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName("verticalLayout_2");
        pushButton = new QPushButton(groupBox_2);
        pushButton->setObjectName("pushButton");

        verticalLayout_2->addWidget(pushButton);

        pushButton_3 = new QPushButton(groupBox_2);
        pushButton_3->setObjectName("pushButton_3");

        verticalLayout_2->addWidget(pushButton_3);


        gridLayout->addWidget(groupBox_2, 0, 0, 1, 1);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName("groupBox");
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName("verticalLayout");
        checkBox_4 = new QCheckBox(groupBox);
        checkBox_4->setObjectName("checkBox_4");

        verticalLayout->addWidget(checkBox_4);

        checkBox = new QCheckBox(groupBox);
        checkBox->setObjectName("checkBox");

        verticalLayout->addWidget(checkBox);

        checkBox_2 = new QCheckBox(groupBox);
        checkBox_2->setObjectName("checkBox_2");

        verticalLayout->addWidget(checkBox_2);

        checkBox_3 = new QCheckBox(groupBox);
        checkBox_3->setObjectName("checkBox_3");

        verticalLayout->addWidget(checkBox_3);


        gridLayout->addWidget(groupBox, 0, 1, 1, 1);

        plainTextEdit = new QPlainTextEdit(centralwidget);
        plainTextEdit->setObjectName("plainTextEdit");

        gridLayout->addWidget(plainTextEdit, 1, 0, 1, 2);

        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "\350\275\257\344\273\266\346\223\215\344\275\234", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "\345\220\257\345\212\250\346\217\222\344\273\266", nullptr));
        pushButton_3->setText(QCoreApplication::translate("MainWindow", "\345\205\263\351\227\255\346\217\222\344\273\266", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "\350\277\236\346\216\245\347\212\266\346\200\201", nullptr));
        checkBox_4->setText(QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\346\272\220", nullptr));
        checkBox->setText(QCoreApplication::translate("MainWindow", "\345\217\230\345\214\226\346\243\200\346\265\213\346\217\222\344\273\266", nullptr));
        checkBox_2->setText(QCoreApplication::translate("MainWindow", "\347\233\256\346\240\207\346\243\200\346\265\213\346\217\222\344\273\266", nullptr));
        checkBox_3->setText(QCoreApplication::translate("MainWindow", "\345\244\232\346\272\220\344\277\241\346\201\257\350\236\215\345\220\210\346\217\222\344\273\266", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MWD_H
