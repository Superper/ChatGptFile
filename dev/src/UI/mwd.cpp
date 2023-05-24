#include "mwd.h"
#include <cstdlib>
#include <iostream>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this); // 使用 UI 类设置 UI

    // 信号槽连接
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::StartSoftWare);
    connect(ui->pushButton_3, &QPushButton::clicked, this, &MainWindow::StopSoftWare);

    ui->plainTextEdit->setReadOnly(true);
    ui->checkBox->setCheckable(false);
    ui->checkBox_2->setCheckable(false);
    ui->checkBox_3->setCheckable(false);
    ui->checkBox_4->setCheckable(false);
    LogManager::instance()->m_pLogWidget = ui->plainTextEdit;
    LogManager::instance()->m_pCheckBox_1 = ui->checkBox;
    LogManager::instance()->m_pCheckBox_2 = ui->checkBox_2;
    LogManager::instance()->m_pCheckBox_3 = ui->checkBox_3;
    LogManager::instance()->m_pCheckBox_4 = ui->checkBox_4;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::StartSoftWare()
{
    LogManager::instance()->addText("Start the external plug-in...");
    thread_ = std::make_unique<std::thread>([]() {
        // 替换 "your_program" 和 "arguments" 为你要启动的程序的名称和参数
        if (std::system("your_program arguments"))
        {
            std::cout << "The external program was executed successfully" << std::endl;
        }
        else
        {
            std::cerr << "There was an error executing the external program" << std::endl;
        }
    });
    thread_->detach();
}

void MainWindow::StopSoftWare()
{
    LogManager::instance()->addText("Stop the external plug-in...");
    if (std::system("pkill gedit"))
    {
        LogManager::instance()->addText("The external program was closed successfully");
    }
    else
    {
        LogManager::instance()->addText("There was an error closing the external program");
    }
}
