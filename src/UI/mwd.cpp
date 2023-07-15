#include "mwd.h"
#include "../DataProcess/SystemInterface.h"
#include <QMetaType>
#include <QString>
#include <QTextBlock>
#include <QTextCursor>
#include <cstdlib>
#include <iostream>

std::string folderpath = "/usr/";
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    qRegisterMetaType<QTextBlock>("QTextBlock");
    qRegisterMetaType<QTextCursor>("QTextCursor");
    ui->setupUi(this); // 使用 UI 类设置 UI
    ui->plainTextEdit->setReadOnly(true);

    // 信号槽连接
    connect(ui->pushButton, &QPushButton::click, this, &MainWindow::on_pushButton_clicked);
    connect(this, &MainWindow::SetPlainTextEditSignal, this, &MainWindow::SetPlainTextEdit);
}

void MainWindow::on_pushButton_clicked()
{
    // std::string command = "xdg-open " + folderpath;
    // int error = std::system(command.c_str());

    // if (error)
    // {
    //     ui->plainTextEdit->appendPlainText("error cmd: " + QString::fromStdString(command));
    // }
    ImageDetectionResult data;

    // 设置PkgHeader字段的值
    data.header.header = 0x55555555;
    data.header.length = 100;
    data.header.srcId = 1234;
    data.header.dstId = 5678;
    data.header.id = 0x0111;
    data.header.sendTime = 888888;
    data.header.sendSeq = 9876;

    // 设置ImageDetectionResult其他字段的值
    data.recv_timestamp = swap_endian64(GetPkgTime());
    data.finish_timestamp = swap_endian64(GetPkgTime());
    data.is_target = 1;
    data.target_composition = 2;
    data.target_source = 3;
    data.targetType = 4;
    data.targetTypeConfidence = 5;
    data.targetWZType = 6;
    data.targetWZTypeConfidence = 7;
    data.tar_xh = swap_endian16(8);
    data.tar_ser_air_type = 9;
    data.dw = 10;

    // 设置TargetMsg字段的值
    data.kjg_targtet_msg.tar_position_msg = swap_endian64(1234567890);
    data.kjg_targtet_msg.image_source = 1;
    data.kjg_targtet_msg.target_locating_method_1_validity = 1;
    data.kjg_targtet_msg.target_locating_method_2_validity = 2;
    data.kjg_targtet_msg.target_locating_method_3_validity = 1;
    data.kjg_targtet_msg.target_image_center_pixel_coordinate[0] = swap_endian32(123);
    data.kjg_targtet_msg.target_image_center_pixel_coordinate[1] = swap_endian32(456);
    data.kjg_targtet_msg.longitude_1 = swap_endian32(123456);
    data.kjg_targtet_msg.latitude_1 = swap_endian32(654321);
    data.kjg_targtet_msg.height_1 = swap_endian32(100);
    data.kjg_targtet_msg.longitude_2 = swap_endian32(654321);
    data.kjg_targtet_msg.latitude_2 = swap_endian32(123456);
    data.kjg_targtet_msg.height_2 = swap_endian32(200);
    data.kjg_targtet_msg.longitude_3 = swap_endian32(987654);
    data.kjg_targtet_msg.latitude_3 = swap_endian32(789012);
    data.kjg_targtet_msg.height_3 = swap_endian32(300);

    data.sar_targtet_msg.tar_position_msg = swap_endian64(1234567890);
    data.sar_targtet_msg.image_source = 2;
    data.sar_targtet_msg.target_locating_method_1_validity = 1;
    data.sar_targtet_msg.target_locating_method_2_validity = 2;
    data.sar_targtet_msg.target_locating_method_3_validity = 1;
    data.sar_targtet_msg.longitude_1 = swap_endian32(123456);
    data.sar_targtet_msg.latitude_1 = swap_endian32(654321);
    data.sar_targtet_msg.height_1 = swap_endian32(100);
    data.sar_targtet_msg.longitude_2 = swap_endian32(654321);
    data.sar_targtet_msg.latitude_2 = swap_endian32(123456);
    data.sar_targtet_msg.height_2 = swap_endian32(200);
    data.sar_targtet_msg.longitude_3 = swap_endian32(987654);
    data.sar_targtet_msg.latitude_3 = swap_endian32(789012);
    data.sar_targtet_msg.height_3 = swap_endian32(300);

    data.hw_targtet_msg.tar_position_msg = swap_endian64(1234567890);
    data.hw_targtet_msg.image_source = 3;
    data.hw_targtet_msg.target_locating_method_1_validity = 1;
    data.hw_targtet_msg.target_locating_method_2_validity = 2;
    data.hw_targtet_msg.target_locating_method_3_validity = 1;
    data.hw_targtet_msg.longitude_1 = swap_endian32(123456);
    data.hw_targtet_msg.latitude_1 = swap_endian32(654321);
    data.hw_targtet_msg.height_1 = swap_endian32(100);
    data.hw_targtet_msg.longitude_2 = swap_endian32(654321);
    data.hw_targtet_msg.latitude_2 = swap_endian32(123456);
    data.hw_targtet_msg.height_2 = swap_endian32(200);
    data.hw_targtet_msg.longitude_3 = swap_endian32(987654);
    data.hw_targtet_msg.latitude_3 = swap_endian32(789012);
    data.hw_targtet_msg.height_3 = swap_endian32(300);
    data.tail = 0xAAAAAAAA;

    udp.writeDatagram((char *)&data, sizeof(data), QHostAddress("192.168.112.47"), 22001);
    udp.writeDatagram((char *)&data, sizeof(data), QHostAddress("127.0.0.1"), 52047);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::UIControlDisplay(const std::string &msg)
{
    emit SetPlainTextEditSignal(QString::fromStdString(msg));
}

void MainWindow::SetPlainTextEdit(QString msg)
{
    std::cout << msg.toStdString() << std::endl;
    std::unique_lock<std::mutex> lock(mx_);
    // 与环境支持软件的连接状态
    if (msg.contains(ip_se.c_str()) && msg.contains(std::to_string(port_se).c_str()))
    {
        if (msg.contains("connected"))
        {
            ui->checkBox_4->setChecked(true);
        }
        if (msg.contains("disconnected"))
        {
            ui->checkBox_4->setChecked(false);
        }
        ui->plainTextEdit->appendPlainText(msg);
        return;
    }

    // 与目标检测软件的连接状态
    if (msg.contains(ip_ds.c_str()) && msg.contains(std::to_string(port_ds).c_str()))
    {
        if (msg.contains("connected"))
        {
            ui->checkBox_2->setChecked(true);
        }
        if (msg.contains("disconnected"))
        {
            ui->checkBox_2->setChecked(false);
        }
        ui->plainTextEdit->appendPlainText(msg);
        return;
    }

    // 与变化检测软件的连接状态
    if (msg.contains(ip_cd.c_str()) && msg.contains(std::to_string(port_cd).c_str()))
    {
        if (msg.contains("connected"))
        {
            ui->checkBox->setChecked(true);
        }
        if (msg.contains("disconnected"))
        {
            ui->checkBox->setChecked(false);
        }
        ui->plainTextEdit->appendPlainText(msg);
        return;
    }

    // 与MSIF的连接状态
    if (msg.contains(ip_msif.c_str()) && msg.contains(std::to_string(port_msif).c_str()))
    {
        if (msg.contains("connected"))
        {
            ui->checkBox_3->setChecked(true);
        }
        if (msg.contains("disconnected"))
        {
            ui->checkBox_3->setChecked(false);
        }
        ui->plainTextEdit->appendPlainText(msg);

        return;
    }

    if (msg.contains("inf-"))
    {
        ui->sar_c->setText(msg.mid(4));
        return;
    }
    if (msg.contains("lig-"))
    {
        ui->ligth_c->setText(msg.mid(4));
        return;
    }
    if (msg.contains("sar-"))
    {
        ui->sar_c->setText(msg.mid(4));
        return;
    }
    if (msg.contains("car-"))
    {
        ui->car_c->setText(msg.mid(4));
        return;
    }
    if (msg.contains("pla-"))
    {
        ui->plane_c->setText(msg.mid(4));
        return;
    }
    if (msg.contains("shi-"))
    {
        ui->ship_c->setText(msg.mid(4));
        return;
    }
    if (msg.contains("clear"))
    {
        ui->sar_c->setText("0");
        ui->ligth_c->setText("0");
        ui->car_c->setText("0");
        ui->plane_c->setText("0");
        ui->ship_c->setText("0");
        ui->plainTextEdit->appendPlainText(msg);
        return;
    }
    if (msg.contains("start"))
    {
        ui->plainTextEdit->appendPlainText(msg);
        return;
    }
}
