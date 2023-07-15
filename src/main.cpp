#include "DataProcess.h"
#include "ImgCheckResult.h"
#include "SocketManager.h"
#include "UI/mwd.h"
#include <fstream>
#include <iostream>
#include <string>

#define MYCLIENTPORT 10000

bool GetAddress(const std::string &data, std::string &ip, uint16_t &port)
{

    // 查找添加分隔符：
    size_t sep_pos = data.find(':');

    if (sep_pos == std::string::npos)
    {
        std::cerr << "Delimiter not found. Please ensure the string is in the format: <IP>:<Port>" << std::endl;
        return false;
    }

    // 提取 IP：
    ip = data.substr(0, sep_pos);

    // 将字符串转换为整数：
    try
    {
        port = std::stoi(data.substr(sep_pos + 1));
    }
    catch (std::invalid_argument &e)
    {
        std::cerr << "Invalid port number: " << e.what() << std::endl;
        return false;
    }
    catch (std::out_of_range &e)
    {
        std::cerr << "Port number out of range: " << e.what() << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow mainWindow;

    if (argc < 4)
    {
        std::cerr << "Error: Not enough arguments,format ZNNDS <Support the environment> <Change Detection Software> "
                     "<Target detection software> <Multi-source information fusion software>"
                  << std::endl;
        std::cerr << "Example format : \n ZNDDS 192.168.1.1:1234 127.0.0.1:10001 127.0.0.1:10002 127.0.0.1:10003";
        return -1;
    }

    auto socketManager = Socket::SocketConnectionManager::getInstance();
    socketManager->SetSocketStateCallback([&mainWindow](const std::string &msg) { mainWindow.UIControlDisplay(msg); });

    // 初始化server数据处理
    auto server_data_process =
        new DataProcess([&mainWindow](const std::string &msg) { mainWindow.UIControlDisplay(msg); });

    // 初始化client数据处理
    auto client_data_process = new ImgCheckResult();
    client_data_process->SetMsgCB([&mainWindow](const std::string &msg) { mainWindow.UIControlDisplay(msg); });

    // 根据输入参数初始化数据处理插件
    for (auto user = 1; user < 6; user++)
    {
        std::string ip;
        uint16_t port;
        if (!GetAddress(argv[user], ip, port))
        {
            std::cerr << "Error: Invalid server address." << std::endl;
            return -1;
        }

        std::string str;

        switch (user)
        {
        case 1:
            // 初始化client接收环境支持数据
            socketManager->AddConnection("SupportEnvironment", Socket::SocketType::TCPClient, port, ip, MYCLIENTPORT);
            socketManager->SubscribeTopic("SupportEnvironment", server_data_process);

            str = "Init support-environment: " + ip + ":" + std::to_string(port);

            // 设置MSIF处理后数据的发送地址
            client_data_process->AddSendAddr(ip, port);

            mainWindow.ip_se = ip;
            mainWindow.port_se = port;
            break;
        case 2:
            // 初始化server接收处理后的数据
            str = "Init TCP server: " + ip + ":" + std::to_string(port);
            socketManager->AddConnection("dataprocess", Socket::SocketType::TCPServer, port, ip, 0);
            socketManager->SubscribeTopic("dataprocess", client_data_process);
            break;
        case 5:
            // 初始化多源信息融合软件ip和端口
            str = "Init MSIF: " + ip + ":" + std::to_string(port);

            mainWindow.ip_msif = ip;
            mainWindow.port_msif = port;
            break;
        default:
            str = "Init software-addr: " + ip + ":" + std::to_string(port);
            server_data_process->AddSendAddr(ip, port);

            if (user == 2)
            {
                mainWindow.ip_cd = ip;
                mainWindow.port_cd = port;
            }
            else if (user == 3)
            {
                mainWindow.ip_ds = ip;
                mainWindow.port_ds = port;
            }
            break;
        }

        mainWindow.UIControlDisplay(str);
    }

    //  是否开启ui
    std::string target_flag = "-guictl";
    bool target_value = true;

    for (int i = 1; i < argc; ++i)
    {
        if (target_flag == argv[i] && i + 1 < argc)
        {
            std::string next_arg = argv[i + 1];
            if (next_arg == "false" || next_arg == "False")
            {
                target_value = false;
                break;
            }
        }
    }

    if (target_value)
    {
        std::cout << "Showing GUI..." << std::endl;
        mainWindow.show();
    }
    else
    {
        std::cout << "Running without GUI..." << std::endl;
    }

    return app.exec();
}
