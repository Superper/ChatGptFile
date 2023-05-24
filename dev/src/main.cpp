#include "DataProcess.h"
#include "SocketManager.h"
#include "UI/mwd.h"
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.show();

    if (argc < 3)
    {
        std::cerr << "Error: Not enough arguments" << std::endl;
        std::cerr << "Example format : \n ZNDDS 127.0.0.1 1234 127.0.0.1 1235 ";
        return -1;
    }

    auto socketManager = SocketConnectionManager::getInstance();

    // socketManager->AddConnection("dataprocess", SocketType::TCPClient, 12346, "192.168.1.14");
    // socketManager->AddConnection("user", SocketType::TCPServer, 12345, "192.168.1.10");
    socketManager->AddConnection("dataprocess", SocketType::TCPClient, 12346, "192.168.43.62");
    socketManager->AddConnection("user", SocketType::TCPServer, 12345, "192.168.43.226");

    auto users = argc / 2;
    for (auto user = 0; user < users; user++)
    {
        std::string str = "User" + std::to_string(user + 1) + " : " + argv[user * 2 + 1] + " " + argv[user * 2 + 2];
        LogManager::instance()->addText(str);
        auto ip = std::string(argv[user * 2 + 1]);
        auto port = static_cast<uint16_t>(std::stoi(argv[user * 2 + 2]));
        if (user == 0)
        {
            socketManager->SubscribeTopic("user", new DataProcess(ip, port));
        }
        else
        {
            socketManager->SubscribeTopic("dataprocess", new DataProcess(ip, port));
        }
    }
    return app.exec();
}
