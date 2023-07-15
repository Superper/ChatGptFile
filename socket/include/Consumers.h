#ifndef CONSUMERS_H
#define CONSUMERS_H
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
class Consumers
{
  public:
    inline void Consume(const char data[], const uint16_t &size)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::vector<uint8_t> temp(data, data + size);
        data_.insert(data_.end(), temp.begin(), temp.end());

        cond_.notify_one();
    }

    virtual void Process() = 0;

    virtual ~Consumers()
    {
        std::cout << "Server shutdown...\n";
    }
    void AddSendAddr(const std::string &ip, const uint16_t &port)
    {
        SendAddr send_addr;
        send_addr.ip = ip;
        send_addr.port = port;
        send_addr_vec_.push_back(send_addr);
    }

  protected:
    struct SendAddr
    {
        std::string ip;
        uint16_t port;
    };
    std::vector<SendAddr> send_addr_vec_;
    std::unique_ptr<std::thread> thread_;
    std::condition_variable cond_;
    std::mutex mutex_;
    std::vector<uint8_t> data_;
};

#endif // CONSUMERS_H
