#ifndef CONSUMERS_H
#define CONSUMERS_H
#include <cstdint>
#include <iostream>
#include <thread>
class Consumers
{
  public:
    inline void Consume(const char data[], const uint16_t &size)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::vector<char> temp(data, data + size);
        data_.push_back(temp);
        cond_.notify_one();
    }

    virtual void Process() = 0;

    virtual ~Consumers()
    {
        std::cout << "Server shutdown...\n";
    }

  protected:
    std::unique_ptr<std::thread> thread_;
    std::condition_variable cond_;
    std::mutex mutex_;
    std::vector<std::vector<char>> data_;
};

#endif // CONSUMERS_H