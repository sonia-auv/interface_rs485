
#ifndef INTERFACE_RS485_SHAREDQUEUE_H
#define INTERFACE_RS485_SHAREDQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SharedQueue
{
public:
    SharedQueue();
    ~SharedQueue();

    T& front();
    void pop_front();

    T get_n_pop_front();

    void push_back(const T& item);
    void push_back(T&& item);


    unsigned long size();
    bool empty();

private:
    std::deque<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

template <typename T>
SharedQueue<T>::SharedQueue(){}

template <typename T>
SharedQueue<T>::~SharedQueue(){}

template <typename T>
T& SharedQueue<T>::front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    return queue_.front();
}

template <typename T>
void SharedQueue<T>::pop_front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    queue_.pop_front();
}

template <typename T>
T SharedQueue<T>::get_n_pop_front()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
        cond_.wait(mlock);
    }
    T temp = queue_.front();
    queue_.pop_front();
    mlock.unlock();
    cond_.notify_one();
    return temp;
}

template <typename T>
void SharedQueue<T>::push_back(const T& item)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push_back(item);
    mlock.unlock();     // unlock before notificiation to minimize mutex con
    cond_.notify_one(); // notify one waiting thread

}

template <typename T>
void SharedQueue<T>::push_back(T&& item)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push_back(std::move(item));
    mlock.unlock();     // unlock before notificiation to minimize mutex con
    cond_.notify_one(); // notify one waiting thread

}

template <typename T>
unsigned long SharedQueue<T>::size()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    unsigned long size = queue_.size();
    mlock.unlock();
    cond_.notify_one();
    return size;
}

template <typename T>
bool SharedQueue<T>::empty()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    bool empty = queue_.empty();
    mlock.unlock();
    cond_.notify_one();
    return empty;
}

#endif //INTERFACE_RS485_SHAREDQUEUE_H
