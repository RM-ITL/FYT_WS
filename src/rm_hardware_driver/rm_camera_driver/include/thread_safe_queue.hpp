#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

namespace utils  // 新增命名空间
{

template <typename T>
class ThreadSafeQueue
{
public:
    // 新增：带容量参数的构造函数（匹配代码中 queue_(3) 的初始化）
    explicit ThreadSafeQueue(size_t capacity = 0) : capacity_(capacity) {}
    ~ThreadSafeQueue() = default;

    // 禁止拷贝
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    // 移动构造
    ThreadSafeQueue(ThreadSafeQueue&& other) noexcept
    {
        std::lock_guard<std::mutex> lock(other.mutex_);
        queue_ = std::move(other.queue_);
        capacity_ = other.capacity_;
    }

    // 入队（代码中用 push，保持接口一致）
    void push(const T& data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (capacity_ > 0 && queue_.size() >= capacity_)
            queue_.pop();  // 容量满时丢弃最旧数据
        queue_.push(data);
        cond_.notify_one();
    }

    void push(T&& data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (capacity_ > 0 && queue_.size() >= capacity_)
            queue_.pop();
        queue_.push(std::move(data));
        cond_.notify_one();
    }

    // 出队（代码中用 pop(data)，修改接口匹配）
    bool pop(T& data, int timeout_ms = -1)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout_ms > 0)
        {
            if (!cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty(); }))
            {
                return false; // 超时返回失败
            }
        }
        else
        {
            cond_.wait(lock, [this] { return !queue_.empty(); });
        }

        data = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    // 清空队列
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty())
            queue_.pop();
    }

    // 获取队列大小
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    // 判断队列是否为空
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    std::queue<T> queue_;
    size_t capacity_;  // 新增容量参数
};

}  // 关闭 utils 命名空间

#endif // THREAD_SAFE_QUEUE_HPP
