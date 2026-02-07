//
// Created by siryellsalot on 2/7/26.
//

#ifndef INC_2026_SYNC_H
#define INC_2026_SYNC_H

template <typename T>
struct SPSCQueue {
private:
    struct alignas(std::hardware_destructive_interference_size) PaddedT { // prevent false sharing
        T value;
    };

    alignas(std::hardware_destructive_interference_size) PaddedT buf[2];
    std::atomic_int head{0};

    std::atomic_bool created {false};
public:
    struct SPSCReader {
        friend struct SPSCQueue;
        SPSCQueue& queue;

    public:
        const T& read() {
            return queue.buf[queue.head.load(std::memory_order_acquire)].value;
        }
    private:
        SPSCReader(SPSCQueue& queue): queue(queue) {}

        SPSCReader(const SPSCReader&) = delete;
        SPSCReader& operator=(const SPSCReader&) = delete;
        SPSCReader(SPSCReader&&) = delete;
        SPSCReader& operator=(SPSCReader&&) = delete;
    };


    void write(T& value) {
        auto next = (head.load(std::memory_order_acquire) + 1) % 2;
        buf[next].value = value;
        head.store(next, std::memory_order_release);
    }

    SPSCQueue(const SPSCReader&) = delete;
    SPSCQueue& operator=(const SPSCQueue&) = delete;
    SPSCQueue(SPSCQueue&&) = delete;
    SPSCQueue& operator=(SPSCQueue&&) = delete;

    SPSCQueue() = default;

    SPSCReader CreateReader() {
        if (created.exchange(true)) {
            throw std::runtime_error("SPSCReader already created");
        }
        return SPSCReader(*this);
    }
};

#endif //INC_2026_SYNC_H