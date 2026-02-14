#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

// A simple lock-free circular buffer for use in shared memory.
// Not thread-safe for multiple writers or multiple readers, but safe for one writer/one reader.
// Buffer stores raw bytes (e.g., serialized protobufs).
//
// Usage:
//   CircularBuffer<4096> buf;
//   buf.Write(data, len);
//   buf.Read(out, maxlen);
//
template <size_t N>
class CircularBuffer {
public:
    CircularBuffer() : head_(0), tail_(0) {}

    // Returns number of bytes available to read
    size_t Available() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        if (head >= tail) return head - tail;
        return N - (tail - head);
    }

    // Returns number of bytes free for writing
    size_t Free() const {
        return N - 1 - Available();
    }

    // Write data to buffer. Returns true if successful, false if not enough space.
    bool Write(const uint8_t* data, size_t len) {
        if (len > Free()) return false;
        size_t head = head_.load(std::memory_order_relaxed);
        for (size_t i = 0; i < len; ++i) {
            buf_[(head + i) % N] = data[i];
        }
        head_.store((head + len) % N, std::memory_order_release);
        return true;
    }

    // Read up to maxlen bytes from buffer into out. Returns number of bytes read.
    size_t Read(uint8_t* out, size_t maxlen) {
        size_t avail = Available();
        size_t to_read = (avail < maxlen) ? avail : maxlen;
        size_t tail = tail_.load(std::memory_order_relaxed);
        for (size_t i = 0; i < to_read; ++i) {
            out[i] = buf_[(tail + i) % N];
        }
        tail_.store((tail + to_read) % N, std::memory_order_release);
        return to_read;
    }

    // Peek at the next byte to read (without advancing tail). Returns true if available.
    bool Peek(uint8_t& out) const {
        if (Available() == 0) return false;
        size_t tail = tail_.load(std::memory_order_acquire);
        out = buf_[tail % N];
        return true;
    }

    // Clear the buffer
    void Clear() {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

private:
    alignas(64) uint8_t buf_[N];
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
}; 