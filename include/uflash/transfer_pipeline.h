#pragma once

#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace uflash {

class BslProtocol;

struct FramedPacket {
    size_t offset = 0;
    size_t size = 0;
    std::vector<uint8_t> frame;
};

class PacketPipeline {
public:
    PacketPipeline(BslProtocol& proto,
                   const std::vector<uint8_t>& buffer,
                   size_t chunk_size,
                   bool enabled,
                   size_t max_depth);
    ~PacketPipeline();

    PacketPipeline(const PacketPipeline&) = delete;
    PacketPipeline& operator=(const PacketPipeline&) = delete;

    void start(size_t initial_offset);
    void stop();
    void reset(size_t offset, size_t chunk_size);
    bool pop(FramedPacket& out);

private:
    void run();

    BslProtocol& proto_;
    const std::vector<uint8_t>& buffer_;
    size_t chunk_size_;
    const bool enabled_;
    const size_t max_depth_;
    size_t next_offset_ = 0;
    bool stop_ = false;
    bool done_ = false;
    std::deque<FramedPacket> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::thread worker_;
};

} // namespace uflash
