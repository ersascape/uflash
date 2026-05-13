#include "uflash/transfer_pipeline.h"

#include "uflash/bsl_protocol.h"

#include <algorithm>
#include <utility>

namespace uflash {

PacketPipeline::PacketPipeline(BslProtocol& proto,
                               const std::vector<uint8_t>& buffer,
                               size_t chunk_size,
                               bool enabled,
                               size_t max_depth)
    : proto_(proto),
      buffer_(buffer),
      chunk_size_(chunk_size),
      enabled_(enabled),
      max_depth_(max_depth) {}

PacketPipeline::~PacketPipeline() {
    stop();
}

void PacketPipeline::start(size_t initial_offset) {
    next_offset_ = initial_offset;
    done_ = false;
    stop_ = false;
    if (!enabled_) {
        return;
    }
    worker_ = std::thread([this]() { run(); });
}

void PacketPipeline::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
        cond_.notify_all();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
}

void PacketPipeline::reset(size_t offset, size_t chunk_size) {
    stop();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        chunk_size_ = chunk_size;
        next_offset_ = offset;
        stop_ = false;
        done_ = false;
    }
    start(offset);
}

bool PacketPipeline::pop(FramedPacket& out) {
    if (!enabled_) {
        return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this]() { return stop_ || !queue_.empty() || done_; });
    if (queue_.empty()) {
        return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    cond_.notify_all();
    return true;
}

void PacketPipeline::run() {
    while (true) {
        FramedPacket packet;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_.wait(lock, [this]() { return stop_ || queue_.size() < max_depth_; });
            if (stop_) {
                return;
            }
            if (next_offset_ >= buffer_.size()) {
                done_ = true;
                cond_.notify_all();
                return;
            }
            packet.offset = next_offset_;
            packet.size = std::min(chunk_size_, buffer_.size() - next_offset_);
            next_offset_ += packet.size;
        }
        packet.frame = proto_.build_midst_data_packet(buffer_.data() + packet.offset,
                                                      static_cast<uint32_t>(packet.size));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back(std::move(packet));
        }
        cond_.notify_all();
    }
}

} // namespace uflash
