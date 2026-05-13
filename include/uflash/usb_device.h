#pragma once

#include <libusb.h>
#include <string>
#include <vector>
#include <memory>
#include <optional>

namespace uflash {

struct DeviceInfo {
    uint16_t vid;
    uint16_t pid;
    std::string path;
};

class UsbDevice {
public:
    static void dump_descriptors();
    void clear_halt();
    void reset();
    bool set_baudrate(uint32_t baud);
    void set_debug_io(bool enable) { debug_io_ = enable; }

    static std::optional<std::unique_ptr<UsbDevice>> find_any();

    UsbDevice(libusb_context* ctx,
              libusb_device_handle* handle,
              uint8_t endpoint_in,
              uint8_t endpoint_out,
              int interface);
    ~UsbDevice();

    // Prevent copying
    UsbDevice(const UsbDevice&) = delete;
    UsbDevice& operator=(const UsbDevice&) = delete;

    int write(const uint8_t* data, size_t len, unsigned int timeout_ms = 10000);
    int write_fast(const uint8_t* data, size_t len, unsigned int timeout_ms = 10000);
    int read(uint8_t* data, size_t len, unsigned int timeout_ms = 10000);

private:
    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    uint8_t ep_in_ = 0;
    uint8_t ep_out_ = 0;
    int interface_ = 0;
    bool debug_io_ = false;
};

} // namespace uflash
