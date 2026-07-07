#include "uflash/usb_device.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace uflash {
namespace {

static constexpr uint16_t SPD_VID = 0x1782;
static constexpr uint16_t SPD_PIDS[] = { 0x4d00, 0x5d00, 0x3d00 };

bool is_supported_pid(uint16_t pid) {
    return std::find(std::begin(SPD_PIDS), std::end(SPD_PIDS), pid) != std::end(SPD_PIDS);
}

struct EndpointSelection {
    uint8_t ep_in = 0;
    uint8_t ep_out = 0;
    int interface_number = -1;
    int alt_setting = -1;
};

std::optional<EndpointSelection> find_bulk_endpoints(const libusb_config_descriptor& config) {
    for (uint8_t iface_index = 0; iface_index < config.bNumInterfaces; ++iface_index) {
        const libusb_interface& intf = config.interface[iface_index];
        for (int alt_index = 0; alt_index < intf.num_altsetting; ++alt_index) {
            const libusb_interface_descriptor& alt = intf.altsetting[alt_index];
            uint8_t ep_in = 0;
            uint8_t ep_out = 0;
            for (uint8_t endpoint_index = 0; endpoint_index < alt.bNumEndpoints; ++endpoint_index) {
                const libusb_endpoint_descriptor& ep = alt.endpoint[endpoint_index];
                if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) {
                    continue;
                }
                if ((ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
                    ep_in = ep.bEndpointAddress;
                } else {
                    ep_out = ep.bEndpointAddress;
                }
            }
            if (ep_in != 0 && ep_out != 0) {
                return EndpointSelection{ep_in, ep_out, alt.bInterfaceNumber, alt.bAlternateSetting};
            }
        }
    }
    return std::nullopt;
}

bool prepare_handle_for_interface(libusb_device_handle* handle, const EndpointSelection& selection) {
    // Enable auto-detach so libusb handles driver detachment during claim.
    libusb_set_auto_detach_kernel_driver(handle, 1);

    // Explicit detach for Linux (belt-and-suspenders; no-op / NOT_SUPPORTED on macOS).
    // LIBUSB_ERROR_NOT_FOUND means no driver was bound — fine.
    int dr = libusb_detach_kernel_driver(handle, selection.interface_number);
    // NOT_FOUND: no driver was bound (fine on any OS).
    // NOT_SUPPORTED / ACCESS: macOS — IOKit does not support explicit detach,
    // auto-detach + set_configuration handles it instead.  Both are silent.
    if (dr != 0 && dr != LIBUSB_ERROR_NOT_FOUND
                && dr != LIBUSB_ERROR_NOT_SUPPORTED
                && dr != LIBUSB_ERROR_ACCESS) {
        std::cerr << "usb: detach kernel driver: " << libusb_error_name(dr) << "\n";
    }

    // On macOS, SET_CONFIGURATION(1) even when the device is already in
    // config 1 causes IOKit to temporarily release any bound interface
    // drivers (e.g. AppleUSBCDCACMData), opening a window for
    // claim_interface to succeed.  On Linux this sends a real USB
    // SET_CONFIGURATION request to the device; for some BROMs this
    // triggers a full interface reinitialisation, leaving bulk endpoints
    // temporarily unavailable.  Skip it on Linux when the device is
    // already in the correct configuration.
#ifdef __APPLE__
    libusb_set_configuration(handle, 1);
#else
    {
        int cur_cfg = 0;
        if (libusb_get_configuration(handle, &cur_cfg) != 0 || cur_cfg != 1) {
            libusb_set_configuration(handle, 1);
        }
    }
#endif

    int cr = libusb_claim_interface(handle, selection.interface_number);
    if (cr != 0) {
        std::cerr << "usb: claim interface " << selection.interface_number
                  << " failed: " << libusb_error_name(cr) << "\n";
        return false;
    }

    // SET_INTERFACE to alt 0: on macOS this initialises the IOKit interface
    // service before bulk transfers can be issued.  Ignore errors; a BROM
    // that does not implement this request will stall ep0 but the bulk
    // endpoints remain functional.
    libusb_set_interface_alt_setting(handle,
                                     selection.interface_number,
                                     selection.alt_setting);

    // SET_CONTROL_LINE_STATE (DTR|RTS): Unisoc BROMs that enumerate as CDC
    // ACM require this before the bulk OUT endpoint will accept data.  This
    // was the original code path that made macOS work; removing it causes
    // every subsequent bulk write to time out.  Ignore the return value;
    // vendor-class interfaces will NAK or STALL the control transfer, which
    // does not affect the bulk endpoints.
    libusb_control_transfer(handle,
                            0x21,   // bmRequestType: Class | Interface | H2D
                            34,     // bRequest: SET_CONTROL_LINE_STATE
                            0x03,   // wValue: DTR | RTS
                            static_cast<uint16_t>(selection.interface_number),
                            nullptr,
                            0,
                            1000);

    // On macOS 15+ AppleUSBCDCACMData can leave the bulk OUT endpoint halted
    // after it is force-detached, and SET_CONTROL_LINE_STATE on a BROM that
    // does not implement the request may re-stall it.  A single clear here
    // after all control transfers ensures the OUT endpoint accepts the first
    // bulk write.  Errors are benign — not all BROMs support CLEAR_FEATURE.
    libusb_clear_halt(handle, selection.ep_out);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

} // namespace

UsbDevice::UsbDevice(libusb_context* ctx,
                     libusb_device_handle* handle,
                     uint8_t in,
                     uint8_t out,
                     int interface)
    : ctx_(ctx), handle_(handle), ep_in_(in), ep_out_(out), interface_(interface) {}

UsbDevice::~UsbDevice() {
    if (handle_ != nullptr) {
        if (interface_ >= 0) {
            libusb_release_interface(handle_, interface_);
        }
        libusb_close(handle_);
    }
    if (ctx_ != nullptr) {
        libusb_exit(ctx_);
    }
}

void UsbDevice::reset() {
    if (handle_) libusb_reset_device(handle_);
}

std::optional<std::unique_ptr<UsbDevice>> UsbDevice::find_any() {
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) {
        return std::nullopt;
    }

    libusb_device** devs;
    ssize_t count = libusb_get_device_list(ctx, &devs);
    if (count < 0) {
        libusb_exit(ctx);
        return std::nullopt;
    }

    libusb_device_handle* found_handle = nullptr;
    EndpointSelection found_selection;

    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) < 0) {
            continue;
        }
        if (desc.idVendor != SPD_VID || !is_supported_pid(desc.idProduct)) {
            continue;
        }

        libusb_config_descriptor* config = nullptr;
        if (libusb_get_active_config_descriptor(devs[i], &config) != 0) {
            if (libusb_get_config_descriptor(devs[i], 0, &config) != 0) {
                continue;
            }
        }

        auto selection = find_bulk_endpoints(*config);
        libusb_free_config_descriptor(config);
        if (!selection) {
            continue;
        }

        if (libusb_open(devs[i], &found_handle) != 0) {
            found_handle = nullptr;
            continue;
        }

        if (!prepare_handle_for_interface(found_handle, *selection)) {
            libusb_close(found_handle);
            found_handle = nullptr;
            continue;
        }

        found_selection = *selection;
        std::cout << "Found Unisoc Device [ "
                  << std::hex << desc.idVendor << ":" << desc.idProduct << std::dec
                  << " ] on interface " << found_selection.interface_number << "\n";
        std::cout << "  Endpoints: IN=0x" << std::hex << static_cast<int>(found_selection.ep_in)
                  << ", OUT=0x" << static_cast<int>(found_selection.ep_out) << std::dec << "\n";
        break;
    }

    libusb_free_device_list(devs, 1);

    if (found_handle != nullptr) {
        return std::make_unique<UsbDevice>(ctx,
                                           found_handle,
                                           found_selection.ep_in,
                                           found_selection.ep_out,
                                           found_selection.interface_number);
    }
    libusb_exit(ctx);
    return std::nullopt;
}

std::optional<std::unique_ptr<UsbDevice>> UsbDevice::wait_for_any(int timeout_s) {
    using clock = std::chrono::steady_clock;

    auto dev = find_any();
    if (dev) return dev;

    if (timeout_s == 0) return std::nullopt;

    auto deadline = timeout_s < 0
        ? clock::time_point::max()
        : clock::now() + std::chrono::seconds(timeout_s);

    std::cout << "Waiting for Unisoc device...\n";

    while (clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        dev = find_any();
        if (dev) return dev;
    }

    return std::nullopt;
}

int UsbDevice::write(const uint8_t* data, size_t len, unsigned int timeout_ms) {
    if (debug_io_) {
        std::cout << "usb: write len=" << len << " timeout=" << timeout_ms << "ms\n";
    }
    constexpr size_t kBulkSlice = 4096;
    int r = 0;

    for (int retry = 0; retry < 3; ++retry) {
        size_t total_transferred = 0;
        bool failed = false;

        while (total_transferred < len) {
            const size_t remaining = len - total_transferred;
            const size_t slice = std::min(kBulkSlice, remaining);
            int transferred = 0;
            unsigned int cur_timeout = timeout_ms * (retry + 1);

            r = libusb_bulk_transfer(
                handle_,
                ep_out_,
                const_cast<uint8_t*>(data + total_transferred),
                static_cast<int>(slice),
                &transferred,
                cur_timeout);

            if (r == 0 && transferred == static_cast<int>(slice)) {
                total_transferred += slice;
                continue;
            }

            if (r == 0 && transferred > 0) {
                total_transferred += static_cast<size_t>(transferred);
                std::cerr << "debug: short write " << transferred << "/" << slice
                          << " within packet, retry " << (retry + 1) << "/3...\n";
                r = LIBUSB_ERROR_TIMEOUT;
            }

            failed = true;
            break;
        }

        if (!failed && total_transferred == len) {
            return static_cast<int>(total_transferred);
        }

        if (r != LIBUSB_ERROR_TIMEOUT) {
            break;
        }

        std::cerr << "debug: write timeout, retry " << (retry + 1) << "/3...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(20 * (retry + 1)));
    }

    std::cerr << "debug: write error " << r << " (" << libusb_error_name(r) << ")\n";
    return r;
}

int UsbDevice::write_fast(const uint8_t* data, size_t len, unsigned int timeout_ms) {
    if (debug_io_) {
        std::cout << "usb: write_fast len=" << len << " timeout=" << timeout_ms << "ms\n";
    }
    int r = 0;
    for (int retry = 0; retry < 2; ++retry) {
        int transferred = 0;
        unsigned int cur_timeout = timeout_ms * (retry + 1);
        r = libusb_bulk_transfer(
            handle_,
            ep_out_,
            const_cast<uint8_t*>(data),
            static_cast<int>(len),
            &transferred,
            cur_timeout);

        if (r == 0 && transferred == static_cast<int>(len)) {
            return transferred;
        }

        if (r == 0 && transferred > 0 && transferred < static_cast<int>(len)) {
            std::cerr << "debug: fast short write " << transferred << "/" << len
                      << ", retry " << (retry + 1) << "/2...\n";
            r = LIBUSB_ERROR_TIMEOUT;
        }

        if (r != LIBUSB_ERROR_TIMEOUT) {
            break;
        }

        std::cerr << "debug: fast write timeout, retry " << (retry + 1) << "/2...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10 * (retry + 1)));
    }

    std::cerr << "debug: fast write error " << r << " (" << libusb_error_name(r) << ")\n";
    return r;
}

int UsbDevice::read(uint8_t* data, size_t len, unsigned int timeout_ms) {
    int transferred = 0;
    int r = 0;
    for (int retry = 0; retry < 3; ++retry) {
        r = libusb_bulk_transfer(handle_, ep_in_, data, 
                                     static_cast<int>(len), &transferred, timeout_ms);
        if (r == 0) {
            if (debug_io_ && transferred > 0) {
                std::cout << "usb: read len=" << transferred << " timeout=" << timeout_ms << "ms\n";
            }
            return transferred;
        }
        if (r != LIBUSB_ERROR_TIMEOUT) break;
        // Don't spam read timeouts during handshake
    }
    if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) {
        std::cerr << "debug: read error " << r << " (" << libusb_error_name(r) << ")\n";
    }
    return (r == 0) ? transferred : r;
}

bool UsbDevice::set_baudrate(uint32_t baud) {
    if (!handle_) return false;
    
    // CDC Line Coding structure (7 bytes)
    // baud (4), stop (1), parity (1), data (1)
    uint8_t coding[7];
    coding[0] = (baud >> 0) & 0xFF;
    coding[1] = (baud >> 8) & 0xFF;
    coding[2] = (baud >> 16) & 0xFF;
    coding[3] = (baud >> 24) & 0xFF;
    coding[4] = 0; // 1 stop bit
    coding[5] = 0; // No parity
    coding[6] = 8; // 8 data bits
    
    // 0x21: LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT
    // 0x20: SET_LINE_CODING
    int r = libusb_control_transfer(handle_, 0x21, 0x20, 0, interface_, coding, sizeof(coding), 1000);
    return (r >= 0);
}

void UsbDevice::clear_halt() {
    if (handle_) {
        libusb_clear_halt(handle_, ep_in_);
        libusb_clear_halt(handle_, ep_out_);
    }
}

void UsbDevice::drain(unsigned int timeout_ms) {
    if (!handle_) return;
    uint8_t buf[4096];
    int transferred = 0;
    // Read until the endpoint returns no data or any error (including IO).
    // IO errors here just mean the endpoint isn't ready yet — not a fault.
    while (libusb_bulk_transfer(handle_, ep_in_, buf, sizeof(buf),
                                &transferred, timeout_ms) == 0 && transferred > 0) {}
}

void UsbDevice::dump_descriptors() {
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) return;

    libusb_device** devs;
    ssize_t count = libusb_get_device_list(ctx, &devs);
    if (count < 0) {
        libusb_exit(ctx);
        return;
    }

    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) < 0) continue;

        if (desc.idVendor == SPD_VID) {
            std::cout << "Device: " << std::hex << desc.idVendor << ":" << desc.idProduct << std::dec << "\n";
            libusb_config_descriptor* config;
            libusb_get_config_descriptor(devs[i], 0, &config);
            
            for (int j = 0; j < config->bNumInterfaces; ++j) {
                const libusb_interface* intf = &config->interface[j];
                std::cout << "  Interface " << j << " (AltSettings: " << intf->num_altsetting << ")\n";
                for (int a = 0; a < intf->num_altsetting; ++a) {
                    const libusb_interface_descriptor* alt = &intf->altsetting[a];
                    std::cout << "    Alt " << a << ": Class=0x" << std::hex << (int)alt->bInterfaceClass << std::dec 
                              << ", Endpoints=" << (int)alt->bNumEndpoints << "\n";
                    for (int e = 0; e < alt->bNumEndpoints; ++e) {
                        auto& ep = alt->endpoint[e];
                        std::cout << "      EP 0x" << std::hex << (int)ep.bEndpointAddress << std::dec 
                                  << ": Type=" << (int)(ep.bmAttributes & 0x03) 
                                  << ", MaxPacket=" << ep.wMaxPacketSize << "\n";
                    }
                }
            }
            libusb_free_config_descriptor(config);
        }
    }
    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
}

} // namespace uflash
