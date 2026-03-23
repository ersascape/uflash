#include "uflash/usb_device.h"
#include <iostream>
#include <algorithm>
#include <cstring>
#include <thread>
#include <chrono>

namespace uflash {

static constexpr uint16_t SPD_VID = 0x1782;
static constexpr uint16_t SPD_PIDS[] = { 0x4d00, 0x5d00, 0x3d00 };

UsbDevice::UsbDevice(libusb_device_handle* handle, uint8_t in, uint8_t out, int interface)
    : handle_(handle), ep_in_(in), ep_out_(out), interface_(interface) {
}

UsbDevice::~UsbDevice() {
    if (handle_) {
        libusb_release_interface(handle_, interface_);
        libusb_close(handle_);
    }
}

void UsbDevice::reset() {
    if (handle_) libusb_reset_device(handle_);
}

std::optional<std::unique_ptr<UsbDevice>> UsbDevice::find_any() {
    if (libusb_init(nullptr) < 0) return std::nullopt;

    libusb_device** devs;
    ssize_t count = libusb_get_device_list(nullptr, &devs);
    if (count < 0) return std::nullopt;

    libusb_device_handle* found_handle = nullptr;
    uint8_t found_in = 0, found_out = 0;
    int best_interface = -1;

    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) < 0) continue;

        bool match = false;
        if (desc.idVendor == SPD_VID) {
            for (auto pid : SPD_PIDS) {
                if (desc.idProduct == pid) {
                    match = true;
                    break;
                }
            }
        }

        if (match) {
            if (libusb_open(devs[i], &found_handle) == 0) {
                // Explicitly set configuration 1
                libusb_set_configuration(found_handle, 1);

                libusb_config_descriptor* config;
                libusb_get_config_descriptor(devs[i], 0, &config);
                
                bool endpoints_found = false;
                for (int j = 0; j < config->bNumInterfaces; ++j) {
                    const libusb_interface* intf = &config->interface[j];
                    const libusb_interface_descriptor* alt = &intf->altsetting[0];
                    
                    std::cout << "  Interface " << j << " (Class=0x" << std::hex << (int)alt->bInterfaceClass 
                              << ", SubClass=0x" << (int)alt->bInterfaceSubClass << std::dec << ") has " 
                              << (int)alt->bNumEndpoints << " endpoints\n";
                    for (int e = 0; e < alt->bNumEndpoints; ++e) {
                         auto& ep = alt->endpoint[e];
                         std::cout << "    EP 0x" << std::hex << (int)ep.bEndpointAddress 
                                   << " Type=" << (int)(ep.bmAttributes & 0x03) << std::dec << "\n";
                    }

                    uint8_t tmp_in = 0, tmp_out = 0;
                    for (int e = 0; e < alt->bNumEndpoints; ++e) {
                        auto& ep = alt->endpoint[e];
                        if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK) {
                            if (ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) tmp_in = ep.bEndpointAddress;
                            else tmp_out = ep.bEndpointAddress;
                        }
                    }

                    if (tmp_in && tmp_out) {
                        libusb_set_auto_detach_kernel_driver(found_handle, 1);
                        
                        int current_cfg = 0;
                        libusb_get_configuration(found_handle, &current_cfg);
                        if (current_cfg != 1) {
                            libusb_set_configuration(found_handle, 1);
                        }

                        if (libusb_claim_interface(found_handle, j) == 0) {
                            std::cout << "Found Unisoc Device [ " 
                                      << std::hex << desc.idVendor << ":" << desc.idProduct 
                                      << std::dec << " ] on interface " << j << "\n";
                            std::cout << "  Endpoints: IN=0x" << std::hex << (int)tmp_in 
                                      << ", OUT=0x" << (int)tmp_out << std::dec << "\n";
                            
                            libusb_set_interface_alt_setting(found_handle, j, 0);

                            // Set DTR/RTS HIGH (0x03)
                            libusb_control_transfer(found_handle, 0x21, 34, 0x03, j, nullptr, 0, 1000);
                            
                            // Conservative settle after claiming the interface.
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));

                            best_interface = j;
                            found_in = tmp_in;
                            found_out = tmp_out;
                            endpoints_found = true;
                            break;
                        }
                    }
                }
                
                libusb_free_config_descriptor(config);
                if (endpoints_found) break;

                libusb_close(found_handle);
                found_handle = nullptr;
            }
        }
    }

    libusb_free_device_list(devs, 1);

    if (found_handle) {
        return std::make_unique<UsbDevice>(found_handle, found_in, found_out, best_interface);
    }
    return std::nullopt;
}

int UsbDevice::write(const uint8_t* data, size_t len, unsigned int timeout_ms) {
    if (debug_io_) {
        std::cout << "usb: write len=" << len << " timeout=" << timeout_ms << "ms\n";
    }
    constexpr size_t kBulkSlice = 4096;
    const bool large_transfer = len > kBulkSlice;
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
                if (large_transfer) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
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

void UsbDevice::dump_descriptors() {
    if (libusb_init(nullptr) < 0) return;

    libusb_device** devs;
    ssize_t count = libusb_get_device_list(nullptr, &devs);
    if (count < 0) return;

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
}

} // namespace uflash
