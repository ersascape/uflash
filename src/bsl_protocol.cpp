#include "uflash/bsl_protocol.h"
#include "uflash/usb_device.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace {
    uint16_t bsl_crc16(uint16_t crc, const uint8_t* buf, uint32_t len) {
        for (uint32_t i = 0; i < len; ++i) {
            uint8_t data = buf[i];
            for (int bit = 0x80; bit != 0; bit >>= 1) {
                bool carry = (crc & 0x8000) != 0;
                crc <<= 1;
                if (carry) crc ^= 0x1021;
                if (data & bit) crc ^= 0x1021;
            }
        }
        return crc;
    }

    uint16_t bsl_checksum(const uint8_t* buf, uint32_t len) {
        uint32_t sum = 0;
        for (uint32_t i = 0; i < len; i += 2) {
            if (i + 1 < len) {
                sum += (buf[i] << 8) | buf[i+1];
            } else {
                sum += (buf[i] << 8);
            }
        }
        sum = (sum >> 16) + (sum & 0xFFFF);
        sum += (sum >> 16);
        return (uint16_t)(~sum);
    }

    uint32_t read_le32(const uint8_t* p) {
        return static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
    }

    std::string hex_preview_local(const std::vector<uint8_t>& data, size_t max_bytes = 16) {
        std::ostringstream out;
        out << std::hex << std::setfill('0');
        size_t count = std::min(max_bytes, data.size());
        for (size_t i = 0; i < count; ++i) {
            if (i != 0) out << ' ';
            out << std::setw(2) << static_cast<unsigned>(data[i]);
        }
        if (data.size() > max_bytes) out << " ...";
        return out.str();
    }

    bool maybe_parse_da_info(const std::vector<uint8_t>& payload, bool& transcode_supported) {
        // Vendor DA_INFO_T is surfaced on some replies, most importantly the
        // 0x96 / 0xFE incompatible-partition reply from ExecNandInit. Version 1 uses
        // the first 8 bytes; later FDLs may return larger versioned variants,
        // but the disable-transcode flag is still in the second DWORD.
        if (payload.size() < 8) {
            return false;
        }
        uint32_t version = read_le32(payload.data());
        if (version != 1 && version != 2 && version != 4) {
            return false;
        }
        uint32_t disable_flag = read_le32(payload.data() + 4);
        transcode_supported = disable_flag != 0;
        return true;
    }
}

namespace uflash {

static constexpr uint8_t BSL_FRAME_TAG = 0x7E;
static constexpr uint8_t BSL_ESCAPE_TAG = 0x7D;

BslProtocol::BslProtocol(UsbDevice& dev) : dev_(dev), use_checksum_(false) {}

void BslProtocol::set_use_checksum(bool enable) {
    use_checksum_ = enable;
}

std::vector<uint8_t> BslProtocol::frame_packet(uint16_t type, const std::vector<uint8_t>& payload) {
    return frame_packet(type, payload.data(), payload.size());
}

std::vector<uint8_t> BslProtocol::frame_packet(uint16_t type, const uint8_t* payload, size_t payload_size) {
    std::vector<uint8_t> raw;
    raw.push_back(static_cast<uint8_t>(type >> 8));
    raw.push_back(static_cast<uint8_t>(type & 0xFF));
    raw.push_back(static_cast<uint8_t>(payload_size >> 8));
    raw.push_back(static_cast<uint8_t>(payload_size & 0xFF));
    if (payload != nullptr && payload_size != 0) {
        raw.insert(raw.end(), payload, payload + payload_size);
    }
    
    uint16_t computed_crc = bsl_crc16(0, raw.data(), raw.size());
    uint16_t computed_sum = bsl_checksum(raw.data(), raw.size());
    uint16_t check = use_checksum_ ? computed_sum : computed_crc;
    
    raw.push_back(static_cast<uint8_t>(check >> 8));
    raw.push_back(static_cast<uint8_t>(check & 0xFF));

    std::vector<uint8_t> frame;
    frame.push_back(BSL_FRAME_TAG);
    if (disable_transcode_) {
        frame.insert(frame.end(), raw.begin(), raw.end());
    } else {
        for (uint8_t b : raw) {
            if (b == BSL_FRAME_TAG || b == BSL_ESCAPE_TAG) {
                frame.push_back(BSL_ESCAPE_TAG);
                frame.push_back(b ^ 0x20);
            } else {
                frame.push_back(b);
            }
        }
    }
    frame.push_back(BSL_FRAME_TAG);
    return frame;
}

bool BslProtocol::handshake(int timeout_ms) {
    // Conservative flush; some devices need a wider quiet window before they
    // start accepting the handshake burst reliably.
    uint8_t garbage[4096];
    while (dev_.read(garbage, sizeof(garbage), 20) > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    while (dev_.read(garbage, sizeof(garbage), 20) > 0);

    std::cout << "Sending 0x7E handshake (32-byte burst)...\n";
    std::vector<uint8_t> burst(32, BSL_FRAME_TAG);
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        dev_.write(burst.data(), burst.size(), 500);
        
        uint8_t resp[1024];
        int r = dev_.read(resp, sizeof(resp), 500);
        if (r > 0) {
            for (int i = 0; i < r; ++i) {
                if (resp[i] == BSL_FRAME_TAG && (i + 1 < r)) {
                    // Check if it's the start of a version string (usually 0x00 0x81 or 0x81 0x00)
                    std::cout << "Handshake successful! (RX: " << r << " bytes)\n";
                    last_handshake_response_.assign(resp, resp + r);
                    return true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

bool BslProtocol::connect() {
    uint16_t type;
    std::vector<uint8_t> payload;
    if (!send_command(BslCommand::BSL_CMD_CONNECT)) return false;
    if (!receive_packet(type, payload)) return false;
    last_connect_type_ = type;
    last_connect_payload_ = payload;
    return type == BslResponse::BSL_REP_ACK;
}

std::string BslProtocol::get_last_handshake_response() {
    std::string s;
    for (uint8_t b : last_handshake_response_) {
        if (b >= 32 && b < 127) s += (char)b;
    }
    return s;
}

bool BslProtocol::send_command(BslCommand cmd, const std::vector<uint8_t>& payload, int timeout_ms) {
    auto frame = frame_packet(static_cast<uint16_t>(cmd), payload);
    if (debug_protocol_) {
        std::cout << "bsl: tx cmd=0x" << std::hex << static_cast<unsigned>(cmd) << std::dec
                  << " payload_len=" << payload.size()
                  << " frame_len=" << frame.size();
        if (!payload.empty()) {
            std::cout << " payload_hex=" << hex_preview_local(payload);
        }
        std::cout << "\n";
    }
    return dev_.write(frame.data(), frame.size(), timeout_ms) == static_cast<int>(frame.size());
}

std::vector<uint8_t> BslProtocol::build_midst_data_packet(const uint8_t* data, uint32_t size) {
    return frame_packet(static_cast<uint16_t>(BslCommand::BSL_CMD_MIDST_DATA), data, size);
}

bool BslProtocol::send_framed_packet(const std::vector<uint8_t>& frame, int write_timeout_ms, int ack_timeout_ms) {
    if (debug_protocol_) {
        std::cout << "bsl: tx framed len=" << frame.size()
                  << " write_timeout=" << write_timeout_ms
                  << " ack_timeout=" << ack_timeout_ms << "\n";
    }
    if (dev_.write(frame.data(), frame.size(), write_timeout_ms) != static_cast<int>(frame.size())) {
        return false;
    }
    uint16_t type;
    std::vector<uint8_t> out;
    return receive_packet(type, out, ack_timeout_ms) && type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::send_framed_packet_fast(const std::vector<uint8_t>& frame, int write_timeout_ms, int ack_timeout_ms) {
    if (debug_protocol_) {
        std::cout << "bsl: tx fast-framed len=" << frame.size()
                  << " write_timeout=" << write_timeout_ms
                  << " ack_timeout=" << ack_timeout_ms << "\n";
    }
    if (dev_.write_fast(frame.data(), frame.size(), write_timeout_ms) != static_cast<int>(frame.size())) {
        return false;
    }
    uint16_t type;
    std::vector<uint8_t> out;
    return receive_packet(type, out, ack_timeout_ms) && type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::receive_packet(uint16_t& out_type, std::vector<uint8_t>& out_payload, int timeout_ms) {
    std::vector<uint8_t> buffer;
    uint8_t chunk[8192];
    bool in_frame = false;
    int bytes_needed = -1;
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        int r = dev_.read(chunk, sizeof(chunk), 100);
        if (r > 0) {
            for (int i = 0; i < r; ++i) {
                uint8_t b = chunk[i];
                if (disable_transcode_) {
                    if (b == BSL_FRAME_TAG) {
                        if (!in_frame) {
                            in_frame = true;
                            bytes_needed = -1;
                            buffer.clear();
                        } else if (!buffer.empty()) {
                            if (bytes_needed == 0) {
                                if (buffer.size() < 6) return false;
                                out_type = (buffer[0] << 8) | buffer[1];
                                uint16_t len = (buffer[2] << 8) | buffer[3];
                                if (buffer.size() != static_cast<size_t>(6 + len)) return false;
                                out_payload.assign(buffer.begin() + 4, buffer.begin() + 4 + len);
                                uint16_t stored_check = (buffer[4 + len] << 8) | buffer[5 + len];
                                uint16_t computed_crc = bsl_crc16(0, buffer.data(), 4 + len);
                                uint16_t computed_sum = bsl_checksum(buffer.data(), 4 + len);
                                bool ok = stored_check == computed_crc || stored_check == computed_sum;
                                if (ok) {
                                    maybe_parse_da_info(out_payload, transcode_supported_);
                                }
                                if (debug_protocol_) {
                                    std::cout << "bsl: rx type=0x" << std::hex << out_type << std::dec
                                              << " payload_len=" << out_payload.size();
                                    if (!out_payload.empty()) {
                                        std::cout << " payload_hex=" << hex_preview_local(out_payload);
                                    }
                                    std::cout << "\n";
                                }
                                return ok;
                            }
                            buffer.clear();
                            bytes_needed = -1;
                        }
                        continue;
                    }

                    if (in_frame) {
                        buffer.push_back(b);
                        if (buffer.size() == 4) {
                            uint16_t len = (buffer[2] << 8) | buffer[3];
                            bytes_needed = static_cast<int>(len) + 2;
                        } else if (bytes_needed > 0) {
                            --bytes_needed;
                        }
                    }
                    continue;
                }

                if (b == BSL_FRAME_TAG) {
                    if (!in_frame) {
                        in_frame = true;
                        buffer.clear();
                    } else if (!buffer.empty()) {
                        std::vector<uint8_t> raw;
                        for (size_t j = 0; j < buffer.size(); ++j) {
                            if (buffer[j] == BSL_ESCAPE_TAG && j + 1 < buffer.size()) {
                                raw.push_back(buffer[j + 1] ^ 0x20);
                                j++;
                            } else {
                                raw.push_back(buffer[j]);
                            }
                        }
                        
                        if (raw.size() < 6) return false;
                        out_type = (raw[0] << 8) | raw[1];
                        uint16_t len = (raw[2] << 8) | raw[3];
                        if (raw.size() < static_cast<size_t>(6 + len)) return false;
                        
                        out_payload.assign(raw.begin() + 4, raw.begin() + 4 + len);
                        uint16_t stored_check = (raw[4 + len] << 8) | raw[5 + len];
                        uint16_t computed_crc = bsl_crc16(0, raw.data(), 4 + len);
                        uint16_t computed_sum = bsl_checksum(raw.data(), 4 + len);
                        
                        bool ok = stored_check == computed_crc || stored_check == computed_sum;
                        if (ok) {
                            maybe_parse_da_info(out_payload, transcode_supported_);
                        }
                        if (debug_protocol_) {
                            std::cout << "bsl: rx type=0x" << std::hex << out_type << std::dec
                                      << " payload_len=" << out_payload.size();
                            if (!out_payload.empty()) {
                                std::cout << " payload_hex=" << hex_preview_local(out_payload);
                            }
                            std::cout << "\n";
                        }
                        return ok;
                    }
                } else if (in_frame) {
                    buffer.push_back(b);
                }
            }
        }
    }
    return false;
}

std::string BslProtocol::get_version() {
    if (!send_command(BslCommand::BSL_CMD_START_DATA)) return "";
    uint16_t type;
    std::vector<uint8_t> payload;
    if (receive_packet(type, payload)) {
        if (type == BslResponse::BSL_REP_BSL_VER) {
            return std::string(payload.begin(), payload.end());
        }
    }
    return "";
}

bool BslProtocol::host_handshake(int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    uint8_t garbage[1024];
    while (dev_.read(garbage, sizeof(garbage), 50) > 0);

    host::PacketHeader ph;
    ph.tag = host::TAG;
    uint32_t ds = sizeof(host::DataHeader);
    ph.data_size = ((ds & 0xFF) << 24) | ((ds & 0xFF00) << 8) | ((ds & 0xFF0000) >> 8) | ((ds & 0xFF000000) >> 24);
    ph.flow_id = host::FLOWID;
    ph.reserved = 0;

    host::DataHeader dh;
    dh.cmd_type = 0; dh.addr = 0; dh.size = 0;

    uint8_t pkt[20];
    std::memcpy(pkt, &ph, 8);
    std::memcpy(pkt + 8, &dh, 12);

    std::cout << "Sending 0xAE Host Protocol Connect...\n";
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        if (dev_.write(pkt, 8, 1000) == 8 && dev_.write(pkt + 8, 12, 1000) == 12) {
             uint8_t resp[64];
             int r = dev_.read(resp, sizeof(resp), 1000);
             if (r > 0 && resp[0] == host::TAG) {
                 std::cout << "Host Protocol Response 0xAE detected!\n";
                 return true;
             }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

bool BslProtocol::start_data(uint32_t address, uint32_t size, uint32_t checksum) {
    std::vector<uint8_t> payload(checksum ? 12 : 8);
    payload[0] = (address >> 24) & 0xFF; payload[1] = (address >> 16) & 0xFF;
    payload[2] = (address >> 8) & 0xFF;  payload[3] = address & 0xFF;
    payload[4] = (size >> 24) & 0xFF;    payload[5] = (size >> 16) & 0xFF;
    payload[6] = (size >> 8) & 0xFF;     payload[7] = size & 0xFF;
    if (checksum) {
        payload[8] = (checksum >> 24) & 0xFF; payload[9] = (checksum >> 16) & 0xFF;
        payload[10] = (checksum >> 8) & 0xFF; payload[11] = checksum & 0xFF;
    }
    uint16_t type;
    std::vector<uint8_t> out;
    if (!send_command(BslCommand::BSL_CMD_START_DATA, payload)) return false;
    return receive_packet(type, out) && type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::start_data(uint64_t address, uint64_t size, uint32_t checksum) {
    std::vector<uint8_t> payload(16 + (checksum ? 4 : 0));
    // Unisoc 64-bit is typically Big Endian in BSL as well
    for (int i=0; i<8; ++i) payload[i] = (address >> ((7-i)*8)) & 0xFF;
    for (int i=0; i<8; ++i) payload[8+i] = (size >> ((7-i)*8)) & 0xFF;
    if (checksum) {
        for (int i=0; i<4; ++i) payload[16+i] = (checksum >> ((3-i)*8)) & 0xFF;
    }
    uint16_t type;
    std::vector<uint8_t> out;
    if (!send_command(BslCommand::BSL_CMD_START_DATA, payload)) return false;
    if (!receive_packet(type, out)) return false;
    if (type == 0x96) {
        std::cerr << "    error: Incompatible Partition (0x96). The firmware partition size does not match the device layout.\n";
        return false;
    }
    return type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::start_data(const std::string& name, uint64_t size, uint32_t checksum, bool use_64bit) {
    // DownloadByID:   [NAME:72 UTF-16LE][SIZE:4][CHECKSUM?:4]
    // DownloadByIDEx: [NAME:72 UTF-16LE][SIZE:8][RESERVED:8][CHECKSUM?:4]
    use_64bit = use_64bit || size >= 0x100000000ULL;
    std::vector<uint8_t> payload(72 + (use_64bit ? 16 : 4) + (checksum ? 4 : 0), 0);
    size_t chars = std::min(name.length(), (size_t)36);
    for (size_t i = 0; i < chars; ++i) {
        payload[i * 2]     = (uint8_t)name[i];
        payload[i * 2 + 1] = 0x00;
    }

    if (use_64bit) {
        // Bytes 72-79: size, bytes 80-87: reserved/zero.
        for (int i = 0; i < 8; ++i) payload[72 + i] = (size >> ((7 - i) * 8)) & 0xFF;
    } else {
        uint32_t size32 = static_cast<uint32_t>(size);
        payload[72] = (size32 >> 0) & 0xFF;
        payload[73] = (size32 >> 8) & 0xFF;
        payload[74] = (size32 >> 16) & 0xFF;
        payload[75] = (size32 >> 24) & 0xFF;
    }

    if (checksum) {
        uint8_t* p = payload.data() + 72 + (use_64bit ? 16 : 4);
        p[0] = (checksum >> 24) & 0xFF; p[1] = (checksum >> 16) & 0xFF;
        p[2] = (checksum >> 8) & 0xFF;  p[3] = checksum & 0xFF;
    }
    
    uint16_t type;
    std::vector<uint8_t> out;
    if (!send_command(BslCommand::BSL_CMD_START_DATA, payload)) return false;
    if (!receive_packet(type, out)) return false;
    if (type == 0x96) {
        std::cerr << "    error: Incompatible Partition (0x96). The firmware partition size does not match the device layout.\n";
        return false;
    }
    if (type == 0xFE) {
        std::cerr << "    error: FDL2 returned 0xFE (unsupported). Partition name not found in device table.\n";
        return false;
    }
    return type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::midst_data(const uint8_t* data, uint32_t size) {
    auto frame = build_midst_data_packet(data, size);
    return send_framed_packet(frame, 30000, 60000);
}

bool BslProtocol::end_data(int timeout_ms) {
    if (!send_command(BslCommand::BSL_CMD_END_DATA, {}, timeout_ms)) return false;
    uint16_t type;
    std::vector<uint8_t> out;
    return receive_packet(type, out, timeout_ms) && type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::change_baud(uint32_t baud, int timeout_ms) {
    std::vector<uint8_t> payload(4);
    payload[0] = static_cast<uint8_t>((baud >> 0) & 0xFF);
    payload[1] = static_cast<uint8_t>((baud >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((baud >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((baud >> 24) & 0xFF);
    if (!send_command(BslCommand::BSL_CMD_CHANGE_BAUD, payload, timeout_ms)) return false;
    uint16_t type;
    std::vector<uint8_t> out;
    return receive_packet(type, out, timeout_ms) && type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::disable_transcode(int timeout_ms) {
    bool prev = disable_transcode_;
    disable_transcode_ = false;
    if (!send_command(BslCommand::BSL_CMD_DISABLE_TRANSCODE, {}, timeout_ms)) {
        return false;
    }
    uint16_t type;
    std::vector<uint8_t> out;
    bool ok = receive_packet(type, out, timeout_ms) && type == BslResponse::BSL_REP_ACK;
    if (ok) {
        disable_transcode_ = true;
    } else {
        disable_transcode_ = prev;
    }
    return ok;
}

bool BslProtocol::exec_data(uint32_t address) {
    std::vector<uint8_t> payload(4);
    payload[0] = (address >> 24) & 0xFF; payload[1] = (address >> 16) & 0xFF;
    payload[2] = (address >> 8) & 0xFF;  payload[3] = address & 0xFF;
    if (!send_command(BslCommand::BSL_CMD_EXEC_DATA, payload)) return false;
    uint16_t type;
    std::vector<uint8_t> out;
    return receive_packet(type, out) && type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::repartition(const std::vector<uint8_t>& partition_table) {
    // BSL_CMD_REPARTITION = 0x0B, payload is array of PARTITION_INFO_T structs
    if (!send_command(BslCommand::BSL_CMD_REPARTITION, partition_table)) return false;
    uint16_t type;
    std::vector<uint8_t> out;
    if (!receive_packet(type, out, 60000)) {
        std::cerr << "  debug: repartition receive_packet timed out.\n";
        return false;
    }
    if (type != BslResponse::BSL_REP_ACK) {
        std::cerr << "  debug: repartition failed, FDL2 returned 0x" << std::hex << type << std::dec << "\n";
    }
    return type == BslResponse::BSL_REP_ACK;
}

bool BslProtocol::read_partition(const std::string& name, uint64_t size, std::vector<uint8_t>& out_data, bool use_64bit) {
    // ReadFlash2:    [NAME:72 UTF-16LE][SIZE:4]
    // ReadFlash2_64: [NAME:72 UTF-16LE][SIZE:8][RESERVED:8]
    use_64bit = use_64bit || size >= 0x100000000ULL;
    std::vector<uint8_t> payload(72 + (use_64bit ? 16 : 4), 0);
    size_t chars = std::min(name.length(), (size_t)35);
    for (size_t i = 0; i < chars; ++i) {
        payload[i * 2]     = (uint8_t)name[i];
        payload[i * 2 + 1] = 0x00;
    }
    if (use_64bit) {
        for (int i = 0; i < 8; ++i) payload[72 + i] = ((uint64_t)size >> ((7 - i) * 8)) & 0xFF;
    } else {
        uint32_t size32 = static_cast<uint32_t>(size);
        payload[72] = (size32 >> 0) & 0xFF;
        payload[73] = (size32 >> 8) & 0xFF;
        payload[74] = (size32 >> 16) & 0xFF;
        payload[75] = (size32 >> 24) & 0xFF;
    }

    if (!send_command(BslCommand::BSL_CMD_READ_START, payload)) return false;
    uint16_t type;
    std::vector<uint8_t> resp;
    if (!receive_packet(type, resp, 5000)) return false;
    if (type != BslResponse::BSL_REP_ACK) {
        std::cerr << "    read_partition: READ_START rejected (0x" << std::hex << type << std::dec << ")\n";
        return false;
    }

    // Read chunks via BSL_CMD_READ_MIDST
    out_data.clear();
    out_data.reserve(static_cast<size_t>(std::min<uint64_t>(size, 0x7fffffffULL)));
    const uint32_t CHUNK = 4096;
    uint64_t remaining = size;

    while (remaining > 0) {
        uint32_t to_read = static_cast<uint32_t>(std::min<uint64_t>(CHUNK, remaining));
        std::vector<uint8_t> mid_payload(use_64bit ? 12 : 8, 0);
        uint64_t offset = (uint64_t)size - remaining;
        mid_payload[0] = (to_read >> 0) & 0xFF; mid_payload[1] = (to_read >> 8) & 0xFF;
        mid_payload[2] = (to_read >> 16) & 0xFF; mid_payload[3] = (to_read >> 24) & 0xFF;
        if (use_64bit) {
            for (int i = 0; i < 8; ++i) {
                mid_payload[4 + i] = (offset >> (i * 8)) & 0xFF;
            }
        } else {
            uint32_t offset32 = static_cast<uint32_t>(offset);
            mid_payload[4] = (offset32 >> 0) & 0xFF;
            mid_payload[5] = (offset32 >> 8) & 0xFF;
            mid_payload[6] = (offset32 >> 16) & 0xFF;
            mid_payload[7] = (offset32 >> 24) & 0xFF;
        }

        if (!send_command(BslCommand::BSL_CMD_READ_MIDST, mid_payload)) {
            std::cerr << "    read_partition: send READ_MIDST failed at offset " << offset << "\n";
            return false;
        }

        // FDL2 replies with BSL_REP_READ_FLASH (0x93) containing the data
        if (!receive_packet(type, resp, 5000)) {
            std::cerr << "    read_partition: no reply to READ_MIDST at offset " << offset << "\n";
            return false;
        }
        if (type != 0x93 && type != BslResponse::BSL_REP_ACK) {
            std::cerr << "    read_partition: READ_MIDST unexpected reply 0x" << std::hex << type << std::dec << "\n";
            return false;
        }
        out_data.insert(out_data.end(), resp.begin(), resp.end());
        remaining -= resp.size();
        
        int percent = (int)(((uint64_t)(size - remaining) * 100) / size);
        std::cout << "\r      Progress: " << percent << "%    " << std::flush;
    }
    std::cout << "\n";

    // Send READ_END
    send_command(BslCommand::BSL_CMD_READ_END);
    receive_packet(type, resp, 2000); // ACK, ignore result

    return true;
}

void BslProtocol::clear_halts() {
    dev_.clear_halt();
}

void BslProtocol::drain_recv() {
    // drain any pending packets FDL2 may have queued after an error
    for (int i = 0; i < 8; ++i) {
        uint16_t type;
        std::vector<uint8_t> out;
        if (!receive_packet(type, out, 200)) break; // stop when nothing pending
    }
}

} // namespace uflash
