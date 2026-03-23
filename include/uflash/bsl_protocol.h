#pragma once

#include <vector>
#include <cstdint>
#include <string>

namespace uflash {

class UsbDevice;

enum BslCommand : uint16_t {
    BSL_CMD_CONNECT = 0x00,
    BSL_CMD_START_DATA = 0x01,
    BSL_CMD_MIDST_DATA = 0x02,
    BSL_CMD_END_DATA = 0x03,
    BSL_CMD_EXEC_DATA = 0x04,
    BSL_CMD_NORMAL_RESET = 0x05,  // Reset to normal mode
    BSL_CMD_ERASE_FLASH = 0x06,   // Erase flash by address range
    BSL_CMD_CHANGE_BAUD = 0x09,
    BSL_CMD_REPARTITION = 0x0B,   // Repartition nand/emmc flash
    BSL_CMD_READ_START = 0x10,    // Start reading a partition
    BSL_CMD_READ_MIDST = 0x11,    // Read next chunk
    BSL_CMD_READ_END = 0x12,      // End read
    BSL_CMD_READ_CHIP_UID = 0x1A,
    BSL_CMD_DISABLE_TRANSCODE = 0x21,
    BSL_CMD_CHECK_BAUD = 0x7E,
    BSL_CMD_HOST_CONNECT = 0xAE,
};

enum BslResponse : uint16_t {
    BSL_REP_ACK = 0x80,
    BSL_REP_BSL_VER = 0x81,
    BSL_REP_INVALID_CMD = 0x82,
    BSL_REP_UNKWN_CHIP = 0x83,
    BSL_REP_VERIFY_ERROR = 0x8B,
    BSL_REP_VERIFY_SUCCESS = 0x85,
};

// Host Protocol (0xAE) definitions
namespace host {
    static constexpr uint8_t TAG = 0xAE;
    static constexpr uint8_t FLOWID = 0xFF;

    enum Command : uint32_t {
        CMD_CONNECT = 0x00,
        CMD_START_DATA = 0x04,
        CMD_MID_DATA = 0x05,
        CMD_END_DATA = 0x06,
        CMD_EXEC_DATA = 0x07
    };

    #pragma pack(push, 1)
    struct PacketHeader {
        uint8_t  tag;       // 0xAE
        uint32_t data_size; // size of DataHeader + payload
        uint8_t  flow_id;   // 0xFF
        uint16_t reserved;  // 0x0000
    };

    struct DataHeader {
        uint32_t cmd_type;
        uint32_t addr;
        uint32_t size;
    };
    #pragma pack(pop)
}

class BslProtocol {
public:
    explicit BslProtocol(UsbDevice& dev);

    // Perform the initial 0x7E handshake
    bool handshake(int timeout_ms = 5000);
    bool connect();
    std::string get_last_handshake_response();

    // Perform the modern 0xAE handshake
    bool host_handshake(int timeout_ms = 5000);

    // Send a command and receive response
    bool send_command(BslCommand cmd, const std::vector<uint8_t>& payload = {}, int timeout_ms = 10000);
    bool receive_packet(uint16_t& out_type, std::vector<uint8_t>& out_payload, int timeout_ms = 1000);

    // Helper: read version string
    std::string get_version();

    void set_use_checksum(bool enable);
    void set_debug_protocol(bool enable) { debug_protocol_ = enable; }

    // FDL Streaming
    bool start_data(uint32_t address, uint32_t size, uint32_t checksum = 0);
    bool start_data(uint64_t address, uint64_t size, uint32_t checksum = 0);
    bool start_data(const std::string& name, uint64_t size, uint32_t checksum = 0, bool use_64bit = false);
    bool midst_data(const uint8_t* data, uint32_t size);
    std::vector<uint8_t> build_midst_data_packet(const uint8_t* data, uint32_t size);
    bool send_framed_packet(const std::vector<uint8_t>& frame, int write_timeout_ms = 30000, int ack_timeout_ms = 15000);
    bool send_framed_packet_fast(const std::vector<uint8_t>& frame, int write_timeout_ms = 30000, int ack_timeout_ms = 15000);
    bool end_data(int timeout_ms = 30000);
    bool exec_data(uint32_t address);
    bool change_baud(uint32_t baud, int timeout_ms = 2000);
    bool disable_transcode(int timeout_ms = 1000);
    bool transcode_disabled() const { return disable_transcode_; }
    bool transcode_supported() const { return transcode_supported_; }
    uint16_t last_connect_type() const { return last_connect_type_; }
    const std::vector<uint8_t>& last_connect_payload() const { return last_connect_payload_; }
    bool repartition(const std::vector<uint8_t>& partition_table); // BSL_CMD_REPARTITION (0x0B)

    // Read partition data from device
    // Returns true on success, fills out_data with partition contents
    bool read_partition(const std::string& name, uint64_t size, std::vector<uint8_t>& out_data, bool use_64bit = false);

    void clear_halts();    // clear USB endpoint stalls after partition failures
    void drain_recv();     // drain stale IN packets after FDL2 error state

private:
    UsbDevice& dev_;
    std::vector<uint8_t> last_handshake_response_;
    bool use_checksum_ = false;
    bool disable_transcode_ = false;
    bool transcode_supported_ = false;
    bool debug_protocol_ = false;
    uint16_t last_connect_type_ = 0;
    std::vector<uint8_t> last_connect_payload_;

    // HDLC Framing
    std::vector<uint8_t> frame_packet(uint16_t type, const std::vector<uint8_t>& payload);
    std::vector<uint8_t> frame_packet(uint16_t type, const uint8_t* payload, size_t payload_size);
    bool unframe_packet(const std::vector<uint8_t>& frame, uint16_t& out_type, std::vector<uint8_t>& out_payload);
};

} // namespace uflash
