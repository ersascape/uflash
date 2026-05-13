#include "uflash/flash_workflow.h"

#include "uflash/bsl_protocol.h"
#include "uflash/transfer_pipeline.h"
#include "uflash/usb_device.h"
#include "upac/pac_reader.h"
#include "upac/xml_config.h"

#include <pugixml.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;

namespace uflash {
namespace {

constexpr size_t kDefaultDownloadChunkSize = 0xFC00;
constexpr size_t kCode2ChunkSize = 0x5000;
constexpr size_t kCode264ChunkSize = 0xE000;
constexpr size_t kBootLoader2ChunkSize = 2112;
constexpr size_t kMinAdaptiveChunkSize = 0x400;
constexpr size_t kMinNamedTailChunkSize = 0x20;
constexpr uint64_t kLargeImageThreshold = 8ULL * 1024ULL * 1024ULL; // 8 MB: use fast chunk strategy for medium+ partitions
constexpr size_t kLargeImageNamedChunkSize = 0x8000;
constexpr size_t kLargeImageStartChunkSize = 0x8000;
constexpr size_t kLargeImageMinChunkSize = 0x2000;
constexpr size_t kFastLaneWindowSize = 0x4000000; // 64 MB — sync before eMMC erase-boundary stalls accumulate
constexpr size_t kLargeImageRecoveryDistance = 16 * 1024 * 1024;
constexpr size_t kLargeImageProbeDistance = 256 * 1024 * 1024;
constexpr int kDefaultEndDataTimeoutMs = 30000;
constexpr int kLargeImageEndDataTimeoutMs = 180000;

struct PartitionEntry {
    std::string id;
    std::string id2;
    uint32_t size = 0;
    uint8_t type = 0;
};

uint16_t nv_crc16(uint16_t crc, const uint8_t* buf, uint32_t len) {
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

std::vector<PartitionEntry> parse_partition_entries(const std::string& xml_str) {
    pugi::xml_document doc;
    std::vector<PartitionEntry> entries;
    if (!doc.load_buffer(xml_str.data(), xml_str.size())) {
        return entries;
    }

    for (auto part : doc.select_nodes("//Partitions/Partition")) {
        auto node = part.node();
        PartitionEntry entry;
        entry.id = node.attribute("id").as_string();
        entry.id2 = node.attribute("id2").as_string();
        std::string size_str = node.attribute("size").as_string();
        try {
            if (size_str.find("0x") == 0 || size_str.find("0X") == 0) {
                entry.size = static_cast<uint32_t>(std::stoul(size_str, nullptr, 16));
            } else {
                entry.size = static_cast<uint32_t>(std::stoul(size_str, nullptr, 10));
            }
        } catch (...) {
            entry.size = 0;
        }

        std::string type_str = node.attribute("type").as_string();
        if (!type_str.empty() && std::isdigit(static_cast<unsigned char>(type_str[0]))) {
            entry.type = static_cast<uint8_t>(type_str[0] - '0');
        }
        if (!entry.id.empty()) {
            entries.push_back(std::move(entry));
        }
    }

    return entries;
}

std::vector<uint8_t> build_repartition_table(const std::vector<PartitionEntry>& entries) {
    std::vector<uint8_t> out;
    if (entries.empty()) {
        return out;
    }

    const bool use_ext_table = std::any_of(entries.begin(), entries.end(), [](const PartitionEntry& entry) {
        return !entry.id2.empty() || entry.type != 0;
    });

    if (use_ext_table) {
        for (const auto& entry : entries) {
            std::vector<uint8_t> part(152, 0);
            size_t chars1 = std::min(entry.id.length(), static_cast<size_t>(35));
            for (size_t i = 0; i < chars1; ++i) {
                part[i * 2] = static_cast<uint8_t>(entry.id[i]);
                part[i * 2 + 1] = 0x00;
            }

            std::string id2 = entry.id2.empty() ? entry.id : entry.id2;
            size_t chars2 = std::min(id2.length(), static_cast<size_t>(35));
            for (size_t i = 0; i < chars2; ++i) {
                part[72 + i * 2] = static_cast<uint8_t>(id2[i]);
                part[72 + i * 2 + 1] = 0x00;
            }

            part[144] = (entry.size >> 0) & 0xFF;
            part[145] = (entry.size >> 8) & 0xFF;
            part[146] = (entry.size >> 16) & 0xFF;
            part[147] = (entry.size >> 24) & 0xFF;
            part[148] = entry.type;
            out.insert(out.end(), part.begin(), part.end());
        }
        return out;
    }

    for (const auto& entry : entries) {
        std::vector<uint8_t> part(76, 0);
        size_t chars = std::min(entry.id.length(), static_cast<size_t>(35));
        for (size_t i = 0; i < chars; ++i) {
            part[i * 2] = static_cast<uint8_t>(entry.id[i]);
            part[i * 2 + 1] = 0x00;
        }
        part[72] = (entry.size >> 0) & 0xFF;
        part[73] = (entry.size >> 8) & 0xFF;
        part[74] = (entry.size >> 16) & 0xFF;
        part[75] = (entry.size >> 24) & 0xFF;
        out.insert(out.end(), part.begin(), part.end());
    }

    return out;
}

std::unordered_map<std::string, uint32_t> build_partition_size_map(const std::vector<PartitionEntry>& entries) {
    std::unordered_map<std::string, uint32_t> out;
    for (const auto& entry : entries) {
        out[entry.id] = entry.size;
    }
    return out;
}

size_t choose_download_chunk_size(const upac::XmlFileConfig& fc, uint64_t file_size) {
    const bool is_64bit = file_size >= 0x100000000ULL;
    if (fc.type == "BOOT_LOADER2" || fc.type == "UBOOT_LOADER2") {
        return kBootLoader2ChunkSize;
    }
    if (fc.type == "CODE2" || fc.type == "YAFFS_IMG2" || fc.type == "NV_COMM" || fc.type == "CHECK_NV2") {
        return is_64bit ? kCode264ChunkSize : kCode2ChunkSize;
    }
    return kDefaultDownloadChunkSize;
}

bool is_nv_checksum_file(const upac::XmlFileConfig& fc) {
    return fc.id.rfind("NV", 0) == 0 || fc.id.rfind("_CHECK_NV", 0) == 0;
}

bool should_auto_pad_named_download(const upac::XmlFileConfig& fc) {
    return fc.type == "CODE2" || fc.type == "YAFFS_IMG2" || fc.type == "CHECK_NV2";
}

bool is_large_transfer_profile(uint64_t file_size, bool use_named_download) {
    return use_named_download && file_size >= kLargeImageThreshold;
}

double bytes_to_mib(size_t bytes) {
    return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

std::string hex_preview(const std::vector<uint8_t>& data, size_t max_bytes = 16) {
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

size_t next_large_transfer_chunk(size_t chunk_size) {
    static constexpr size_t kSteps[] = {
        0x2000, 0x4000, 0x8000
    };
    for (size_t step : kSteps) {
        if (step > chunk_size) {
            return step;
        }
    }
    return kSteps[sizeof(kSteps) / sizeof(kSteps[0]) - 1];
}

size_t prev_large_transfer_chunk(size_t chunk_size) {
    static constexpr size_t kSteps[] = {
        0x2000, 0x4000, 0x8000
    };
    size_t prev = kSteps[0];
    for (size_t step : kSteps) {
        if (step >= chunk_size) {
            return prev;
        }
        prev = step;
    }
    return prev;
}

bool matches_partition_selector(const upac::XmlFileConfig& fc, const std::string& selector) {
    return selector == fc.id || selector == fc.id_name;
}

uint32_t compute_download_checksum(const upac::XmlFileConfig& fc, std::vector<uint8_t>& buf) {
    if (is_nv_checksum_file(fc) && buf.size() >= 2) {
        uint16_t crc = nv_crc16(0, buf.data() + 2, static_cast<uint32_t>(buf.size() - 2));
        buf[0] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        buf[1] = static_cast<uint8_t>(crc & 0xFF);
    }

    uint32_t checksum = 0;
    for (uint8_t b : buf) checksum += b;
    return checksum;
}

std::optional<uint32_t> find_stage_load_address(const upac::PacReader& reader, const std::string& file_id) {
    upac::XmlProductConfig config;
    if (upac::parse_xml_config(reader.xml_config(), config)) {
        for (const auto& fc : config.files) {
            if (fc.id == file_id && fc.base_address != 0) {
                return static_cast<uint32_t>(fc.base_address);
            }
        }
    }

    for (const auto& fi : reader.file_infos()) {
        if (fi.id == file_id && fi.addr_count > 0) {
            for (uint32_t i = 0; i < fi.addr_count && i < upac::MAX_BLOCKS; ++i) {
                if (fi.addr[i] != 0) {
                    return fi.addr[i];
                }
            }
        }
    }

    return std::nullopt;
}

std::string find_extracted_stage_path(const fs::path& tmp_dir, const std::string& preferred_name) {
    fs::path direct = tmp_dir / preferred_name;
    if (fs::exists(direct)) {
        return direct.string();
    }

    std::string needle = preferred_name;
    std::transform(needle.begin(), needle.end(), needle.begin(), [](unsigned char c) { return std::tolower(c); });
    for (const auto& entry : fs::directory_iterator(tmp_dir)) {
        std::string name = entry.path().filename().string();
        std::string lowered = name;
        std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return std::tolower(c); });
        if (lowered.find(needle) != std::string::npos) {
            return entry.path().string();
        }
    }

    return {};
}

const std::unordered_set<std::string>& protected_partition_ids() {
    static const std::unordered_set<std::string> ids = {
        "l_fixnv1", "l_fixnv2", "l_runtimenv1", "l_runtimenv2", "l_deltanv", "prodnv", "miscdata", "userdata"
    };
    return ids;
}

const std::unordered_set<std::string>& protected_file_ids() {
    static const std::unordered_set<std::string> ids = {
        "NV_WLTE", "ProdNV", "PhaseCheck", "Modem_WLTE_DELTANV", "UserData"
    };
    return ids;
}

bool is_protected_flash_target(const upac::XmlFileConfig& fc) {
    return protected_file_ids().count(fc.id) > 0 || protected_partition_ids().count(fc.id_name) > 0;
}

bool connect_with_retry(BslProtocol& bsl, int attempts = 2) {
    for (int i = 0; i < attempts; ++i) {
        if (bsl.connect()) {
            return true;
        }
    }
    return false;
}

bool perform_vendor_fdl2_change_baud(BslProtocol& bsl) {
    // ResearchDownload runs ChangeBaud in the FDL2 sequence. On USB bulk this
    // may be a logical DA/channel step rather than a real host serial retune,
    // so we currently send the vendor opcode with the default vendor baud but
    // intentionally do not change host-side USB line coding yet.
    constexpr uint32_t kVendorDefaultBaud = 115200;
    return bsl.change_baud(kVendorDefaultBaud, 2000);
}

int load_fdl2(BslProtocol& bsl, const std::string& fdl2_path, uint32_t fdl2_addr) {
    std::cout << "Starting Stage 2 (FDL2) transfer...\n";
    if (!connect_with_retry(bsl)) {
        std::cerr << "error: FDL2 CONNECT failed before download.\n";
        return 1;
    }
    if (!perform_vendor_fdl2_change_baud(bsl)) {
        std::cout << "  FDL2 ChangeBaud did not ACK, continuing with current link settings...\n";
    }

    FILE* f2 = fopen(fdl2_path.c_str(), "rb");
    if (!f2) { std::cerr << "error: could not open FDL2 binary: " << fdl2_path << "\n"; return 1; }
    fseek(f2, 0, SEEK_END); long fsize2 = ftell(f2); fseek(f2, 0, SEEK_SET);
    std::vector<uint8_t> fdl2_buf(fsize2);
    fread(fdl2_buf.data(), 1, fsize2, f2); fclose(f2);

    uint32_t fdl2_checksum = 0; for (uint8_t b : fdl2_buf) fdl2_checksum += b;
    if (!bsl.start_data(fdl2_addr, fdl2_buf.size(), fdl2_checksum)) {
        std::cerr << "error: FDL2 START_DATA failed.\n"; return 1;
    }

    for (size_t offset = 0; offset < fdl2_buf.size(); offset += 1024) {
        size_t size = std::min(static_cast<size_t>(1024), fdl2_buf.size() - offset);
        if (!bsl.midst_data(fdl2_buf.data() + offset, size)) {
            std::cerr << "error: FDL2 MIDST_DATA failed at " << offset << "\n"; return 1;
        }
    }
    bsl.end_data();
    bsl.exec_data(fdl2_addr);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return 0;
}

bool init_flash_and_repartition(BslProtocol& bsl, const upac::PacReader& reader, bool skip_nv_backup, FlashMode flash_mode, bool debug_protocol, bool request_fast_mode) {
    std::cout << "Executing FDL2 (ExecNandInit)...\n";
    bool need_repartition = false;
    bool fast_mode_active = false;
    uint16_t type;
    std::vector<uint8_t> out;
    bsl.send_command(BslCommand::BSL_CMD_EXEC_DATA);
    bool got_reply = bsl.receive_packet(type, out, 5000);
    if (debug_protocol) {
        std::cout << "  ExecNandInit reply type=0x" << std::hex << type << std::dec
                  << " payload_len=" << out.size();
        if (!out.empty()) {
            std::cout << " payload_hex=" << hex_preview(out);
        }
        std::cout << "\n";
    }
    if (!got_reply) {
        std::cerr << "  warning: ExecNandInit had no reply, continuing...\n";
    } else if (type == 0x96) {
        std::cout << "  ExecNandInit: partition layout mismatch (0x96), will repartition eMMC.\n";
        need_repartition = true;
        if (request_fast_mode && bsl.transcode_supported()) {
            std::cout << "  Attempting vendor fast mode (DisableTransCode)...\n";
            if (bsl.disable_transcode(2000)) {
                std::cout << "  DisableTransCode enabled.\n";
                fast_mode_active = true;
            } else {
                std::cout << "  DisableTransCode not supported, continuing with normal transcoded packets.\n";
                bsl.drain_recv();
                bsl.clear_halts();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (!bsl.connect()) {
                    std::cout << "  Re-sync after DisableTransCode probe did not ACK, continuing anyway...\n";
                }
            }
        }
    } else if (type == 0x80) {
        std::cout << "  ExecNandInit OK\n";
    } else {
        std::cerr << "  warning: ExecNandInit returned 0x" << std::hex << type << std::dec << ", continuing...\n";
    }

    if (!need_repartition) {
        return fast_mode_active;
    }
    if (flash_mode == FlashMode::PreserveLayout) {
        std::cout << "  Preserve-layout mode: skipping repartition and keeping current device layout.\n";
        return fast_mode_active;
    }

    std::cout << "  Repartitioning eMMC (BSL_CMD_REPARTITION)...\n";
    auto xml_str = reader.xml_config();
    auto parts = parse_partition_entries(xml_str);
    auto part_table = build_repartition_table(parts);
    auto part_sizes = build_partition_size_map(parts);

    if (part_table.empty()) {
        std::cerr << "  warning: no partitions found in XML, skipping repartition\n";
        return fast_mode_active;
    }

    if (skip_nv_backup) {
        std::cout << "  [!] Skipping NV backups forcefully as requested by --skip-nv-backup.\n";
    } else {
        std::cout << "  Backing up critical NV partitions before repartition...\n";
        std::vector<std::string> nv_parts = {"l_fixnv1", "l_fixnv2", "l_runtimenv1", "l_runtimenv2", "prodnv", "miscdata"};
        std::string bdir_name = "nv_backup_" + std::to_string(static_cast<unsigned long>(time(nullptr)));
        fs::path backup_dir = bdir_name;
        fs::create_directories(backup_dir);
        bool backup_session_dirty = false;

        for (const auto& name : nv_parts) {
            auto it = part_sizes.find(name);
            if (it == part_sizes.end() || it->second == 0 || it->second == 0xFFFFFFFFu) {
                continue;
            }

            uint32_t size_bytes = it->second * 1024 * 1024;
            std::cout << "    Reading " << name << "...\n";
            std::vector<uint8_t> dump_data;
            if (bsl.read_partition(name, size_bytes, dump_data, false)) {
                fs::path out_file = backup_dir / (name + ".bin");
                FILE* f = fopen(out_file.c_str(), "wb");
                if (f) {
                    fwrite(dump_data.data(), 1, dump_data.size(), f);
                    fclose(f);
                }
                std::cout << "      Saved to " << out_file.string() << "\n";
            } else {
                std::cerr << "      warning: failed to back up " << name << ", ignoring.\n";
                backup_session_dirty = true;
                bsl.drain_recv();
                bsl.clear_halts();
                break;
            }
        }

        if (backup_session_dirty) {
            std::cout << "  Re-syncing FDL2 before repartition...\n";
            bsl.drain_recv();
            bsl.clear_halts();
            if (!bsl.connect()) {
                std::cout << "  CONNECT did not ACK after backup failure, continuing with repartition anyway...\n";
            }
        }
    }

    size_t entry_size = 76;
    if (!parts.empty() && part_table.size() == parts.size() * 152) {
        entry_size = 152;
    }
    std::cout << "  Sending partition table (" << (part_table.size() / entry_size) << " entries, " << entry_size << " bytes each)...\n";
    if (bsl.repartition(part_table)) {
        std::cout << "  Repartition OK! All partition names now accessible.\n";
    } else {
        std::cerr << "  warning: Repartition failed, flashing may be incomplete.\n";
    }
    return fast_mode_active;
}

void run_dump_phase(BslProtocol& bsl, const upac::PacReader& reader, const std::string& out_dir) {
    std::cout << "\nStarting Firmware Dump Phase to: " << out_dir << "\n";
    fs::create_directories(out_dir);

    pugi::xml_document doc;
    auto xml_str = reader.xml_config();
    doc.load_buffer(xml_str.data(), xml_str.size());

    for (auto part : doc.select_nodes("//Partitions/Partition")) {
        auto node = part.node();
        std::string id = node.attribute("id").as_string();
        std::string size_str = node.attribute("size").as_string();

        uint32_t size_mb = 0;
        try {
            if (size_str.find("0x") == 0 || size_str.find("0X") == 0) size_mb = static_cast<uint32_t>(std::stoul(size_str, nullptr, 16));
            else size_mb = static_cast<uint32_t>(std::stoul(size_str));
        } catch (...) {
            continue;
        }

        if (size_mb == 0 || size_mb > 1024) {
            std::cout << "[" << id << "] Skipping (invalid/unbounded size: " << size_mb << " MB)\n";
            continue;
        }

        uint32_t size_bytes = size_mb * 1024 * 1024;
        std::cout << "[" << id << "] Dumping " << size_mb << " MB...\n";
        std::vector<uint8_t> dump_data;
        if (!bsl.read_partition(id, size_bytes, dump_data)) {
            std::cerr << "    error: failed to read partition " << id << "\n";
            bsl.drain_recv();
            bsl.clear_halts();
            continue;
        }

        fs::path out_path = fs::path(out_dir) / (id + ".bin");
        FILE* f = fopen(out_path.c_str(), "wb");
        if (f) {
            fwrite(dump_data.data(), 1, dump_data.size(), f);
            fclose(f);
            std::cout << "    Saved to " << out_path.filename().string() << "\n";
        } else {
            std::cerr << "    error: failed to open output file " << out_path << "\n";
        }
    }
    std::cout << "\nFirmware dump complete!\n";
}

void run_flash_phase(BslProtocol& bsl, const upac::PacReader& reader, const fs::path& tmp_dir, const std::vector<std::string>& skip_list, const std::vector<std::string>& only_list, FlashMode flash_mode, bool fast_mode_active, bool debug_fast_lane_enabled) {
    std::cout << "\nStarting Partition Flashing Phase...\n";
    upac::XmlProductConfig config;
    if (!upac::parse_xml_config(reader.xml_config(), config)) {
        std::cerr << "error: could not parse PAC XML config\n";
        return;
    }

    auto file_infos = reader.file_infos();

    for (const auto& fc : config.files) {
        if (fc.id == "FDL" || fc.id == "FDL2" || fc.type == "FDL") continue;
        if (!only_list.empty()) {
            bool selected = false;
            for (const auto& part : only_list) {
                if (matches_partition_selector(fc, part)) {
                    selected = true;
                    break;
                }
            }
            if (!selected) continue;
        }

        bool skip_part = false;
        for (const auto& s : skip_list) {
            if (matches_partition_selector(fc, s)) {
                skip_part = true;
                break;
            }
        }
        if (skip_part) {
            std::cout << "\n[" << fc.id << "] " << (fc.id_name.empty() ? "" : "(" + fc.id_name + ") ") << "Skipping per user request...\n";
            continue;
        }
        if (flash_mode == FlashMode::PreserveLayout && is_protected_flash_target(fc)) {
            std::cout << "\n[" << fc.id << "] " << (fc.id_name.empty() ? "" : "(" + fc.id_name + ") ") << "Skipping protected partition in preserve-layout mode...\n";
            continue;
        }

        std::cout << "\n[" << fc.id << "] " << (fc.id_name.empty() ? "" : "(" + fc.id_name + ") ") << "Base: 0x" << std::hex << fc.base_address << std::dec << "\n";

        auto execute_download = [&]() -> bool {
            fs::path p;
            for (const auto& fi : file_infos) {
                std::string pac_id = fi.id;
                pac_id.erase(std::find(pac_id.begin(), pac_id.end(), '\0'), pac_id.end());
                if (pac_id != fc.id) continue;

                std::string fname = fi.name;
                fname.erase(std::find(fname.begin(), fname.end(), '\0'), fname.end());
                if (!fname.empty()) {
                    std::string basename = fname;
                    auto slash_pos = basename.find_last_of("/\\");
                    if (slash_pos != std::string::npos) basename = basename.substr(slash_pos + 1);
                    fs::path potential_path = tmp_dir / basename;
                    if (fs::exists(potential_path)) p = potential_path;
                }
                break;
            }

            if (p.empty() || !fs::exists(p)) {
                if (fc.type.find("Erase") != std::string::npos) {
                    std::cout << "    Erasing " << fc.id << "...\n";
                    return true;
                }
                std::cerr << "    Warning: could not find data for partition " << fc.id << ", skipping...\n";
                return true;
            }

            std::cout << "    Streaming " << p.filename() << " (" << fs::file_size(p) << " bytes)...\n";
            FILE* f = fopen(p.string().c_str(), "rb");
            if (!f) return false;
            fseek(f, 0, SEEK_END); long fsize = ftell(f); fseek(f, 0, SEEK_SET);
            std::vector<uint8_t> buf(fsize);
            fread(buf.data(), 1, fsize, f);
            fclose(f);

            const bool use_named_download = !fc.id_name.empty() && (fc.type.find("2") != std::string::npos || fc.base_address == 0);
            const bool large_transfer_profile = is_large_transfer_profile(buf.size(), use_named_download);
            const bool use_fast_lane = fast_mode_active && large_transfer_profile;
            const bool debug_fast_lane = debug_fast_lane_enabled && fc.id == "Super";
            const size_t base_chunk_size = large_transfer_profile
                ? kLargeImageNamedChunkSize
                : choose_download_chunk_size(fc, buf.size());
            size_t chunk_size = large_transfer_profile ? kLargeImageStartChunkSize : base_chunk_size;
            size_t max_recovery_chunk_size = base_chunk_size;
            int chunk_delay_ms = 0;
            int stable_chunk_count = 0;
            int recovery_cooldown = 0;
            bool has_backed_off = false;
            size_t last_backoff_offset = 0;
            bool freeze_recovery = false;
            bool large_probe_attempted = false;
            bool fast_lane_fell_back = false;
            size_t next_fast_lane_window = use_fast_lane ? kFastLaneWindowSize : 0;
            std::string last_progress_line;
            auto transfer_start = std::chrono::steady_clock::now();
            size_t fast_lane_packet_logs = 0;
            bsl.drain_recv();

            if (use_named_download && should_auto_pad_named_download(fc)) {
                size_t rem4 = buf.size() & 0x3;
                if (rem4 != 0 && (4 - rem4) < kMinNamedTailChunkSize) {
                    size_t pad = 4 - rem4;
                    buf.insert(buf.end(), pad, 0x00);
                    std::cout << "    auto-padding tail by " << pad << " byte(s) for aligned named download\n";
                }
            }

            uint32_t checksum = compute_download_checksum(fc, buf);
            if (use_named_download) {
                if (large_transfer_profile) {
                    chunk_size = std::min(chunk_size, kLargeImageStartChunkSize);
                    chunk_delay_ms = 0;
                } else {
                    chunk_size = std::min(chunk_size, static_cast<size_t>(0x1000));
                    chunk_delay_ms = buf.size() <= 0x100000 ? 2 : 1;
                }
            }

            bool start_ok = use_named_download
                ? bsl.start_data(fc.id_name, buf.size(), checksum, buf.size() >= 0x100000000ULL)
                : bsl.start_data(static_cast<uint32_t>(fc.base_address), buf.size(), checksum);
            if (!start_ok) {
                std::cerr << "    Skipping " << fc.id << " (not supported in current device state)\n";
                return true;
            }

            // Enable the pipeline for all large transfers, not just when
            // DisableTransCode is active.  build_midst_data_packet() applies
            // HDLC byte-stuffing when needed, so pre-built frames are always
            // correct.  The pipeline lets the encoder run concurrently with
            // the previous ACK round-trip, and the pipeline's write_fast()
            // path sends the whole pre-built frame in a single bulk transfer
            // rather than 4 KB slices, eliminating the 1 ms/slice sleep.
            bool use_packet_pipeline = large_transfer_profile;
            size_t pipeline_depth = use_fast_lane ? 6 : 4;
            PacketPipeline pipeline(bsl, buf, chunk_size, use_packet_pipeline, pipeline_depth);
            pipeline.start(0);

            for (size_t offset = 0; offset < buf.size();) {
                if (use_named_download) {
                    size_t remaining = buf.size() - offset;
                    if (remaining <= 0x80) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x20));
                        chunk_delay_ms = std::max(chunk_delay_ms, 8);
                    } else if (remaining <= 0x200) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x40));
                        chunk_delay_ms = std::max(chunk_delay_ms, 7);
                    } else if (remaining <= 0x800) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x80));
                        chunk_delay_ms = std::max(chunk_delay_ms, 6);
                    } else if (remaining <= 0x1000) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x100));
                        chunk_delay_ms = std::max(chunk_delay_ms, 5);
                    } else if (remaining <= 0x4000) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x200));
                        chunk_delay_ms = std::max(chunk_delay_ms, 4);
                    } else if (remaining <= 0x8000) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x400));
                        chunk_delay_ms = std::max(chunk_delay_ms, 3);
                    } else if (remaining <= 0x20000) {
                        chunk_size = std::min(chunk_size, static_cast<size_t>(0x800));
                        chunk_delay_ms = std::max(chunk_delay_ms, 2);
                    }
                }

                size_t remaining = buf.size() - offset;
                size_t to_send = std::min(chunk_size, remaining);
                if (use_named_download &&
                    remaining > kMinNamedTailChunkSize &&
                    remaining - to_send > 0 &&
                    remaining - to_send < kMinNamedTailChunkSize &&
                    remaining <= chunk_size + kMinNamedTailChunkSize) {
                    to_send = remaining;
                }

                bool sent_ok = false;
                size_t sent_size = to_send;
                bool pipeline_popped = false;
                if (use_packet_pipeline && to_send == chunk_size) {
                    FramedPacket packet;
                    if (pipeline.pop(packet)) {
                        pipeline_popped = true;
                        sent_size = packet.size;
                        if (debug_fast_lane && fast_lane_packet_logs < 6) {
                            std::cout << "    " << (use_fast_lane ? "fast lane" : "pipeline")
                                      << " debug: offset=" << packet.offset
                                      << " payload=" << packet.size
                                      << " frame=" << packet.frame.size()
                                      << (use_fast_lane ? " [no-transcode]" : " [hdlc]")
                                      << " ack=60000ms\n";
                            ++fast_lane_packet_logs;
                        }
                        sent_ok = bsl.send_framed_packet_fast(packet.frame, 30000, 60000);
                    }
                }
                if (!pipeline_popped) {
                    sent_size = to_send;
                    sent_ok = bsl.midst_data(buf.data() + offset, static_cast<uint32_t>(to_send));
                }

                if (!sent_ok) {
                    stable_chunk_count = 0;
                    recovery_cooldown = large_transfer_profile ? 256 : 0;
                    has_backed_off = true;
                    last_backoff_offset = offset;
                    if (large_transfer_profile && remaining > 0x20000) {
                        freeze_recovery = true;
                    }
                    bsl.drain_recv();
                    bsl.clear_halts();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    if (use_packet_pipeline) {
                        use_packet_pipeline = false;
                        fast_lane_fell_back = true;
                        chunk_size = std::min(chunk_size, kLargeImageStartChunkSize);
                        max_recovery_chunk_size = std::min(max_recovery_chunk_size, chunk_size);
                        chunk_delay_ms = std::max(chunk_delay_ms, 1);
                        freeze_recovery = true;
                        std::cout << "\n    " << (use_fast_lane ? "fast lane" : "pipeline")
                                  << " fallback: switching to non-pipelined transfer at chunk="
                                  << chunk_size << " delay=" << chunk_delay_ms << "ms\n";
                        if (debug_fast_lane) {
                            std::cout << "    " << (use_fast_lane ? "fast lane" : "pipeline")
                                      << " debug: fallback at offset=" << offset
                                      << " remaining=" << remaining
                                      << " last_attempt=" << sent_size
                                      << " frame_mode=" << (use_fast_lane ? "no-transcode" : "hdlc") << "\n";
                        }
                    }
                    pipeline.reset(offset, chunk_size);
                    size_t retry_send = std::min(chunk_size, remaining);
                    if (use_named_download &&
                        remaining > kMinNamedTailChunkSize &&
                        remaining - retry_send > 0 &&
                        remaining - retry_send < kMinNamedTailChunkSize &&
                        remaining <= chunk_size + kMinNamedTailChunkSize) {
                        retry_send = remaining;
                    }
                    if (bsl.midst_data(buf.data() + offset, static_cast<uint32_t>(retry_send))) {
                        offset += retry_send;
                        continue;
                    }

                    size_t min_chunk_size = kMinAdaptiveChunkSize;
                    if (use_named_download) {
                        bool in_tail_taper = remaining <= 0x20000;
                        min_chunk_size = in_tail_taper
                            ? kMinNamedTailChunkSize
                            : (large_transfer_profile ? kLargeImageMinChunkSize : kMinAdaptiveChunkSize);
                    }
                    if (chunk_size > min_chunk_size) {
                        if (large_transfer_profile && remaining > 0x20000) {
                            chunk_size = std::max(min_chunk_size, prev_large_transfer_chunk(chunk_size));
                            max_recovery_chunk_size = std::min(max_recovery_chunk_size, chunk_size);
                            pipeline.reset(offset, chunk_size);
                        } else {
                            chunk_size = std::max(min_chunk_size, chunk_size / 2);
                        }
                        chunk_delay_ms = std::min(10, chunk_delay_ms + 2);
                        std::cout << "\n    adapting transfer: chunk=" << chunk_size << " delay=" << chunk_delay_ms << "ms\n";
                        continue;
                    }

                    std::cerr << "\n    warning: MIDST_DATA failed at offset " << offset << ", skipping partition\n";
                    bsl.drain_recv();
                    bsl.clear_halts();
                    return true;
                }

                if (chunk_delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(chunk_delay_ms));
                offset += sent_size;
                if (use_packet_pipeline && next_fast_lane_window != 0 && offset >= next_fast_lane_window) {
                    pipeline.stop();
                    std::cout << "\n    fast lane window complete at offset=" << offset
                              << ", syncing FDL2...\n";
                    bsl.drain_recv();
                    bsl.clear_halts();
                    // CONNECT resets the FDL2 watchdog timer mid-transfer, matching
                    // ResearchDownload's behaviour every ~128 MB.  ACK failure is
                    // non-fatal; the download state on the device is not reset.
                    if (!bsl.connect()) {
                        std::cout << "    fast lane sync: CONNECT did not ACK, continuing...\n";
                    }
                    pipeline.reset(offset, chunk_size);
                    next_fast_lane_window += kFastLaneWindowSize;
                }
                ++stable_chunk_count;
                if (recovery_cooldown > 0) {
                    --recovery_cooldown;
                }

                int recovery_window = large_transfer_profile ? 16 : 64;
                bool passed_recovery_distance =
                    !large_transfer_profile || !has_backed_off || (offset >= last_backoff_offset + kLargeImageRecoveryDistance);
                if (stable_chunk_count >= recovery_window &&
                    recovery_cooldown == 0 &&
                    passed_recovery_distance &&
                    !freeze_recovery) {
                    size_t remaining_after = buf.size() - offset;
                    bool in_tail_taper = use_named_download && remaining_after <= 0x20000;
                    if (!in_tail_taper && chunk_size < base_chunk_size) {
                        size_t next_chunk_size = chunk_size;
                        if (large_transfer_profile) {
                            const bool can_probe_faster =
                                !has_backed_off &&
                                !large_probe_attempted &&
                                !fast_lane_fell_back &&
                                !use_fast_lane &&
                                offset >= kLargeImageProbeDistance;
                            if (can_probe_faster) {
                                next_chunk_size = std::min(base_chunk_size, next_large_transfer_chunk(chunk_size));
                                large_probe_attempted = true;
                            }
                        } else {
                            next_chunk_size = std::min(base_chunk_size, chunk_size * 2);
                        }
                        next_chunk_size = std::min(next_chunk_size, max_recovery_chunk_size);
                        int next_delay_ms = std::max(0, chunk_delay_ms - 1);
                        if (next_chunk_size != chunk_size || next_delay_ms != chunk_delay_ms) {
                            chunk_size = next_chunk_size;
                            chunk_delay_ms = next_delay_ms;
                            std::cout << "\n    "
                                      << (has_backed_off ? "recovering transfer" : "ramping up speed")
                                      << ": chunk=" << chunk_size
                                      << " delay=" << chunk_delay_ms << "ms\n";
                        }
                    }
                    stable_chunk_count = 0;
                }

                if (offset % (chunk_size * 10) == 0 || offset >= buf.size()) {
                    auto elapsed = std::chrono::steady_clock::now() - transfer_start;
                    double elapsed_sec = std::max(0.001, std::chrono::duration<double>(elapsed).count());
                    double speed_mib_s = bytes_to_mib(offset) / elapsed_sec;
                    std::ostringstream progress;
                    progress << "    Progress: " << (offset * 100 / buf.size()) << "%"
                             << " (" << std::fixed << std::setprecision(1) << bytes_to_mib(offset)
                             << "/" << bytes_to_mib(buf.size()) << " MiB"
                             << ", " << speed_mib_s << " MiB/s"
                             << ", chunk=" << chunk_size << ")";
                    std::string progress_line = progress.str();
                    size_t pad = last_progress_line.size() > progress_line.size()
                        ? last_progress_line.size() - progress_line.size()
                        : 0;
                    std::cout << "\r" << progress_line << std::string(pad, ' ') << std::flush;
                    last_progress_line = progress_line;
                }
            }
            pipeline.stop();

            int end_data_timeout_ms = large_transfer_profile ? kLargeImageEndDataTimeoutMs : kDefaultEndDataTimeoutMs;
            std::cout << "\n    Data transfer complete, waiting for END_DATA verify...\n";
            if (!bsl.end_data(end_data_timeout_ms)) {
                std::cerr << "\n    warning: END_DATA verification failed, continuing to next partition\n";
                bsl.drain_recv();
                bsl.clear_halts();
                return true;
            }
            size_t ok_len = std::char_traits<char>::length("    Progress: 100% - OK");
            size_t ok_pad = last_progress_line.size() > ok_len ? last_progress_line.size() - ok_len : 0;
            std::cout << "\r    Progress: 100% - OK" << std::string(ok_pad, ' ') << "\n";
            return true;
        };

        if (fc.operations.empty()) {
            if (!execute_download()) break;
            continue;
        }

        bool ok = true;
        for (const auto& op : fc.operations) {
            if (op.name == "CheckBaud") bsl.handshake(500);
            else if (op.name == "Connect") bsl.connect();
            else if (op.name == "Download" && !execute_download()) {
                ok = false;
                break;
            }
        }
        if (!ok) break;
    }
}

bool maybe_enable_fast_download_mode(BslProtocol& bsl, const CliOptions& options) {
    if (!options.disable_transcode) {
        return false;
    }
    if (options.debug_fast_lane) {
        const auto& payload = bsl.last_connect_payload();
        std::cout << "Attempting vendor fast mode (DisableTransCode)...\n";
        std::cout << "  CONNECT ACK type=0x" << std::hex << bsl.last_connect_type() << std::dec
                  << " payload_len=" << payload.size();
        if (!payload.empty()) {
            std::cout << " payload_hex=" << hex_preview(payload);
        }
        std::cout << "\n";
    }
    if (!bsl.transcode_supported()) {
        if (!options.debug_fast_lane) {
            std::cout << "Attempting vendor fast mode (DisableTransCode)...\n";
        }
        std::cout << "  DisableTransCode not advertised by current FDL session/ExecNandInit reply, staying on normal transcoded packets.\n";
        return false;
    }
    if (!options.debug_fast_lane) {
        std::cout << "Attempting vendor fast mode (DisableTransCode)...\n";
    }
    if (bsl.disable_transcode(2000)) {
        std::cout << "  DisableTransCode enabled.\n";
        return true;
    } else {
        std::cout << "  DisableTransCode not supported, continuing with normal transcoded packets.\n";
        bsl.drain_recv();
        bsl.clear_halts();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!bsl.connect()) {
            std::cout << "  Re-sync after DisableTransCode probe did not ACK, continuing anyway...\n";
        }
        return false;
    }
}

int run_with_connected_device(std::unique_ptr<UsbDevice>& dev, BslProtocol& bsl, const upac::PacReader& reader, const CliOptions& options, const fs::path& tmp_dir, bool fdl_running) {
    dev->set_debug_io(options.debug_protocol);
    bsl.set_debug_protocol(options.debug_protocol);
    if (fdl_running) {
        std::cout << "Detected FDL1/FDL2 already running. Proceeding to flash phase...\n";
        bsl.set_use_checksum(true);
        if (!bsl.connect()) {
            std::cout << "  CONNECT failed, trying modern 0xAE fallback...\n";
            if (!bsl.host_handshake(2000)) {
                std::cout << "  Warning: could not confirm connection, continuing anyway...\n";
            }
        }

        bool fast_mode_active = init_flash_and_repartition(bsl, reader, options.skip_nv_backup, options.flash_mode, options.debug_protocol, options.disable_transcode);
        if (!fast_mode_active) {
            fast_mode_active = maybe_enable_fast_download_mode(bsl, options);
        }
        if (!options.dump_out_dir.empty()) run_dump_phase(bsl, reader, options.dump_out_dir);
        else run_flash_phase(bsl, reader, tmp_dir, options.skip_list, options.only_list, options.flash_mode, fast_mode_active, options.debug_fast_lane);
        return 0;
    }

    std::cout << "FDL not detected. Loading Stage 1 (FDL1)...\n";
    bsl.send_command(BSL_CMD_CONNECT);
    uint16_t type;
    std::vector<uint8_t> payload;
    bsl.receive_packet(type, payload);

    std::string fdl1_path = find_extracted_stage_path(tmp_dir, "fdl1");
    FILE* f1 = fopen(fdl1_path.c_str(), "rb");
    if (!f1) { std::cerr << "error: could not open FDL1 binary\n"; return 1; }
    fseek(f1, 0, SEEK_END); long fsize1 = ftell(f1); fseek(f1, 0, SEEK_SET);
    std::vector<uint8_t> fdl1_buf(fsize1);
    fread(fdl1_buf.data(), 1, fsize1, f1);
    fclose(f1);

    uint32_t fdl1_checksum = 0;
    for (uint8_t b : fdl1_buf) fdl1_checksum += b;
    auto fdl1_addr_opt = find_stage_load_address(reader, "FDL");
    if (!fdl1_addr_opt) {
        std::cerr << "error: could not determine FDL1 load address from PAC metadata\n";
        return 1;
    }

    uint32_t fdl1_addr = *fdl1_addr_opt;
    if (!bsl.start_data(fdl1_addr, fdl1_buf.size(), fdl1_checksum)) {
        std::cerr << "error: FDL1 START_DATA failed\n";
        return 1;
    }
    for (size_t offset = 0; offset < fdl1_buf.size(); offset += 512) {
        size_t to_send = std::min(static_cast<size_t>(512), fdl1_buf.size() - offset);
        if (!bsl.midst_data(fdl1_buf.data() + offset, to_send)) {
            std::cerr << "error: FDL1 MIDST_DATA failed at offset " << offset << "\n";
            return 1;
        }
    }
    bsl.end_data();
    bsl.exec_data(fdl1_addr);

    std::cout << "FDL1 is running. Waiting for re-enumeration...\n";
    dev.reset();

    std::unique_ptr<UsbDevice> dev2;
    for (int i = 0; i < 80; ++i) {
        auto opt2 = UsbDevice::find_any();
        if (opt2) {
            dev2 = std::move(*opt2);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!dev2) {
        std::cerr << "error: device did not re-appear.\n";
        return 1;
    }

    if (options.fdl2_settle_ms > 0) {
        std::cout << "Allowing FDL1->FDL2 settle time (" << options.fdl2_settle_ms << " ms)...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(options.fdl2_settle_ms));
    }

    dev2->clear_halt();

    BslProtocol bsl2(*dev2);
    dev2->set_debug_io(options.debug_protocol);
    bsl2.set_debug_protocol(options.debug_protocol);
    std::cout << "Attempting Stage 2 Handshake...\n";
    if (!bsl2.handshake(5000)) {
        std::cout << "  Sync silent, trying CONNECT directly...\n";
        if (!bsl2.connect()) {
            std::cerr << "  error: Stage 2 Handshake failed.\n";
            return 1;
        }
    }

    std::string fdl2_path = find_extracted_stage_path(tmp_dir, "fdl2");
    auto fdl2_addr_opt = find_stage_load_address(reader, "FDL2");
    if (!fdl2_addr_opt) {
        std::cerr << "error: could not determine FDL2 load address from PAC metadata\n";
        return 1;
    }

    bsl2.set_use_checksum(true);
    if (load_fdl2(bsl2, fdl2_path, *fdl2_addr_opt) != 0) {
        return 1;
    }

    bool fast_mode_active = init_flash_and_repartition(bsl2, reader, options.skip_nv_backup, options.flash_mode, options.debug_protocol, options.disable_transcode);
    if (!fast_mode_active) {
        fast_mode_active = maybe_enable_fast_download_mode(bsl2, options);
    }
    if (!options.dump_out_dir.empty()) run_dump_phase(bsl2, reader, options.dump_out_dir);
    else run_flash_phase(bsl2, reader, tmp_dir, options.skip_list, options.only_list, options.flash_mode, fast_mode_active, options.debug_fast_lane);
    return 0;
}

} // namespace

int run_flash_tool(const CliOptions& options) {
    if (options.dump_descriptors) {
        UsbDevice::dump_descriptors();
        return 0;
    }

    if (options.dump_xml && !options.pac_path.empty()) {
        auto r_opt = upac::PacReader::open(options.pac_path);
        if (r_opt) std::cout << r_opt->xml_config() << std::endl;
        return 0;
    }

    if (options.reset_only) {
        auto d_opt = UsbDevice::find_any();
        if (!d_opt) {
            std::cerr << "error: no device found for reset\n";
            return 1;
        }
        BslProtocol b_reset(**d_opt);
        std::cout << "Attempting software reset (0x0B)...\n";
        b_reset.handshake(2000);
        b_reset.send_command(BSL_CMD_NORMAL_RESET);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Reset sent.\n";
        return 0;
    }

    auto reader_opt = upac::PacReader::open(options.pac_path);
    if (!reader_opt) {
        std::cerr << "error: could not open PAC file: " << options.pac_path << "\n";
        return 1;
    }
    const auto& reader = *reader_opt;

    auto dev_opt = UsbDevice::find_any();
    if (!dev_opt) {
        std::cerr << "error: no Unisoc device found in BROM/BSL mode.\n";
        std::cerr << "Hint: Hold Vol-Down and plug in the USB cable.\n";
        return 1;
    }

    auto& dev = *dev_opt;
    BslProtocol bsl(*dev);
    bool handshake_ok = bsl.handshake();
    if (!handshake_ok) {
        std::cout << "Legacy 0x7E handshake failed, trying modern 0xAE Host Protocol...\n";
        handshake_ok = bsl.host_handshake();
    }
    if (!handshake_ok) {
        std::cerr << "error: all handshake methods failed.\n";
        return 1;
    }

    std::string handshake_ver = bsl.get_last_handshake_response();
    std::cout << "Handshake successful! Response: " << handshake_ver << "\n";

    fs::path tmp_dir = "/tmp/uflash_extract";
    fs::create_directories(tmp_dir);
    if (reader.extract_all(tmp_dir)) {
        std::cout << "PAC components extracted to " << tmp_dir << "\n";
    }

    bool fdl_running = handshake_ver.find("Spreadtrum") != std::string::npos;
    return run_with_connected_device(dev, bsl, reader, options, tmp_dir, fdl_running);
}

} // namespace uflash
