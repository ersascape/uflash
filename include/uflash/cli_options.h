#pragma once

#include <optional>
#include <string>
#include <vector>

namespace uflash {

enum class FlashMode {
    Full = 0,
    PreserveLayout = 1,
};

struct CliOptions {
    bool dump_descriptors = false;
    bool reset_only = false;
    bool dump_xml = false;
    bool skip_nv_backup = false;
    bool disable_transcode = false;
    bool debug_fast_lane = false;
    bool debug_protocol = false;
    int fdl2_settle_ms = 0;
    FlashMode flash_mode = FlashMode::Full;
    std::string dump_out_dir;
    std::vector<std::string> skip_list;
    std::vector<std::string> only_list;
    std::string pac_path;
};

struct ParseResult {
    bool ok = true;
    bool should_exit = false;
    int exit_code = 0;
    std::string message;
    CliOptions options;
};

ParseResult parse_cli_options(int argc, char** argv);
std::string build_usage();

} // namespace uflash
