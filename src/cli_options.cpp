#include "uflash/cli_options.h"

#include <algorithm>

namespace uflash {

std::string build_usage() {
    return "Usage: uflash <firmware.pac> [--full-flash | --preserve-layout | --reset-only | --dump-firmware <dir> | --skip-nv-backup | --skip <part> | --partition <part> | --fdl2-settle-ms <ms> | --disable-transcode | --debug-fast-lane | --debug-protocol]";
}

ParseResult parse_cli_options(int argc, char** argv) {
    ParseResult result;

    if (argc > 1 && std::string(argv[1]) == "--dump-descriptors") {
        result.options.dump_descriptors = true;
        result.should_exit = true;
        return result;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--reset-only") {
            result.options.reset_only = true;
        } else if (arg == "--dump-xml") {
            result.options.dump_xml = true;
        } else if (arg == "--skip-nv-backup") {
            result.options.skip_nv_backup = true;
        } else if (arg == "--disable-transcode") {
            result.options.disable_transcode = true;
        } else if (arg == "--debug-fast-lane") {
            result.options.debug_fast_lane = true;
        } else if (arg == "--debug-protocol") {
            result.options.debug_protocol = true;
        } else if (arg == "--preserve-layout") {
            result.options.flash_mode = FlashMode::PreserveLayout;
        } else if (arg == "--full-flash") {
            result.options.flash_mode = FlashMode::Full;
        } else if (arg == "--skip" && i + 1 < argc) {
            result.options.skip_list.push_back(argv[++i]);
        } else if (arg == "--partition" && i + 1 < argc) {
            result.options.only_list.push_back(argv[++i]);
        } else if (arg == "--fdl2-settle-ms" && i + 1 < argc) {
            result.options.fdl2_settle_ms = std::max(0, std::stoi(argv[++i]));
        } else if (arg == "--dump-descriptors") {
            result.options.dump_descriptors = true;
            result.should_exit = true;
            return result;
        } else if (arg == "--dump-firmware" && i + 1 < argc) {
            result.options.dump_out_dir = argv[++i];
        } else if (result.options.pac_path.empty() && arg.find("--") == std::string::npos) {
            result.options.pac_path = arg;
        }
    }

    if (result.options.pac_path.empty() && !result.options.reset_only && !result.options.dump_descriptors) {
        result.ok = false;
        result.exit_code = 1;
        result.message = build_usage();
        return result;
    }

    return result;
}

} // namespace uflash
