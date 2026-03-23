#include "uflash/cli_options.h"
#include "uflash/flash_workflow.h"

#include <iostream>

int main(int argc, char** argv) {
    std::cout << "uflash — Unisoc Flash Tool\n";
    std::cout << "--------------------------\n";

    uflash::ParseResult parsed = uflash::parse_cli_options(argc, argv);
    if (!parsed.ok) {
        std::cerr << parsed.message << "\n";
        return parsed.exit_code;
    }

    return uflash::run_flash_tool(parsed.options);
}
