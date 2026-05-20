#include "uflash/cli_options.h"
#include "uflash/flash_workflow.h"

#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {
    bool tty = isatty(STDOUT_FILENO);

    // When stdout is a pipe (TUI subprocess mode), the C library switches to
    // fully-buffered mode, holding all output until the buffer fills or the
    // process exits.  Switch to line-buffered so every "\n" line is flushed
    // to the pipe immediately — the TUI needs real-time updates.
    if (!tty) {
        setvbuf(stdout, nullptr, _IOLBF, 0);
    }

    if (tty) {
        std::cout
            << "\x1b[1;36m  ⚡ uflash\x1b[0m"
            << "  \x1b[38;5;244mUnisoc / Spreadtrum flash tool\x1b[0m\n"
            << "\x1b[38;5;238m  ────────────────────────────────────────\x1b[0m\n";
    } else {
        std::cout << "uflash — Unisoc Flash Tool\n"
                  << "──────────────────────────\n";
    }

    uflash::ParseResult parsed = uflash::parse_cli_options(argc, argv);
    if (!parsed.ok) {
        if (tty) std::cerr << "\x1b[31merror:\x1b[0m " << parsed.message << "\n";
        else     std::cerr << "error: " << parsed.message << "\n";
        return parsed.exit_code;
    }
    return uflash::run_flash_tool(parsed.options);
}
