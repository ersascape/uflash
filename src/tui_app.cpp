#include "upac/pac_reader.h"
#include "upac/xml_config.h"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct TerminalGuard {
    termios original {};
    bool active = false;

    bool enable_raw() {
        if (!isatty(STDIN_FILENO)) return false;
        if (tcgetattr(STDIN_FILENO, &original) != 0) return false;
        termios raw = original;
        raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        raw.c_iflag &= ~(IXON | ICRNL | BRKINT | INPCK | ISTRIP);
        raw.c_oflag &= ~(OPOST);
        raw.c_cflag |= CS8;
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 1;
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) return false;
        active = true;
        return true;
    }

    void disable_raw() {
        if (active) {
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &original);
            active = false;
        }
    }

    ~TerminalGuard() {
        disable_raw();
    }
};

struct AltScreenGuard {
    AltScreenGuard() {
        std::cout << "\x1b[?1049h\x1b[?25l" << std::flush;
    }
    ~AltScreenGuard() {
        std::cout << "\x1b[?25h\x1b[?1049l" << std::flush;
    }
};

struct PartitionItem {
    std::string id;
    std::string id_name;
    std::string type;
    uint64_t size_bytes = 0;
    bool selected = true;
};

struct ScreenSize {
    int rows = 32;
    int cols = 120;
};

enum class FlashMode {
    Preserve,
    Full,
};

enum class Key {
    None,
    Up,
    Down,
    Left,
    Right,
    Enter,
    Space,
    Quit,
    ToggleAll,
    ToggleModePreserve,
    ToggleModeFull,
    ToggleTranscode,
    ToggleDebugFast,
    Start,
};

struct SessionConfig {
    FlashMode flash_mode = FlashMode::Preserve;
    bool disable_transcode = false;
    bool debug_fast_lane = false;
};

struct FlashProgress {
    std::string current_partition;
    int percent = 0;
    std::string detail;
    std::deque<std::string> logs;
    bool finished = false;
    int exit_code = -1;
};

ScreenSize get_screen_size() {
    winsize ws {};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0) {
        int rows = static_cast<int>(ws.ws_row);
        int cols = static_cast<int>(ws.ws_col);
        rows = std::max(20, std::min(rows, 60));
        cols = std::max(80, std::min(cols, 140));
        return {rows, cols};
    }
    return {};
}

std::string fit_text(const std::string& text, int width) {
    if (width <= 0) return "";
    if (static_cast<int>(text.size()) <= width) return text;
    if (width <= 3) return text.substr(0, width);
    return text.substr(0, width - 3) + "...";
}

std::string pad_right(const std::string& text, int width) {
    if (width <= 0) return "";
    std::string out = fit_text(text, width);
    if (static_cast<int>(out.size()) < width) out += std::string(width - out.size(), ' ');
    return out;
}

std::string human_mib(uint64_t bytes) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(bytes >= 100ULL * 1024ULL * 1024ULL ? 0 : 1)
        << (static_cast<double>(bytes) / (1024.0 * 1024.0)) << " MiB";
    return out.str();
}

Key read_key() {
    char c = 0;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0) return Key::None;
    if (c == '\x1b') {
        char seq[2] = {};
        if (::read(STDIN_FILENO, &seq[0], 1) <= 0) return Key::Quit;
        if (::read(STDIN_FILENO, &seq[1], 1) <= 0) return Key::Quit;
        if (seq[0] == '[') {
            if (seq[1] == 'A') return Key::Up;
            if (seq[1] == 'B') return Key::Down;
            if (seq[1] == 'C') return Key::Right;
            if (seq[1] == 'D') return Key::Left;
        }
        return Key::None;
    }
    if (c == '\r' || c == '\n') return Key::Enter;
    if (c == ' ') return Key::Space;
    if (c == 'q' || c == 'Q') return Key::Quit;
    if (c == 'a' || c == 'A') return Key::ToggleAll;
    if (c == 'p' || c == 'P') return Key::ToggleModePreserve;
    if (c == 'f' || c == 'F') return Key::ToggleModeFull;
    if (c == 't' || c == 'T') return Key::ToggleTranscode;
    if (c == 'g' || c == 'G') return Key::ToggleDebugFast;
    if (c == 's' || c == 'S') return Key::Start;
    if (c == 'j' || c == 'J') return Key::Down;
    if (c == 'k' || c == 'K') return Key::Up;
    return Key::None;
}

std::vector<PartitionItem> load_partitions(const fs::path& pac_path) {
    auto reader_opt = upac::PacReader::open(pac_path);
    if (!reader_opt) {
        throw std::runtime_error("Could not open PAC file");
    }
    const auto& reader = *reader_opt;

    upac::XmlProductConfig config;
    if (!upac::parse_xml_config(reader.xml_config(), config)) {
        throw std::runtime_error("Could not parse PAC XML");
    }

    auto infos = reader.file_infos();
    std::vector<PartitionItem> items;

    for (const auto& fc : config.files) {
        if (fc.id == "FDL" || fc.id == "FDL2" || fc.type == "FDL") continue;
        PartitionItem item;
        item.id = fc.id;
        item.id_name = fc.id_name;
        item.type = fc.type;
        item.selected = fc.type.find("Erase") == std::string::npos;

        for (const auto& fi : infos) {
            std::string pac_id = fi.id;
            pac_id.erase(std::find(pac_id.begin(), pac_id.end(), '\0'), pac_id.end());
            if (pac_id == fc.id) {
                item.size_bytes = fi.size;
                break;
            }
        }
        items.push_back(std::move(item));
    }

    return items;
}

void render_selector(const fs::path& pac_path, const std::vector<PartitionItem>& items, size_t cursor, size_t scroll, const SessionConfig& config) {
    ScreenSize sz = get_screen_size();
    const int header_lines = 8;
    const int footer_lines = 4;
    const int visible_rows = std::max(5, sz.rows - header_lines - footer_lines);
    const int content_width = std::min(110, std::max(60, sz.cols - 2));
    const int size_w = 10;
    const int type_w = content_width >= 110 ? 14 : 10;
    const int block_w = content_width >= 95 ? 16 : 12;
    const int part_w = std::max(18, content_width - size_w - type_w - block_w - 6);

    size_t selected_count = 0;
    for (const auto& item : items) if (item.selected) ++selected_count;

    std::ostringstream out;
    out << "\x1b[2J\x1b[H";
    out << "\x1b[48;5;236m \x1b[1;38;5;81muflash-tui\x1b[0m  "
        << fit_text(pac_path.filename().string(), std::max(20, sz.cols - 24)) << "\n";
    out << " Mode: " << (config.flash_mode == FlashMode::Preserve ? "\x1b[38;5;120mpreserve-layout\x1b[0m" : "\x1b[38;5;214mfull-flash\x1b[0m")
        << "   Fast mode: " << (config.disable_transcode ? "\x1b[38;5;45mon\x1b[0m" : "\x1b[38;5;244moff\x1b[0m")
        << "   Fast debug: " << (config.debug_fast_lane ? "\x1b[38;5;208mon\x1b[0m" : "\x1b[38;5;244moff\x1b[0m")
        << "   Selected: " << selected_count << "/" << items.size() << "\n";
    out << " Move: j/k or arrows   Toggle: space   Start: enter/s   Quit: q\n";
    out << " Actions: a all   p preserve   f full   t fast   g fast-debug\n";
    out << std::string(std::max(0, content_width), '=') << "\n";
    out << pad_right("Partition", part_w)
        << "  " << pad_right("Block", block_w)
        << "  " << pad_right("Type", type_w)
        << "  " << pad_right("Size", size_w) << "\n";
    out << std::string(std::max(0, content_width), '-') << "\n";

    for (int row = 0; row < visible_rows; ++row) {
        size_t idx = scroll + static_cast<size_t>(row);
        if (idx >= items.size()) {
            out << "\n";
            continue;
        }
        const auto& item = items[idx];
        bool active = idx == cursor;
        if (active) out << "\x1b[48;5;238m\x1b[38;5;255m";
        std::ostringstream left;
        left << (item.selected ? "[x] " : "[ ] ");
        if (active) left << "> ";
        else left << "  ";
        left << item.id;
        out << pad_right(left.str(), part_w)
            << "  " << pad_right(item.id_name.empty() ? "-" : item.id_name, block_w)
            << "  " << pad_right(item.type, type_w)
            << "  " << pad_right(item.size_bytes ? human_mib(item.size_bytes) : "-", size_w);
        if (active) out << "\x1b[0m";
        out << "\n";
    }

    out << std::string(std::max(0, content_width), '=') << "\n";
    out << "Press enter to start flashing the selected partitions.\n";
    std::cout << out.str() << std::flush;
}

std::string render_bar(int width, int percent) {
    width = std::max(10, width);
    int filled = (width * std::max(0, std::min(percent, 100))) / 100;
    return std::string(filled, '#') + std::string(width - filled, '-');
}

void push_log(std::deque<std::string>& logs, const std::string& line, size_t max_lines) {
    logs.push_back(line);
    while (logs.size() > max_lines) logs.pop_front();
}

void render_flash(const fs::path& pac_path, const SessionConfig& config, const FlashProgress& progress) {
    ScreenSize sz = get_screen_size();
    int log_lines = std::max(8, sz.rows - 10);
    const int content_width = std::min(110, std::max(60, sz.cols - 2));
    std::ostringstream out;
    out << "\x1b[2J\x1b[H";
    out << "\x1b[1;38;5;81muflash-tui\x1b[0m  running " << fit_text(pac_path.filename().string(), content_width - 18) << "\n";
    out << "Mode: " << (config.flash_mode == FlashMode::Preserve ? "preserve-layout" : "full-flash")
        << "   Fast mode: " << (config.disable_transcode ? "on" : "off")
        << "   Fast debug: " << (config.debug_fast_lane ? "on" : "off") << "\n";
    out << "Current: " << (progress.current_partition.empty() ? "(starting)" : progress.current_partition) << "\n";
    out << "[" << render_bar(std::max(20, content_width - 8), progress.percent) << "] " << progress.percent << "%\n";
    if (!progress.detail.empty()) out << fit_text(progress.detail, content_width) << "\n";
    out << std::string(std::max(0, content_width), '-') << "\n";

    int skip = std::max(0, static_cast<int>(progress.logs.size()) - log_lines);
    for (int i = skip; i < static_cast<int>(progress.logs.size()); ++i) {
        out << fit_text(progress.logs[i], content_width) << "\n";
    }

    if (progress.finished) {
        out << std::string(std::max(0, content_width), '-') << "\n";
        out << (progress.exit_code == 0 ? "Flash process finished successfully. Press q to exit." :
                                          "Flash process exited with errors. Press q to exit.") << "\n";
    }

    std::cout << out.str() << std::flush;
}

void parse_flash_line(const std::string& line, FlashProgress& progress) {
    static const std::regex part_re("^\\[([^\\]]+)\\]");
    static const std::regex prog_re("Progress: ([0-9]+)%");
    std::smatch match;

    if (std::regex_search(line, match, part_re)) {
        progress.current_partition = match[1];
    }
    if (std::regex_search(line, match, prog_re)) {
        progress.percent = std::stoi(match[1]);
        progress.detail = line;
    } else if (!line.empty()) {
        progress.detail = line;
    }
}

int run_child_flash(const fs::path& exe_path, const fs::path& pac_path, const std::vector<PartitionItem>& items, const SessionConfig& config) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        std::perror("pipe");
        return 1;
    }

    std::vector<std::string> args;
    args.push_back(exe_path.string());
    args.push_back(pac_path.string());
    args.push_back(config.flash_mode == FlashMode::Preserve ? "--preserve-layout" : "--full-flash");
    if (config.disable_transcode) args.push_back("--disable-transcode");
    if (config.debug_fast_lane) args.push_back("--debug-fast-lane");
    for (const auto& item : items) {
        if (item.selected) {
            args.push_back("--partition");
            args.push_back(item.id);
        }
    }

    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& arg : args) argv.push_back(arg.data());
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid == 0) {
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[0]);
        close(pipefd[1]);
        execv(argv[0], argv.data());
        _exit(127);
    }

    close(pipefd[1]);
    fcntl(pipefd[0], F_SETFL, O_NONBLOCK);

    FlashProgress progress;
    std::string buffer;
    TerminalGuard raw_guard;
    raw_guard.enable_raw();
    AltScreenGuard screen_guard;

    while (!progress.finished) {
        char tmp[4096];
        ssize_t n = read(pipefd[0], tmp, sizeof(tmp));
        if (n > 0) {
            buffer.append(tmp, static_cast<size_t>(n));
            size_t pos = 0;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                push_log(progress.logs, line, 200);
                parse_flash_line(line, progress);
                buffer.erase(0, pos + 1);
            }
        }

        int status = 0;
        pid_t done = waitpid(pid, &status, WNOHANG);
        if (done == pid) {
            progress.finished = true;
            progress.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
            if (!buffer.empty()) {
                push_log(progress.logs, buffer, 200);
                parse_flash_line(buffer, progress);
                buffer.clear();
            }
        }

        render_flash(pac_path, config, progress);
        Key key = read_key();
        if (progress.finished && key == Key::Quit) break;
        usleep(30000);
    }

    close(pipefd[0]);
    return progress.exit_code;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]) == "--help") {
        std::cerr << "Usage: uflash-tui <firmware.pac>\n";
        return argc < 2 ? 1 : 0;
    }

    fs::path pac_path = argv[1];
    std::vector<PartitionItem> items;
    try {
        items = load_partitions(pac_path);
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << "\n";
        return 1;
    }

    SessionConfig config;
    TerminalGuard raw_guard;
    if (!raw_guard.enable_raw()) {
        std::cerr << "error: could not enable raw terminal mode\n";
        return 1;
    }
    AltScreenGuard screen_guard;

    size_t cursor = 0;
    size_t scroll = 0;

    while (true) {
        ScreenSize sz = get_screen_size();
        int visible_rows = std::max(5, sz.rows - 11);
        if (cursor < scroll) scroll = cursor;
        if (cursor >= scroll + static_cast<size_t>(visible_rows)) scroll = cursor - static_cast<size_t>(visible_rows) + 1;

        render_selector(pac_path, items, cursor, scroll, config);
        Key key = read_key();
        switch (key) {
            case Key::Up:
                if (cursor > 0) --cursor;
                break;
            case Key::Down:
                if (cursor + 1 < items.size()) ++cursor;
                break;
            case Key::Space:
                items[cursor].selected = !items[cursor].selected;
                break;
            case Key::ToggleAll: {
                bool any_unselected = std::any_of(items.begin(), items.end(), [](const auto& item) { return !item.selected; });
                for (auto& item : items) item.selected = any_unselected;
                break;
            }
            case Key::ToggleModePreserve:
                config.flash_mode = FlashMode::Preserve;
                break;
            case Key::ToggleModeFull:
                config.flash_mode = FlashMode::Full;
                break;
            case Key::ToggleTranscode:
                config.disable_transcode = !config.disable_transcode;
                break;
            case Key::ToggleDebugFast:
                config.debug_fast_lane = !config.debug_fast_lane;
                break;
            case Key::Enter:
            case Key::Start: {
                size_t selected = 0;
                for (const auto& item : items) if (item.selected) ++selected;
                if (selected == 0) break;
                raw_guard.disable_raw();
                fs::path exe_path = fs::absolute(argv[0]).parent_path() / "uflash";
                int exit_code = run_child_flash(exe_path, pac_path, items, config);
                return exit_code;
            }
            case Key::Quit:
                return 0;
            case Key::None:
            case Key::Left:
            case Key::Right:
                break;
        }
        usleep(16000);
    }
}
