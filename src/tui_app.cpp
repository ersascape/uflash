// uflash-tui  — FTXUI-based frontend for uflash
//
// Screen 1: partition selector (j/k, space, a, p/f, t, enter)
// Screen 2: live flash progress with animated gauge + log

#include "upac/pac_reader.h"
#include "upac/xml_config.h"

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/color.hpp>
#include <ftxui/screen/terminal.hpp>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace ftxui;

// ─── types ────────────────────────────────────────────────────────────────────

struct PartitionItem {
    std::string id, block, type;
    uint64_t size = 0;
    bool selected = true;
};

struct Opts {
    bool preserve_layout   = true;
    bool disable_transcode = false;
    bool debug_fast_lane   = false;
};

struct FlashState {
    std::string partition;
    int     percent   = 0;
    double  speed     = 0.0;
    double  mib_done  = 0.0;
    double  mib_total = 0.0;
    std::deque<std::string> log;
    bool done      = false;
    int  exit_code = -1;
    std::mutex mtx;
};

// ─── helpers ──────────────────────────────────────────────────────────────────

static std::string human_size(uint64_t b) {
    if (b == 0) return "—";
    std::ostringstream s;
    s << std::fixed;
    if      (b >= (1ULL << 30)) s << std::setprecision(1) << b / double(1ULL << 30) << " GiB";
    else if (b >= (1ULL << 20)) s << std::setprecision(0) << b / double(1ULL << 20) << " MiB";
    else                         s << b << " B";
    return s.str();
}

static std::vector<PartitionItem> load_partitions(const fs::path& pac_path) {
    auto r = upac::PacReader::open(pac_path);
    if (!r) throw std::runtime_error("cannot open PAC");
    upac::XmlProductConfig cfg;
    if (!upac::parse_xml_config(r->xml_config(), cfg))
        throw std::runtime_error("cannot parse PAC XML");
    auto infos = r->file_infos();
    std::vector<PartitionItem> out;
    for (auto& fc : cfg.files) {
        if (fc.id == "FDL" || fc.id == "FDL2" || fc.type == "FDL") continue;
        PartitionItem p;
        p.id = fc.id; p.block = fc.id_name; p.type = fc.type;
        p.selected = fc.type.find("Erase") == std::string::npos;
        for (auto& fi : infos) {
            std::string fid = fi.id;
            fid.erase(std::find(fid.begin(), fid.end(), '\0'), fid.end());
            if (fid == fc.id) { p.size = fi.size; break; }
        }
        out.push_back(std::move(p));
    }
    return out;
}

static void parse_flash_line(const std::string& line, FlashState& st) {
    // Extract: [PartName] and Progress: N% (done/total MiB, speed MiB/s
    static const std::regex part_re(R"(\[([A-Za-z0-9_\-\.]+)\])");
    static const std::regex prog_re(R"(Progress:\s*(\d+)%\s*\(([0-9.]+)/([0-9.]+) MiB[^,]*, ([0-9.]+) MiB/s)");
    std::smatch m;
    if (std::regex_search(line, m, part_re)) st.partition = m[1];
    if (std::regex_search(line, m, prog_re)) {
        st.percent   = std::stoi(m[1]);
        st.mib_done  = std::stod(m[2]);
        st.mib_total = std::stod(m[3]);
        st.speed     = std::stod(m[4]);
    }
}

// ─── shared header ────────────────────────────────────────────────────────────

static Element make_header(const std::string& pac_name, const Opts& opts) {
    return window(
        hbox({ text(" ⚡ uflash ") | bold | color(Color::Cyan1) }),
        hbox({
            text(pac_name) | flex | color(Color::White),
            separatorLight(),
            text(opts.preserve_layout ? " preserve " : " full-flash ")
                | color(opts.preserve_layout ? Color::Green : Color::Yellow),
            separatorLight(),
            text(" fast: ") | color(Color::GrayDark),
            text(opts.disable_transcode ? "on " : "off ")
                | color(opts.disable_transcode ? Color::Cyan1 : Color::GrayDark),
        })
    );
}

// ─── selector ─────────────────────────────────────────────────────────────────

static bool run_selector(const fs::path& pac_path,
                         std::vector<PartitionItem>& items,
                         Opts& opts) {
    auto screen = ScreenInteractive::Fullscreen();
    int cursor = 0;
    int scroll  = 0;
    bool started = false;
    std::string pac_name = pac_path.filename().string();

    auto component = Renderer([&] {
        int sel_count = 0;
        uint64_t sel_bytes = 0;
        for (auto& p : items) {
            if (p.selected) { ++sel_count; sel_bytes += p.size; }
        }

        int visible = std::max(5, Terminal::Size().dimy - 16);
        if (cursor < scroll) scroll = cursor;
        if (cursor >= scroll + visible) scroll = cursor - visible + 1;

        Elements rows;
        for (int i = scroll; i < scroll + visible && i < (int)items.size(); ++i) {
            auto& p  = items[i];
            bool active = i == cursor;

            auto check_sym = text(p.selected ? " ✓ " : "   ")
                | color(p.selected ? Color::Green : Color::GrayDark);
            auto name_elem = text(p.id);
            if (active) name_elem = name_elem | bold;
            auto block_elem = text(p.block.empty() ? "—" : p.block)
                | color(Color::GrayDark) | size(WIDTH, EQUAL, 16);
            auto size_elem  = text(human_size(p.size))
                | color(Color::Cyan1) | size(WIDTH, EQUAL, 10);

            auto row = hbox({
                text("[") | color(Color::GrayDark),
                check_sym,
                text("] ") | color(Color::GrayDark),
                name_elem | flex,
                block_elem,
                text("  "),
                size_elem,
            });
            if (active) row = row | inverted;
            rows.push_back(row);
        }

        auto list_panel = window(
            text(" Partitions ") | bold,
            vbox({
                vbox(std::move(rows)),
                separatorLight(),
                hbox({
                    text(std::to_string(sel_count) + "/" +
                         std::to_string(items.size()) + " selected"),
                    text("  ·  ") | color(Color::GrayDark),
                    text(human_size(sel_bytes)) | color(Color::Cyan1),
                }) | color(Color::GrayLight),
            })
        );

        auto opt_text = [](const std::string& s, bool on, Color on_col) -> Element {
            auto e = text(s);
            return on ? (e | bold | color(on_col)) : (e | color(Color::GrayDark));
        };
        auto opts_panel = window(
            text(" Options "),
            hbox({
                text("Mode: ") | color(Color::GrayDark),
                opt_text("Preserve", opts.preserve_layout,  Color::Green),
                text("  /  ") | color(Color::GrayDark),
                opt_text("Full",     !opts.preserve_layout, Color::Yellow),
                text("   "),
                separatorLight(),
                text("  Fast: ")  | color(Color::GrayDark),
                opt_text(opts.disable_transcode ? "ON " : "off", opts.disable_transcode, Color::Cyan1),
                text("   "),
                separatorLight(),
                text("  Debug: ") | color(Color::GrayDark),
                opt_text(opts.debug_fast_lane ? "ON" : "off", opts.debug_fast_lane, Color::Yellow),
            })
        );

        auto hints = hbox({
            text("↑↓") | color(Color::Cyan1), text("/jk nav  "),
            text("space") | color(Color::Cyan1), text(" toggle  "),
            text("a") | color(Color::Cyan1), text(" all  "),
            text("p/f") | color(Color::Cyan1), text(" mode  "),
            text("t") | color(Color::Cyan1), text(" fast  "),
            text("g") | color(Color::Cyan1), text(" debug  "),
            text("enter") | color(Color::Green) | bold, text(" start  "),
            text("q") | color(Color::Red), text(" quit"),
        }) | color(Color::GrayDark);

        return vbox({
            make_header(pac_name, opts),
            list_panel,
            opts_panel,
            hints,
        });
    });

    component = CatchEvent(component, [&](Event e) -> bool {
        if (e == Event::ArrowUp    || e == Event::Character('k')) {
            if (cursor > 0) --cursor; return true;
        }
        if (e == Event::ArrowDown  || e == Event::Character('j')) {
            if (cursor + 1 < (int)items.size()) ++cursor; return true;
        }
        if (e == Event::Character(' ')) {
            items[cursor].selected = !items[cursor].selected; return true;
        }
        if (e == Event::Character('a') || e == Event::Character('A')) {
            bool any_off = std::any_of(items.begin(), items.end(),
                                       [](auto& p) { return !p.selected; });
            for (auto& p : items) p.selected = any_off;
            return true;
        }
        if (e == Event::Character('p') || e == Event::Character('P')) { opts.preserve_layout = true;  return true; }
        if (e == Event::Character('f') || e == Event::Character('F')) { opts.preserve_layout = false; return true; }
        if (e == Event::Character('t') || e == Event::Character('T')) { opts.disable_transcode = !opts.disable_transcode; return true; }
        if (e == Event::Character('g') || e == Event::Character('G')) { opts.debug_fast_lane = !opts.debug_fast_lane; return true; }
        if (e == Event::Return || e == Event::Character('s') || e == Event::Character('S')) {
            if (std::any_of(items.begin(), items.end(), [](auto& p) { return p.selected; })) {
                started = true;
                screen.ExitLoopClosure()();
            }
            return true;
        }
        if (e == Event::Character('q') || e == Event::Character('Q')) {
            screen.ExitLoopClosure()();
            return true;
        }
        return false;
    });

    screen.Loop(component);
    return started;
}

// ─── flash screen ─────────────────────────────────────────────────────────────

static int run_flash(const fs::path& exe, const fs::path& pac_path,
                     const std::vector<PartitionItem>& items, const Opts& opts) {
    // Build argv
    std::vector<std::string> arg_strs;
    arg_strs.push_back(exe.string());
    arg_strs.push_back(pac_path.string());
    arg_strs.push_back(opts.preserve_layout ? "--preserve-layout" : "--full-flash");
    if (opts.disable_transcode) arg_strs.push_back("--disable-transcode");
    if (opts.debug_fast_lane)   arg_strs.push_back("--debug-fast-lane");
    for (auto& p : items)
        if (p.selected) { arg_strs.push_back("--partition"); arg_strs.push_back(p.id); }
    std::vector<char*> argv_v;
    for (auto& s : arg_strs) argv_v.push_back(s.data());
    argv_v.push_back(nullptr);

    int pipefd[2];
    if (pipe(pipefd) != 0) return 1;

    pid_t pid = fork();
    if (pid == 0) {
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[0]); close(pipefd[1]);
        execv(argv_v[0], argv_v.data());
        _exit(127);
    }
    close(pipefd[1]);
    fcntl(pipefd[0], F_SETFL, O_NONBLOCK);

    FlashState st;
    std::atomic<size_t> frame{0};
    std::atomic<bool>   all_done{false};
    std::string pac_name = pac_path.filename().string();
    auto screen = ScreenInteractive::Fullscreen();

    // I/O reader thread — reads child stdout/stderr into st.log
    std::thread io_thread([&] {
        std::string buf;
        while (true) {
            char tmp[4096];
            ssize_t n = ::read(pipefd[0], tmp, sizeof(tmp));
            if (n > 0) {
                buf.append(tmp, static_cast<size_t>(n));
                size_t pos;
                while ((pos = buf.find('\n')) != std::string::npos) {
                    auto line = buf.substr(0, pos);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    buf.erase(0, pos + 1);
                    {
                        std::lock_guard<std::mutex> lk(st.mtx);
                        st.log.push_back(line);
                        if (st.log.size() > 300) st.log.pop_front();
                        parse_flash_line(line, st);
                    }
                    screen.PostEvent(Event::Custom);
                }
            } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                break;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
            }
        }
    });

    // Reaper thread — waits for child, marks done
    std::thread wait_thread([&] {
        int status = 0;
        waitpid(pid, &status, 0);
        {
            std::lock_guard<std::mutex> lk(st.mtx);
            st.done      = true;
            st.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
        }
        all_done = true;
        screen.PostEvent(Event::Custom);
    });

    // Animation tick thread — drives spinner + periodic redraws
    std::thread anim_thread([&] {
        while (!all_done) {
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
            ++frame;
            screen.PostEvent(Event::Custom);
        }
    });

    auto component = Renderer([&] {
        std::lock_guard<std::mutex> lk(st.mtx);

        // ── spinner + current partition ──────────────────────────────────────
        Element status_elem;
        if (!st.done) {
            status_elem = hbox({
                spinner(3, frame) | color(Color::Cyan1),
                text("  Flashing: ") | color(Color::GrayDark),
                text(st.partition.empty() ? "(connecting…)" : st.partition)
                    | bold | color(Color::White),
            });
        } else if (st.exit_code == 0) {
            status_elem = hbox({
                text(" ✓ ") | bold | color(Color::Green),
                text("Flash complete") | bold | color(Color::Green),
                text("  —  press ") | color(Color::GrayDark),
                text("q") | color(Color::Cyan1),
                text(" to exit") | color(Color::GrayDark),
            });
        } else {
            status_elem = hbox({
                text(" ✗ ") | bold | color(Color::Red),
                text("Flash failed (exit " + std::to_string(st.exit_code) + ")")
                    | bold | color(Color::Red),
                text("  —  press ") | color(Color::GrayDark),
                text("q") | color(Color::Cyan1),
                text(" to exit") | color(Color::GrayDark),
            });
        }

        // ── progress gauge ───────────────────────────────────────────────────
        float frac = float(std::max(0, std::min(st.percent, 100))) / 100.0f;
        Color bar_col = st.done
            ? (st.exit_code == 0 ? Color::Green : Color::Red)
            : Color::Cyan1;
        auto gauge_row = hbox({
            gauge(frac) | flex | color(bar_col),
            text(" " + std::to_string(st.percent) + "%") | bold,
        });

        // ── stats line ───────────────────────────────────────────────────────
        std::ostringstream stats;
        if (st.mib_total > 0.0) {
            stats << std::fixed << std::setprecision(1)
                  << st.mib_done << " / " << st.mib_total << " MiB";
            if (st.speed > 0.1) {
                stats << "  ·  " << std::setprecision(1) << st.speed << " MiB/s";
                double rem_s = (st.mib_total - st.mib_done) / st.speed;
                int eta = static_cast<int>(rem_s);
                if (eta > 0) {
                    stats << "  ·  ETA ";
                    if (eta >= 60) stats << (eta / 60) << "m ";
                    stats << (eta % 60) << "s";
                }
            }
        }
        auto stats_elem = text(stats.str()) | color(Color::GrayDark);

        auto progress_panel = window(
            text(" Progress ") | bold,
            vbox({ status_elem, gauge_row, stats_elem })
        );

        // ── log panel ────────────────────────────────────────────────────────
        int log_visible = std::max(5, Terminal::Size().dimy - 14);
        int skip = std::max(0, (int)st.log.size() - log_visible);
        Elements log_rows;
        for (int i = skip; i < (int)st.log.size(); ++i) {
            const auto& ln = st.log[i];
            Color c = Color::Default;
            if (ln.find("error") != std::string::npos
                || ln.find("Error") != std::string::npos
                || ln.find("failed") != std::string::npos)
                c = Color::Red;
            else if (ln.find("warning") != std::string::npos
                     || ln.find("Warning") != std::string::npos)
                c = Color::Yellow;
            else if (ln.find("OK") != std::string::npos
                     || ln.find("complete") != std::string::npos)
                c = Color::Green;
            else if (ln.find("Progress:") != std::string::npos)
                c = Color::Cyan1;
            auto display = ln.size() > 200 ? ln.substr(0, 200) + "…" : ln;
            log_rows.push_back(text(display) | color(c));
        }
        if (log_rows.empty()) log_rows.push_back(text("(waiting for output…)") | color(Color::GrayDark));

        auto log_panel = window(text(" Log "), vbox(std::move(log_rows)));

        return vbox({
            make_header(pac_name, opts),
            progress_panel,
            log_panel,
        });
    });

    component = CatchEvent(component, [&](Event e) -> bool {
        if (e == Event::Character('q') || e == Event::Character('Q')) {
            std::lock_guard<std::mutex> lk(st.mtx);
            if (st.done) { screen.ExitLoopClosure()(); return true; }
        }
        return false;
    });

    screen.Loop(component);

    all_done = true;
    close(pipefd[0]);
    anim_thread.join();
    io_thread.join();
    wait_thread.join();

    std::lock_guard<std::mutex> lk(st.mtx);
    return st.exit_code;
}

// ─── main ─────────────────────────────────────────────────────────────────────

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

    Opts opts;
    if (!run_selector(pac_path, items, opts)) return 0;

    fs::path exe = fs::absolute(argv[0]).parent_path() / "uflash";
    return run_flash(exe, pac_path, items, opts);
}
