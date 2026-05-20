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

#include <libusb.h>

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
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;
using namespace ftxui;

// ─── device presence polling ──────────────────────────────────────────────────
// Non-claiming scan: only reads descriptors, never opens or claims the device.
// Safe to run while the child uflash process holds the device.

static std::atomic<int> g_device_present{0}; // 1 = Unisoc device visible on USB

static void device_poller(std::atomic<bool>& stop) {
    constexpr uint16_t kVid  = 0x1782;
    constexpr uint16_t kPids[] = {0x4d00, 0x5d00, 0x3d00};
    while (!stop) {
        bool found = false;
        libusb_context* ctx = nullptr;
        if (libusb_init(&ctx) == 0) {
            libusb_device** devs = nullptr;
            ssize_t n = libusb_get_device_list(ctx, &devs);
            if (n >= 0) {
                for (ssize_t i = 0; i < n && !found; ++i) {
                    libusb_device_descriptor d{};
                    if (libusb_get_device_descriptor(devs[i], &d) == 0 && d.idVendor == kVid)
                        for (auto pid : kPids) if (d.idProduct == pid) { found = true; break; }
                }
                libusb_free_device_list(devs, 1);
            }
            libusb_exit(ctx);
        }
        g_device_present = found ? 1 : 0;
        // Sleep in 50 ms slices so the stop flag is honoured quickly.
        for (int ms = 0; ms < 800 && !stop; ms += 50)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

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
    std::string stage     = "Connecting";
    std::string sub_stage;
    bool        fast_lane = false;
    int     percent    = 0;
    double  speed      = 0.0;
    double  mib_done   = 0.0;
    double  mib_total  = 0.0;
    size_t  chunk_bytes = 0;
    int     active_step = 0;  // 0=Connect 1=BSL 2=FDL1 3=FDL2 4=Init 5=Flash
    int     steps_done  = 0;  // bitmask: bit i set when step i completes
    std::deque<std::string> log;
    std::unordered_set<std::string> done_partitions;
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
    // Partition header: "[ID] (block) Base: 0x..." — anchored to "Base:" to
    // avoid matching [no-transcode] or other bracketed tokens.
    static const std::regex part_hdr_re(R"(\[([A-Za-z0-9_\-\.]+)\].*Base:)");
    // Progress: N% (x/y MiB, speed MiB/s, chunk=N)
    static const std::regex prog_re(
        R"(Progress:\s*(\d+)%\s*\(([0-9.]+)/([0-9.]+) MiB[^,]*, ([0-9.]+) MiB/s.*chunk=(\d+))");
    static const std::regex prog_ok_re(R"(Progress:\s*100%\s*-\s*OK)");

    std::smatch m;

    // ── stage + step transitions ──────────────────────────────────────────────
    auto has = [&](const char* s) { return line.find(s) != std::string::npos; };
    // Mark steps 0..through done and optionally advance active step
    auto advance = [&](int through, int next_active) {
        for (int i = 0; i <= through; ++i) st.steps_done |= (1 << i);
        st.active_step = next_active;
    };

    if (has("Found Unisoc Device")) {
        st.stage = "Device found"; advance(0, 0);
    } else if (has("PAC components extracted")) {
        st.stage = "Unpacking"; advance(0, 0);
    } else if (has("Detected FDL1/FDL2 already running")) {
        st.stage = "FDL running"; advance(3, 4);
    } else if (has("Handshake successful") || has("Host Protocol Response")) {
        st.stage = "Handshake OK"; advance(1, 1);
    } else if (has("Legacy 0x7E handshake failed")) {
        st.stage = "Handshaking"; st.sub_stage = "Retry 0xAE"; advance(0, 1);
    } else if (has("Sending 0x7E") || has("Sending 0xAE") || has("Handshake")) {
        st.stage = "Handshaking"; advance(0, 1);
    } else if (has("FDL not detected") || has("Loading Stage 1")) {
        st.stage = "Loading FDL1"; st.sub_stage = "Transferring"; advance(1, 2);
    } else if (has("FDL1 is running")) {
        st.stage = "FDL1 OK"; st.sub_stage = "Executing"; advance(2, 2);
    } else if (has("Waiting for re-enumeration")) {
        st.sub_stage = "Re-enumerating";
    } else if (has("Attempting Stage 2 Handshake")) {
        st.sub_stage = "Stage 2 sync"; advance(2, 3);
    } else if (has("Starting Stage 2") || has("Stage 2 (FDL2)")) {
        st.stage = "Loading FDL2"; st.sub_stage = "Transferring"; advance(2, 3);
    } else if (has("ChangeBaud") || has("current link settings")) {
        st.sub_stage = "Baud sync";
    } else if (has("Executing FDL2") || has("ExecNandInit")) {
        st.stage = "ExecNandInit"; st.sub_stage = "Waiting"; advance(3, 4);
    } else if (has("ExecNandInit OK")) {
        st.sub_stage = "OK";
    } else if (has("layout mismatch") || has("will repartition")) {
        st.sub_stage = "Layout mismatch";
    } else if (has("DisableTransCode enabled")) {
        st.sub_stage = "Fast mode ON";
    } else if (has("Backing up") && has("NV")) {
        st.sub_stage = "NV backup";
    } else if (has("Sending partition table")) {
        st.sub_stage = "Writing table";
    } else if (has("Repartition OK")) {
        st.sub_stage = "OK";
    } else if (has("Repartition") || has("repartition")) {
        st.stage = "Repartitioning"; st.active_step = 4;
    } else if (has("Partition Flashing Phase")) {
        st.stage = "Flashing"; advance(4, 5);
    }

    // fast-lane detection
    if (has("[no-transcode]")) st.fast_lane = true;

    // ── partition header ──────────────────────────────────────────────────────
    if (std::regex_search(line, m, part_hdr_re)) {
        st.partition  = m[1];
        st.percent    = 0;
        st.mib_done   = 0.0;
        st.mib_total  = 0.0;
        st.speed      = 0.0;
        st.chunk_bytes = 0;
        st.sub_stage  = "Starting";
        st.stage      = "Flashing";
    }

    // ── sub-stage transitions ─────────────────────────────────────────────────
    if (has("Streaming"))                           { st.sub_stage = "Streaming";  }
    if (has("Data transfer complete"))              { st.sub_stage = "Verifying";  }

    // ── partition completion ──────────────────────────────────────────────────
    if (std::regex_search(line, m, prog_ok_re)) {
        if (!st.partition.empty()) st.done_partitions.insert(st.partition);
        st.percent   = 100;
        st.sub_stage = "Done";
    }

    // ── progress numbers ──────────────────────────────────────────────────────
    if (std::regex_search(line, m, prog_re)) {
        st.percent     = std::stoi(m[1]);
        st.mib_done    = std::stod(m[2]);
        st.mib_total   = std::stod(m[3]);
        st.speed       = std::stod(m[4]);
        st.chunk_bytes = static_cast<size_t>(std::stoull(m[5]));
        if (st.sub_stage != "Verifying" && st.sub_stage != "Done")
            st.sub_stage = "Streaming";
    }
}

// ─── shared header ────────────────────────────────────────────────────────────

static Element make_header(const std::string& pac_name, const Opts& opts) {
    Element dev_elem = g_device_present.load()
        ? hbox({ text(" ● ") | bold | color(Color::Green),  text("ready ") | color(Color::GrayDark) })
        : hbox({ text(" ◌ ") | color(Color::GrayDark),      text("waiting ") | color(Color::GrayDark) });

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
                | color(opts.disable_transcode ? Color(Color::Cyan1) : Color(Color::GrayDark)),
            separatorLight(),
            dev_elem,
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

    std::atomic<bool> ticking{true};
    std::thread tick([&] {
        while (ticking) {
            std::this_thread::sleep_for(std::chrono::milliseconds(900));
            if (ticking) screen.PostEvent(Event::Custom);
        }
    });
    screen.Loop(component);
    ticking = false;
    tick.join();
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
                // Split on \r and \n so that progress lines (emitted with \r)
                // are parsed live rather than only when the next \n arrives.
                while ((pos = buf.find_first_of("\r\n")) != std::string::npos) {
                    auto line = buf.substr(0, pos);
                    buf.erase(0, pos + 1);
                    if (line.empty()) continue;  // skip \r\n double-delimiter
                    {
                        std::lock_guard<std::mutex> lk(st.mtx);
                        bool is_progress = line.find("Progress:") != std::string::npos;
                        // Progress lines overwrite in place — keep only the latest
                        // in the log so it doesn't flood the log panel.
                        if (is_progress && !st.log.empty()
                                && st.log.back().find("Progress:") != std::string::npos) {
                            st.log.back() = line;
                        } else {
                            st.log.push_back(line);
                        }
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
        int term_h = Terminal::Size().dimy;

        // ── stage + status line ──────────────────────────────────────────────
        Element status_elem;
        if (!st.done) {
            // Build the detail suffix: sub_stage + fast-lane + chunk size
            Elements detail;
            if (!st.sub_stage.empty()) {
                detail.push_back(text("  ·  ") | color(Color::GrayDark));
                detail.push_back(text(st.sub_stage) | color(Color::White));
            }
            if (st.fast_lane) {
                detail.push_back(text("  ·  ") | color(Color::GrayDark));
                detail.push_back(text("no-transcode") | color(Color::Cyan1));
            }
            if (st.chunk_bytes > 0) {
                std::string cs = std::to_string(st.chunk_bytes / 1024) + " KB";
                detail.push_back(text("  ·  chunk: ") | color(Color::GrayDark));
                detail.push_back(text(cs) | color(Color::GrayLight));
            }
            Elements row;
            row.push_back(spinner(3, frame) | color(Color::Cyan1));
            row.push_back(text("  ") );
            row.push_back(text(st.stage) | color(Color::GrayDark));
            if (!st.partition.empty()) {
                row.push_back(text(": ") | color(Color::GrayDark));
                row.push_back(text(st.partition) | bold | color(Color::White));
            }
            for (auto& e : detail) row.push_back(e);
            status_elem = hbox(std::move(row));
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
        // Explicit Color() wrapping required: Palette16 vs Palette256.
        Color bar_col = st.done
            ? (st.exit_code == 0 ? Color(Color::Green) : Color(Color::Red))
            : Color(Color::Cyan1);
        auto gauge_row = hbox({
            gauge(frac) | flex | color(bar_col),
            text(" " + std::to_string(st.percent) + "%") | bold
                | size(WIDTH, EQUAL, 5),
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

        // ── step strip ───────────────────────────────────────────────────────
        static constexpr const char* kStepLabels[] =
            {"Connect", "BSL", "FDL1", "FDL2", "Init", "Flash"};
        int eff_done = (st.done && st.exit_code == 0) ? 0x3f : st.steps_done;
        Elements step_elems;
        for (int i = 0; i < 6; ++i) {
            if (i > 0) step_elems.push_back(text("  ·  ") | color(Color::GrayDark));
            bool sdone   = (eff_done >> i) & 1;
            bool sactive = !st.done && (i == st.active_step);
            if (sdone) {
                step_elems.push_back(hbox({
                    text("✓ ") | color(Color::Green),
                    text(kStepLabels[i]) | color(Color::GrayDark),
                }));
            } else if (sactive) {
                step_elems.push_back(hbox({
                    spinner(3, frame) | color(Color::Cyan1),
                    text(" "),
                    text(kStepLabels[i]) | bold | color(Color::White),
                }));
            } else {
                step_elems.push_back(text(kStepLabels[i]) | color(Color::GrayDark));
            }
        }

        auto progress_panel = window(
            text(" Progress ") | bold,
            vbox({
                hbox(std::move(step_elems)),
                separatorLight(),
                status_elem,
                gauge_row,
                stats_elem,
            })
        );

        // ── partition list ───────────────────────────────────────────────────
        Elements part_rows;
        for (const auto& p : items) {
            if (!p.selected) continue;
            bool is_done    = st.done_partitions.count(p.id) > 0;
            bool is_active  = !is_done && (p.id == st.partition);

            Element sym;
            if (is_done)        sym = text(" ✓ ") | color(Color::Green);
            else if (is_active) sym = spinner(5, frame) | color(Color::Cyan1);
            else                sym = text("   ") | color(Color::GrayDark);

            auto id_elem = text(p.id)
                | (is_active ? bold : nothing)
                | color(is_done ? Color::GrayDark : (is_active ? Color::White : Color::GrayLight));

            auto block_elem = text(p.block.empty() ? "" : "  " + p.block)
                | color(Color::GrayDark)
                | size(WIDTH, EQUAL, 18);

            Element right;
            if (is_done) {
                right = text("done") | color(Color::Green) | size(WIDTH, EQUAL, 22);
            } else if (is_active && st.mib_total > 0.0) {
                float f = float(std::max(0, std::min(st.percent, 100))) / 100.0f;
                std::ostringstream s;
                s << std::fixed << std::setprecision(1) << st.speed << " MiB/s";
                right = hbox({
                    gauge(f) | size(WIDTH, EQUAL, 14) | color(Color::Cyan1),
                    text(" " + std::to_string(st.percent) + "%") | bold
                        | size(WIDTH, EQUAL, 5),
                    text("  " + s.str()) | color(Color::GrayDark),
                });
            } else if (is_active) {
                right = hbox({
                    spinner(3, frame) | color(Color::Cyan1),
                    text("  " + (st.sub_stage.empty() ? std::string("starting…") : st.sub_stage))
                        | color(Color::GrayDark),
                }) | size(WIDTH, EQUAL, 22);
            } else {
                right = text("pending") | color(Color::GrayDark) | size(WIDTH, EQUAL, 22);
            }

            part_rows.push_back(hbox({
                sym,
                id_elem | size(WIDTH, EQUAL, 20),
                block_elem,
                right,
            }));
        }
        auto parts_panel = window(
            text(" Partitions ") | bold,
            part_rows.empty()
                ? vbox({ text("(none selected)") | color(Color::GrayDark) })
                : vbox(std::move(part_rows))
        );

        // ── log panel ────────────────────────────────────────────────────────
        // Give partitions list + progress about 12 rows; rest goes to log.
        int fixed_rows = 4   // header
                       + 7   // progress panel (step strip + separator + status + gauge + stats)
                       + 2 + static_cast<int>(
                             std::count_if(items.begin(), items.end(),
                                           [](const PartitionItem& p){ return p.selected; }));
        int log_visible = std::max(3, term_h - fixed_rows - 2);
        int skip = std::max(0, (int)st.log.size() - log_visible);
        Elements log_rows;
        for (int i = skip; i < (int)st.log.size(); ++i) {
            const auto& ln = st.log[i];
            // Skip noisy fast-lane packet debug lines from the log panel
            if (ln.find("fast lane debug:") != std::string::npos) continue;
            Color c = Color::Default;
            if (ln.find("error") != std::string::npos || ln.find("Error") != std::string::npos
                || ln.find("failed") != std::string::npos)
                c = Color::Red;
            else if (ln.find("warning") != std::string::npos || ln.find("Warning") != std::string::npos)
                c = Color::Yellow;
            else if (ln.find("OK") != std::string::npos || ln.find("complete") != std::string::npos
                     || ln.find("✓") != std::string::npos)
                c = Color::Green;
            else if (ln.find("Progress:") != std::string::npos)
                c = Color::Cyan1;
            auto display = ln.size() > 120 ? ln.substr(0, 120) + "…" : ln;
            log_rows.push_back(text(display) | color(c));
        }
        if (log_rows.empty())
            log_rows.push_back(text("(waiting for output…)") | color(Color::GrayDark));

        auto log_panel = window(text(" Log "), vbox(std::move(log_rows)));

        return vbox({
            make_header(pac_name, opts),
            progress_panel,
            parts_panel,
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

    std::atomic<bool> stop_poller{false};
    std::thread poller_thd([&]{ device_poller(stop_poller); });

    int result = 0;
    if (run_selector(pac_path, items, opts)) {
        // Stop the poller before fork()ing uflash.  On macOS, libusb uses IOKit
        // Mach ports; forking while those are live leaves the child with stale
        // port state, which causes libusb_claim_interface in uflash to block
        // indefinitely.  Stopping the poller here drains any in-flight libusb
        // context before we hand control to the child.
        stop_poller = true;
        poller_thd.join();

        fs::path exe = fs::absolute(argv[0]).parent_path() / "uflash";
        result = run_flash(exe, pac_path, items, opts);
    } else {
        stop_poller = true;
        poller_thd.join();
    }
    return result;
}
