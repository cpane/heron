// File        : IBusReader.cpp
// Author      : Chris Pane
// Description : Learning and debugging tool for the FlySky iBUS remote
//               control link on the Pi's UART, built on heron_ibus.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> ibus_reader monitor
//     build-host/bin/ibus_reader monitor --device /tmp/ibus   (with ibus_fake.py)
//
// A bench tool, so it prints; the reusable parts (port, parser, stats, link
// rule, normalization) are in heron_ibus and heron_ibus_port and do not.
// Receive-only: the port is opened read-only. Subcommands and their output
// are described in docs/remote-ibus-reader.md, the protocol in
// docs/remote-ibus-protocol.md.

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "heron/remote/IBusParser.h"
#include "heron/remote/IBusPort.h"
#include "heron/remote/LinkMonitor.h"

namespace {

using namespace Heron::Remote;
using std::chrono::milliseconds;

volatile std::sig_atomic_t s_stopRequested = 0;

void onSignal(int) {
  s_stopRequested = 1;
}

/** How long to wait before saying "no data", and repeating it. */
constexpr auto kNoDataHintAfter = std::chrono::seconds(2);
constexpr milliseconds kPumpTimeout(20);

// ---------------------------------------------------------------------------
// Options

struct Options {
  std::string command;
  SerialConfig serial;
  milliseconds staleAfter{100};
  std::int64_t count{0};      // dump: frames to print, 0 = unlimited
  std::int64_t decimate{1};   // dump: print every Nth frame
  double seconds{0.0};        // 0 = until stopped (stats: 10)
};

/** Applies --seconds: true once it has elapsed since start. */
bool timeUp(const Options &options, TimePoint start, TimePoint now) {
  return options.seconds > 0.0 &&
         now - start >= std::chrono::duration<double>(options.seconds);
}

[[noreturn]] void usage(int status) {
  std::fprintf(status == 0 ? stdout : stderr,
               "Usage: ibus_reader <command> [options]\n"
               "\n"
               "Commands:\n"
               "  monitor        live display of all 14 channels (r: reset min/max, q: quit)\n"
               "\n"
               "Options:\n"
               "  --device PATH  serial device (default /dev/serial0)\n"
               "  --stale-ms N   no valid frame for N ms means link lost (default 100)\n"
               "  --stop-bits N  1 or 2 (default 1; see docs/remote-ibus-protocol.md §2)\n"
               "  --baud N       baud rate (default 115200)\n"
               "  --seconds S    stop after S seconds\n"
               "  -h, --help     this text\n");
  std::exit(status);
}

std::int64_t toInt(const std::string &flag, const std::string &text) {
  try {
    std::size_t used = 0;
    const long long value = std::stoll(text, &used);
    if (used == text.size()) {
      return value;
    }
  } catch (const std::exception &) {
  }
  std::fprintf(stderr, "ERROR: %s needs a whole number, got '%s'\n", flag.c_str(), text.c_str());
  usage(2);
}

double toDouble(const std::string &flag, const std::string &text) {
  try {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used == text.size()) {
      return value;
    }
  } catch (const std::exception &) {
  }
  std::fprintf(stderr, "ERROR: %s needs a number, got '%s'\n", flag.c_str(), text.c_str());
  usage(2);
}

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {  // int: matches argc
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ERROR: %s needs a value\n", arg.c_str());
        usage(2);
      }
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") {
      usage(0);
    } else if (arg == "--device") {
      options.serial.device = value();
    } else if (arg == "--stale-ms") {
      options.staleAfter = milliseconds(toInt(arg, value()));
    } else if (arg == "--stop-bits") {
      options.serial.stopBits = static_cast<std::uint8_t>(toInt(arg, value()));
    } else if (arg == "--seconds") {
      options.seconds = toDouble(arg, value());
    } else if (arg == "--baud") {
      options.serial.baud = static_cast<std::uint32_t>(toInt(arg, value()));
    } else if (!arg.empty() && arg[0] != '-' && options.command.empty()) {
      options.command = arg;
    } else {
      std::fprintf(stderr, "ERROR: unexpected argument '%s'\n", arg.c_str());
      usage(2);
    }
  }
  if (options.command.empty()) {
    usage(2);
  }
  if (options.seconds < 0.0) {
    std::fprintf(stderr, "ERROR: --seconds cannot be negative\n");
    usage(2);
  }
  if (options.staleAfter <= milliseconds(0)) {
    std::fprintf(stderr, "ERROR: --stale-ms must be positive\n");
    usage(2);
  }
  return options;
}

// ---------------------------------------------------------------------------
// The receiver: port, parser and link rule, as every command uses them

class Receiver {
  public:
    explicit Receiver(const Options &options)
        : m_port(options.serial), m_link(options.staleAfter) {}

    /**
     * Waits up to timeout for bytes and decodes them.
     * @return Frames decoded
     */
    std::size_t pump(milliseconds timeout, const IBusParser::FrameHandler &onFrame = {}) {
      const std::size_t n = m_port.read(m_buf, timeout);
      if (n > 0) {
        m_sawBytes = true;
      }
      return m_parser.feed(std::span(m_buf.data(), n), Clock::now(), [&](const IBusFrame &f) {
        m_link.onFrame(f.stamp);
        if (onFrame) {
          onFrame(f);
        }
      });
    }

    [[nodiscard]] const IBusStats &getStats() const noexcept { return m_parser.getStats(); }
    void resetStats() noexcept { m_parser.resetStats(); }
    [[nodiscard]] const LinkMonitor &getLink() const noexcept { return m_link; }
    [[nodiscard]] bool sawBytes() const noexcept { return m_sawBytes; }
    [[nodiscard]] const SerialConfig &getConfig() const noexcept { return m_port.getConfig(); }

  private:
    IBusPort m_port;
    IBusParser m_parser;
    LinkMonitor m_link;
    std::array<std::uint8_t, 256> m_buf{};
    bool m_sawBytes{false};
};

/** The likely causes when no valid frame arrives, depending on whether any bytes did. */
void printNoDataHint(const Receiver &rx) {
  const char *device = rx.getConfig().device.c_str();
  if (!rx.sawBytes()) {
    std::fprintf(stderr,
                 "\nNo data at all on %s. Check:\n"
                 "  - the transmitter is on and bound, and its RX output mode is i-BUS\n"
                 "    (FS-i6X: System > RX setup > Output mode)\n"
                 "  - the receiver's SERVO iBUS pin (not SENS) goes to GPIO15/RXD, with a\n"
                 "    common ground\n"
                 "  - the UART is enabled: enable_uart=1 and dtoverlay=disable-bt in\n"
                 "    /boot/firmware/config.txt, then a reboot (target/provision/base.sh)\n"
                 "  - the serial console is off: no console=serial0 in cmdline.txt\n\n",
                 device);
  } else {
    std::fprintf(stderr,
                 "\nBytes are arriving on %s but no valid iBUS frame. Check:\n"
                 "  - the wire is on the SERVO iBUS pin, not SENS (telemetry)\n"
                 "  - the RX output mode is i-BUS, not S.BUS or PPM\n"
                 "  - the line settings (now %u baud, %u stop bit%s)\n\n",
                 device, rx.getConfig().baud, rx.getConfig().stopBits,
                 rx.getConfig().stopBits == 1 ? "" : "s");
  }
}

/** Says so, once, when no frame has arrived kNoDataHintAfter after start. */
class NoDataWatch {
  public:
    void check(const Receiver &rx, TimePoint now) {
      if (!m_hinted && !rx.getLink().hasFrame() && now - m_start > kNoDataHintAfter) {
        printNoDataHint(rx);
        m_hinted = true;
      }
    }

  private:
    TimePoint m_start{Clock::now()};
    bool m_hinted{false};
};

// ---------------------------------------------------------------------------
// The terminal

/**
 * Puts stdin in single-key mode for the lifetime of the object, restoring it
 * on every exit path. Ctrl-C still raises SIGINT (ISIG stays on), which the
 * loops turn into a clean exit, so the destructor always runs.
 */
class KeyMode {
  public:
    KeyMode() {
      if (isatty(STDIN_FILENO) == 0 || tcgetattr(STDIN_FILENO, &m_saved) != 0) {
        return;
      }
      termios raw = m_saved;
      raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      m_active = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }

    ~KeyMode() {
      if (m_active) {
        tcsetattr(STDIN_FILENO, TCSANOW, &m_saved);
      }
    }

    KeyMode(const KeyMode &) = delete;
    KeyMode &operator=(const KeyMode &) = delete;

    /** @return The next key pressed, or 0 if none is waiting */
    [[nodiscard]] char poll() const {
      if (!m_active) {
        return 0;
      }
      pollfd pfd{STDIN_FILENO, POLLIN, 0};
      char c = 0;
      if (::poll(&pfd, 1, 0) > 0 && ::read(STDIN_FILENO, &c, 1) == 1) {
        return c;
      }
      return 0;
    }

  private:
    termios m_saved{};
    bool m_active{false};
};

/** The alternate screen with the cursor hidden, undone on destruction. */
class FullScreen {
  public:
    FullScreen() : m_active(isatty(STDOUT_FILENO) != 0) {
      if (m_active) {
        std::fputs("\033[?1049h\033[?25l\033[2J", stdout);
      }
    }

    ~FullScreen() {
      if (m_active) {
        std::fputs("\033[?25h\033[?1049l", stdout);
        std::fflush(stdout);
      }
    }

    FullScreen(const FullScreen &) = delete;
    FullScreen &operator=(const FullScreen &) = delete;

  private:
    bool m_active;
};

double toMs(Duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

// ---------------------------------------------------------------------------
// monitor

/**
 * A bar from 1000 to 2000, filled from the 1500 midpoint to the value, so
 * sticks and knobs read the same way. Out-of-range values show an arrow.
 */
std::string bar(std::uint16_t raw) {
  constexpr std::int32_t kCells = 41;  // 25 counts per cell, centre cell at 20
  constexpr std::int32_t kMid = kCells / 2;
  std::string out(kCells, ' ');
  out[kMid] = '|';
  const std::int32_t value = channelValue(raw);
  const std::int32_t cell = std::clamp((value - 1000 + 12) / 25, 0, kCells - 1);
  for (std::int32_t i = std::min(cell, kMid); i <= std::max(cell, kMid); ++i) {
    if (i != kMid) {
      out[static_cast<std::size_t>(i)] = '=';
    }
  }
  if (value < 1000) {
    out.front() = '<';
  } else if (value > 2000) {
    out.back() = '>';
  }
  return "[" + out + "]";
}

int runMonitor(const Options &options) {
  Receiver rx(options);
  KeyMode keys;
  FullScreen screen;
  const bool color = isatty(STDOUT_FILENO) != 0;

  IBusFrame latest;
  std::array<std::uint16_t, kIBusChannelCount> lo{};
  std::array<std::uint16_t, kIBusChannelCount> hi{};
  bool haveMinMax = false;
  auto resetMinMax = [&] {
    lo.fill(0xFFFF);
    hi.fill(0);
    haveMinMax = false;
  };
  resetMinMax();

  std::uint64_t staleEvents = 0;
  LinkState previous = LinkState::NoSignal;
  double rateHz = 0.0;
  std::uint64_t framesAtMark = 0;
  const TimePoint start = Clock::now();
  TimePoint mark = start;
  TimePoint nextDraw = mark;

  while (s_stopRequested == 0 && !timeUp(options, start, Clock::now())) {
    rx.pump(kPumpTimeout, [&](const IBusFrame &f) {
      latest = f;
      for (std::size_t i = 0; i < kIBusChannelCount; ++i) {
        lo[i] = std::min(lo[i], f.channels[i]);
        hi[i] = std::max(hi[i], f.channels[i]);
      }
      haveMinMax = true;
    });

    const char key = keys.poll();
    if (key == 'q' || key == 'Q') {
      break;
    }
    if (key == 'r' || key == 'R') {
      resetMinMax();
    }

    const TimePoint now = Clock::now();
    const LinkState state = rx.getLink().getState(now);
    if (state == LinkState::Stale && previous == LinkState::Ok) {
      ++staleEvents;
    }
    previous = state;

    if (now - mark >= std::chrono::seconds(1)) {
      const std::uint64_t frames = rx.getStats().framesDecoded;
      rateHz = static_cast<double>(frames - framesAtMark) /
               std::chrono::duration<double>(now - mark).count();
      framesAtMark = frames;
      mark = now;
    }
    if (now < nextDraw) {
      continue;
    }
    nextDraw = now + milliseconds(100);

    const IBusStats &s = rx.getStats();
    const char *on = "";
    const char *off = "";
    if (color) {
      on = state == LinkState::Ok ? "\033[32m" : "\033[31;1m";
      off = "\033[0m";
    }
    std::printf("\033[H");
    std::printf("ibus_reader monitor   %s   %u baud, %u stop bit(s)   stale after %.0f ms"
                "\033[K\n",
                rx.getConfig().device.c_str(), rx.getConfig().baud, rx.getConfig().stopBits,
                toMs(rx.getLink().getStaleAfter()));
    if (state == LinkState::NoSignal && now - start > kNoDataHintAfter) {
      // The full hint goes to stderr from the line-mode commands; here, a
      // one-line version that fits the screen.
      std::printf("link %sNO SIGNAL%s  %s\033[K\n", on, off,
                  rx.sawBytes() ? "bytes but no valid frame: SERVO pin? i-BUS mode? line settings?"
                                : "no bytes: transmitter on, i-BUS mode, wiring, UART enabled?");
    } else if (state == LinkState::NoSignal) {
      std::printf("link %sNO SIGNAL%s (no valid frame yet)\033[K\n", on, off);
    } else {
      std::printf("link %s%-5s%s  last frame %6.0f ms ago   rate %6.1f Hz   frames %llu\033[K\n",
                  on, toString(state), off, toMs(rx.getLink().sinceLastFrame(now)), rateHz,
                  static_cast<unsigned long long>(s.framesDecoded));
    }
    std::printf("errors: checksum %llu   discarded bytes %llu   resyncs %llu   "
                "stale events %llu\033[K\n",
                static_cast<unsigned long long>(s.checksumErrors),
                static_cast<unsigned long long>(s.discardedBytes),
                static_cast<unsigned long long>(s.resyncs),
                static_cast<unsigned long long>(staleEvents));
    std::printf("\033[K\n CH   raw   1000                 1500                 2000    min   max"
                "\033[K\n");
    for (std::size_t i = 0; i < kIBusChannelCount; ++i) {
      const std::uint16_t raw = latest.channels[i];
      const std::uint8_t nibble = channelHighNibble(raw);
      char flag[8] = "";
      if (nibble != 0) {
        std::snprintf(flag, sizeof flag, " !0x%X", nibble);
      }
      if (rx.getLink().hasFrame()) {
        std::printf(" %2zu  %5u  %s", i + 1, raw, bar(raw).c_str());
      } else {
        std::printf(" %2zu      -", i + 1);
      }
      if (haveMinMax) {
        std::printf("  %5u %5u%s\033[K\n", lo[i], hi[i], flag);
      } else {
        std::printf("      -     -\033[K\n");
      }
    }
    std::printf("\033[K\nr: reset min/max   q or Ctrl-C: quit\033[K\n\033[J");
    std::fflush(stdout);
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  const Options options = parse(argc, argv);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  try {
    if (options.command == "monitor") {
      return runMonitor(options);
    }
    std::fprintf(stderr, "ERROR: unknown command '%s'\n", options.command.c_str());
    usage(2);
  } catch (const std::system_error &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    if (e.code() == std::errc::no_such_file_or_directory) {
      std::fprintf(stderr,
                   "  %s does not exist. On the Pi the GPIO UART needs enable_uart=1 and\n"
                   "  dtoverlay=disable-bt (target/provision/base.sh) and a reboot.\n",
                   options.serial.device.c_str());
    } else if (e.code() == std::errc::permission_denied) {
      std::fprintf(stderr,
                   "  Permission denied: add this user to the dialout group, then log in\n"
                   "  again:   sudo usermod -aG dialout \"$USER\"\n");
    }
    return 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
