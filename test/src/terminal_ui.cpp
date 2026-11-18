// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "terminal_ui.hpp"

#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace qrb::adapter::stream_probe
{

namespace
{

/** @brief Lines kept to fill the scrolling part again after a resize. */
constexpr std::size_t kHistory = 1000;
constexpr std::size_t kMaxInput = 4096;
constexpr const char * kHideCursor = "\x1b[?25l";
constexpr const char * kShowCursor = "\x1b[?25h";
constexpr const char * kClearLine = "\x1b[2K";
// DECSC/DECRC: where the next line of the scroll region goes. Nothing else saves the cursor.
constexpr const char * kSaveCursor =
    "\x1b"
    "7";
constexpr const char * kRestoreCursor =
    "\x1b"
    "8";
constexpr const char * kStdinClosed = "stdin is closed; Ctrl-C quits";

std::atomic<bool> gResized{false};

extern "C" void OnResize(int)
{
  gResized.store(true);
}

/** @brief The first of LC_ALL, LC_CTYPE and LANG that is set names a UTF-8 locale. */
bool Utf8Locale()
{
  for (const char * name : {"LC_ALL", "LC_CTYPE", "LANG"}) {
    const char * value = std::getenv(name);
    if (value == nullptr || *value == '\0')
      continue;
    std::string locale(value);
    for (char & c : locale)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return locale.find("utf-8") != std::string::npos || locale.find("utf8") != std::string::npos;
  }
  return false;
}

bool IsContinuation(char c)
{
  return (static_cast<unsigned char>(c) & 0xC0) == 0x80;
}

/** @brief Characters of UTF-8 `text`, one column each. */
int Columns(const std::string & text)
{
  int columns = 0;
  for (char c : text) {
    if (!IsContinuation(c))
      ++columns;
  }
  return columns;
}

/** @brief `text` cut to `columns` characters. */
std::string Fit(const std::string & text, int columns)
{
  int used = 0;
  std::size_t end = 0;
  for (; end < text.size(); ++end) {
    if (IsContinuation(text[end]))
      continue;
    if (used == columns)
      break;
    ++used;
  }
  return text.substr(0, end);
}

std::string MoveTo(int row, int column)
{
  return "\x1b[" + std::to_string(row) + ";" + std::to_string(column) + "H";
}

void Emit(const std::string & out)
{
  std::size_t done = 0;
  while (done < out.size()) {
    const ssize_t wrote = ::write(STDOUT_FILENO, out.data() + done, out.size() - done);
    if (wrote < 0 && errno == EINTR)
      continue;
    if (wrote <= 0)
      return;
    done += static_cast<std::size_t>(wrote);
  }
}

}  // namespace

TerminalUi::~TerminalUi()
{
  Close();
}

bool TerminalUi::Open(const std::string & commands)
{
  if (::isatty(STDIN_FILENO) == 0 || ::isatty(STDOUT_FILENO) == 0)
    return false;
  const char * term = std::getenv("TERM");
  if (term == nullptr || *term == '\0' || std::strcmp(term, "dumb") == 0)
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (active_.load())
    return true;
  if (!Measure() || rows_ < kMinRows || columns_ < kMinColumns)
    return false;
  if (::tcgetattr(STDIN_FILENO, &saved_) != 0)
    return false;
  termios raw = saved_;
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  // Ctrl-Z and Ctrl-\ would leave the terminal as this screen set it.
  raw.c_cc[VSUSP] = _POSIX_VDISABLE;
  raw.c_cc[VQUIT] = _POSIX_VDISABLE;
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    return false;

  struct sigaction action{};
  action.sa_handler = OnResize;
  sigemptyset(&action.sa_mask);
  ::sigaction(SIGWINCH, &action, &savedResize_);

  rule_ = Utf8Locale() ? "\xe2\x94\x80" : "-";
  commands_ = commands;
  std::fflush(stdout);
  // Keep what is on the screen: scroll up just enough for the panel and the
  // input line, and start the output on the line below the shell's command.
  const int reserved = kPanelRows + kBottomRows;
  std::string out(static_cast<std::size_t>(reserved), '\n');
  out += "\r\x1b[" + std::to_string(reserved) + "A" + kSaveCursor + "\x1b[J" + ScrollRegion();
  scrolled_ = false;
  DrawLocked(out);
  Emit(out);
  active_.store(true);
  return true;
}

void TerminalUi::Close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_.exchange(false))
    return;
  std::string out = "\x1b[r";
  out += MoveTo(rows_, 1) + kClearLine + MoveTo(rows_ - 1, 1) + kClearLine + kShowCursor;
  Emit(out);
  ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
  ::sigaction(SIGWINCH, &savedResize_, nullptr);
}

bool TerminalUi::Print(const std::string & text)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_.load())
    return false;
  std::string out = kHideCursor;
  std::size_t begin = 0;
  while (true) {
    const std::size_t end = text.find('\n', begin);
    AppendLocked(
        text.substr(begin, end == std::string::npos ? std::string::npos : end - begin), out);
    if (end == std::string::npos)
      break;
    begin = end + 1;
  }
  PlaceCursorLocked(out);
  Emit(out);
  return true;
}

void TerminalUi::SetPanel(std::vector<std::string> lines)
{
  std::lock_guard<std::mutex> lock(mutex_);
  panel_ = std::move(lines);
  if (!active_.load())
    return;
  std::string out = kHideCursor;
  DrawLocked(out);
  Emit(out);
}

bool TerminalUi::Poll(int timeoutMs, std::vector<std::string> & lines)
{
  if (gResized.exchange(false)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_.load())
      ResizeLocked();
  }
  // Only this thread changes stdinClosed_.
  const bool closed = stdinClosed_;
  pollfd fd{STDIN_FILENO, POLLIN, 0};
  const int ready = ::poll(&fd, closed ? 0 : 1, timeoutMs);
  if (ready <= 0)
    return !closed;  // timeout or a signal
  char buffer[256];
  const ssize_t got = ::read(STDIN_FILENO, buffer, sizeof(buffer));
  if (got < 0 && (errno == EINTR || errno == EAGAIN))
    return true;

  std::lock_guard<std::mutex> lock(mutex_);
  std::string out = kHideCursor;
  if (got <= 0) {
    KeyLocked('\n', lines, out);
    stdinClosed_ = true;
  } else {
    for (ssize_t i = 0; i < got; ++i)
      KeyLocked(static_cast<unsigned char>(buffer[i]), lines, out);
  }
  if (active_.load()) {
    DrawInputLocked(out);
    Emit(out);
  }
  return !stdinClosed_;
}

bool TerminalUi::Measure()
{
  winsize size{};
  if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) != 0 || size.ws_row == 0 || size.ws_col == 0)
    return false;
  rows_ = size.ws_row;
  columns_ = size.ws_col;
  return true;
}

int TerminalUi::ScrollRows() const
{
  return std::max(1, rows_ - kPanelRows - kBottomRows);
}

std::string TerminalUi::ScrollRegion() const
{
  return "\x1b[1;" + std::to_string(ScrollRows()) + "r";
}

std::string TerminalUi::Ruled(const std::string & text) const
{
  // The last column stays empty: a character there may wrap on some terminals.
  const int width = columns_ - 1;
  std::string line = rule_ + rule_ + " " + text + " ";
  const int used = Columns(line);
  if (used >= width)
    return Fit(line, width);
  for (int i = used; i < width; ++i)
    line += rule_;
  return line;
}

void TerminalUi::AppendLocked(const std::string & line, std::string & out)
{
  history_.push_back(line);
  if (history_.size() > kHistory)
    history_.pop_front();
  // Go back to the end of the last line; a newline on the bottom row of the
  // scroll region scrolls it up by one, and a longer line wraps and scrolls on its own.
  out += kRestoreCursor;
  if (scrolled_)
    out += "\n";
  out += line + kSaveCursor;
  scrolled_ = true;
}

void TerminalUi::DrawLocked(std::string & out) const
{
  const int first = ScrollRows() + 1;
  for (int i = 0; i < kPanelRows; ++i) {
    const std::string text =
        static_cast<std::size_t>(i) < panel_.size() ? panel_[i] : std::string();
    out += MoveTo(first + i, 1) + kClearLine + (i == 0 ? Ruled(text) : Fit(text, columns_ - 1));
  }
  out += MoveTo(rows_ - 1, 1) + kClearLine + Ruled(commands_);
  DrawInputLocked(out);
}

void TerminalUi::DrawInputLocked(std::string & out) const
{
  out += MoveTo(rows_, 1) + kClearLine;
  out += stdinClosed_ ? Fit(kStdinClosed, columns_ - 1) : "> " + ShownInputLocked();
  PlaceCursorLocked(out);
}

void TerminalUi::PlaceCursorLocked(std::string & out) const
{
  if (stdinClosed_) {
    out += MoveTo(rows_, columns_);
    return;
  }
  out += MoveTo(rows_, 3 + Columns(ShownInputLocked())) + kShowCursor;
}

std::string TerminalUi::ShownInputLocked() const
{
  // The end of a line longer than the screen.
  const int width = std::max(1, columns_ - 3);
  std::size_t begin = 0;
  for (int extra = Columns(input_) - width; extra > 0; --extra) {
    do
      ++begin;
    while (begin < input_.size() && IsContinuation(input_[begin]));
  }
  return input_.substr(begin);
}

void TerminalUi::ResizeLocked()
{
  if (!Measure())
    return;
  std::string out = std::string(kHideCursor) + "\x1b[r\x1b[2J" + ScrollRegion() + MoveTo(1, 1);
  const std::size_t keep = std::min(history_.size(), static_cast<std::size_t>(ScrollRows()));
  for (auto line = history_.end() - static_cast<std::ptrdiff_t>(keep); line != history_.end();
      ++line)
    out += (line == history_.end() - static_cast<std::ptrdiff_t>(keep) ? "" : "\n") + *line;
  out += kSaveCursor;
  scrolled_ = keep > 0;
  DrawLocked(out);
  Emit(out);
}

void TerminalUi::KeyLocked(unsigned char key, std::vector<std::string> & lines, std::string & out)
{
  // Arrow keys and the like: ESC [ or ESC O, then up to a final byte.
  switch (escape_) {
    case Escape::kStart:
      escape_ = key == '[' || key == 'O' ? Escape::kSequence : Escape::kNone;
      return;
    case Escape::kSequence:
      if (key >= 0x40 && key <= 0x7e)
        escape_ = Escape::kNone;
      return;
    case Escape::kNone:
      break;
  }
  if (key == 0x1b) {
    escape_ = Escape::kStart;
    return;
  }
  if (key == '\n' || key == '\r') {
    if (input_.empty())
      return;
    lines.push_back(input_);
    AppendLocked("> " + input_, out);
    input_.clear();
    return;
  }
  if (key == 0x7f || key == 0x08)  // Backspace: one character
  {
    while (!input_.empty() && IsContinuation(input_.back()))
      input_.pop_back();
    if (!input_.empty())
      input_.pop_back();
    return;
  }
  if (key == 0x15)  // Ctrl-U
  {
    input_.clear();
    return;
  }
  if (key < 0x20 || input_.size() >= kMaxInput)
    return;
  input_.push_back(static_cast<char>(key));
}

}  // namespace qrb::adapter::stream_probe
