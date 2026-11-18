// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_STREAM_PROBE_TERMINAL_UI_HPP
#define QRB_ROS_ADAPTER_STREAM_PROBE_TERMINAL_UI_HPP

//
// The screen of qrb_ros_adapter_stream_probe on a terminal: command results
// and events scroll in the upper part, a status panel redrawn in place sits
// below them, then the command list and the input line. VT100 sequences only
// (a scroll region), no curses. Off unless stdin and stdout are both
// terminals of at least kMinRows x kMinColumns; the host then prints plainly.
//

#include <signal.h>
#include <termios.h>

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace qrb::adapter::stream_probe
{

class TerminalUi
{
public:
  /** @brief A title line, then the status lines. */
  static constexpr int kPanelRows = 6;
  /** @brief The command list and the input line. */
  static constexpr int kBottomRows = 2;
  static constexpr int kMinScrollRows = 10;
  static constexpr int kMinRows = kPanelRows + kBottomRows + kMinScrollRows;
  static constexpr int kMinColumns = 60;

  TerminalUi() = default;
  ~TerminalUi();
  TerminalUi(const TerminalUi &) = delete;
  TerminalUi & operator=(const TerminalUi &) = delete;

  /**
   * @brief Take over the terminal: keys are read one by one without echo,
   * Ctrl-C still raises SIGINT. `commands` is shown above the input line.
   * @return whether the terminal is taken
   */
  bool Open(const std::string & commands);

  /** @brief Give the terminal back; the panel stays and later output goes below it. */
  void Close();

  bool Active() const { return active_.load(); }

  /** @brief Append `text`, one or more lines, to the scrolling part. @return false when not active
   */
  bool Print(const std::string & text);

  /** @brief Replace the panel: a title, then up to kPanelRows - 1 lines. */
  void SetPanel(std::vector<std::string> lines);

  /**
   * @brief Read keys for at most `timeoutMs`; a line finished with Enter
   * goes to `lines` and is echoed to the scrolling part.
   * @return false once stdin is closed
   */
  bool Poll(int timeoutMs, std::vector<std::string> & lines);

private:
  enum class Escape
  {
    kNone,
    kStart,
    kSequence,
  };

  bool Measure();
  int ScrollRows() const;
  std::string ScrollRegion() const;
  std::string Ruled(const std::string & text) const;
  void AppendLocked(const std::string & line, std::string & out);
  void DrawLocked(std::string & out) const;
  void DrawInputLocked(std::string & out) const;
  void PlaceCursorLocked(std::string & out) const;
  std::string ShownInputLocked() const;
  void ResizeLocked();
  void KeyLocked(unsigned char key, std::vector<std::string> & lines, std::string & out);

  mutable std::mutex mutex_;
  std::atomic<bool> active_{false};
  termios saved_{};
  struct sigaction savedResize_{};
  int rows_{0};
  int columns_{0};
  std::string rule_;
  std::string commands_;
  std::vector<std::string> panel_;
  std::deque<std::string> history_;
  /** @brief The scroll region has a line in it; the saved cursor (DECSC) is at its end. */
  bool scrolled_{false};
  std::string input_;
  Escape escape_{Escape::kNone};
  bool stdinClosed_{false};
};

}  // namespace qrb::adapter::stream_probe

#endif  // QRB_ROS_ADAPTER_STREAM_PROBE_TERMINAL_UI_HPP
