#pragma once

// Data-logging facade for last_letter_lib.
//
// This header is intentionally free of any DataTamer / MCAP includes and uses
// only C++11-clean types in its signatures. All backend (DataTamer, C++17)
// code is confined to src/logging/logging.cpp. This keeps the header safe to
// include from a C++11 translation unit (e.g. an ArduPilot-facing facade).
//
// Logging is a process-global concern, off by default. Enabling attaches an
// MCAP sink to DataTamer's global channel registry; every Component / UavModel
// then self-logs into that sink from inside its calc_model()/step(). When
// logging is disabled, the self-log is a cheap no-op.

#include <string>

namespace last_letter_lib
{
namespace logging
{

// Start a new recording, writing to `path` (should end in ".mcap").
// Clears any previous recording and bumps the epoch (see epoch()).
// Call this BEFORE the first calc_model()/step() so channels are created with
// the sink already attached.
void enable(const std::string &path);

// Stop the current recording, flushing and closing the file.
void disable();

// True while a recording is active.
bool is_enabled();

// Set the timestamp (in seconds) applied to every subsequent snapshot until
// changed. The full model calls this once per step(); standalone drivers may
// call it too. If never called, snapshots use an auto-incrementing sample
// index instead (sufficient for a single-component recording).
void set_time(double t_seconds);

// Current snapshot timestamp in nanoseconds. Used internally by the loggers.
// Returns the value set by set_time(), or an auto-incrementing counter.
long long now_ns();

// Monotonically increasing recording id, bumped by each enable(). Loggers use
// it to know when to (re)register their channels for a fresh recording.
unsigned epoch();

} // namespace logging
} // namespace last_letter_lib
