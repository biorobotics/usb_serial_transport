#pragma once
/**
 * @file serial_transport.hpp
 * @brief Header-only USB serial transport for nanopb messages wrapped in COBS
 *        framing and a 0x00 delimiter, implemented with standalone Asio.
 *
 * The transport exposes blocking-style helpers that hide the details of
 * nanopb encoding/decoding, byte stuffing via COBS, and the asynchronous APIs
 * provided by Asio. Projects can therefore queue protobuf-sized structs for
 * transmission over a USB serial link while recovering well-typed errors when
 * anything goes wrong.
 */

#include <asio.hpp>         // build with -DASIO_STANDALONE
#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>

// nanopb
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <tl_expected/expected.hpp>

// COBS (packaged as cobs_c)
extern "C" {
  #include "cobs.h"
}

namespace usb_serial {

//------------------------------------------------------------------------------
// Nanopb descriptor binding helper (macro-based)
//------------------------------------------------------------------------------
/**
 * nanopb requires a `pb_msgdesc_t` descriptor per message type. Rather than
 * asking callers to pass descriptors around manually, we let them register
 * each message once with a macro and then look it up through this template.
 */
template <typename Msg>
struct pb_fields_of {
  static constexpr const pb_msgdesc_t* value = nullptr;
};

#define USB_SERIAL_REGISTER_NANOPB_MESSAGE(MSGTYPE)                     \
  namespace usb_serial {                                               \
  template <> struct pb_fields_of<MSGTYPE> {                           \
    static constexpr const pb_msgdesc_t* value = MSGTYPE##_fields;     \
  };                                                                   \
  }

/**
 * Retrieve the nanopb descriptor associated with a message type. A nullptr
 * return value indicates the message was never registered, which we surface
 * as a runtime error.
 */
template <typename Msg>
const pb_msgdesc_t* descriptor_for() {
  return pb_fields_of<Msg>::value;
}

//------------------------------------------------------------------------------
// Configuration constants
//------------------------------------------------------------------------------
namespace config {
#ifndef PBWIRE_MAX_PROTO_BYTES
inline constexpr std::size_t max_proto_bytes = 512;
#else
inline constexpr std::size_t max_proto_bytes = static_cast<std::size_t>(PBWIRE_MAX_PROTO_BYTES);
#endif

inline constexpr std::size_t framing_overhead(std::size_t n) noexcept { return 1 + (n / 254); }
inline constexpr std::size_t max_framed_bytes = max_proto_bytes + framing_overhead(max_proto_bytes);
inline constexpr std::uint8_t delimiter_byte = 0x00;
// Buffer a tiny amount of extra time so the Asio event loop can finish
// delivering callbacks before we treat a read as timed out.
inline constexpr auto io_processing_margin = std::chrono::milliseconds(5);
} // namespace config

//------------------------------------------------------------------------------
// Error codes
//------------------------------------------------------------------------------
enum class Error : std::uint8_t {
  None = 0,
  PortOpenFailed,
  EncodeFailed,
  CobsEncodeFailed,
  WriteFailed,
  NotOpen,
  Timeout,
  ReadFailed,
  OversizeFrame,
  CobsDecodeFailed,
  DecodeFailed,
  DescriptorNotRegistered
};

inline std::string_view error_to_string(Error e) noexcept {
  switch (e) {
    case Error::None:             return "None";
    case Error::PortOpenFailed:   return "PortOpenFailed";
    case Error::EncodeFailed:     return "EncodeFailed";
    case Error::CobsEncodeFailed: return "CobsEncodeFailed";
    case Error::WriteFailed:      return "WriteFailed";
    case Error::NotOpen:          return "NotOpen";
    case Error::Timeout:          return "Timeout";
    case Error::ReadFailed:       return "ReadFailed";
    case Error::OversizeFrame:    return "OversizeFrame";
    case Error::CobsDecodeFailed: return "CobsDecodeFailed";
    case Error::DecodeFailed:     return "DecodeFailed";
  }
  return "Unknown";
}

//------------------------------------------------------------------------------
// SerialTransport
//------------------------------------------------------------------------------
class SerialTransport {
public:
  /**
   * Construct a transport with its own Asio context, serial port, and timer.
   * Keeping these as members avoids forcing users to stand up an external
   * `io_context` when all they need is synchronous-style reads and writes.
   */
  SerialTransport()
  // `port_` and `timer_` are *bound* to the same `io_context`. Any async
  // operation we start on them will be driven by calls to `io_.run*()`.
  : port_(io_), timer_(io_) {}

  /**
   * @brief Open the given serial device and configure standard UART settings.
   *
   * The call configures 8 data bits, no parity, one stop bit, and no flow
   * control. Any failure during setup returns a descriptive `Error` enum that
   * callers can convert to text via `error_to_string`.
   */
  Error open(const std::string& port_path, unsigned baudrate = 115200) {
  std::error_code ec;
  // Open the underlying OS serial device. This is a *synchronous* call; it
  // either succeeds immediately or returns an error code.
  port_.open(port_path, ec);
    if (ec) {
      return Error::PortOpenFailed;
    }

  using asio::serial_port_base;
  // Configure basic UART settings (8N1, no flow control). All of these are
  // synchronous option setters on the underlying file descriptor.
    port_.set_option(serial_port_base::baud_rate(baudrate), ec);
    if (ec) return Error::PortOpenFailed;
    port_.set_option(serial_port_base::character_size(8), ec);
    if (ec) return Error::PortOpenFailed;
    port_.set_option(serial_port_base::parity(serial_port_base::parity::none), ec);
    if (ec) return Error::PortOpenFailed;
    port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one), ec);
    if (ec) return Error::PortOpenFailed;
    port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none), ec);
    if (ec) return Error::PortOpenFailed;
    return Error::None;
  }

  /**
   * @brief Cancel in-flight operations and close the device, ignoring errors.
   */
  void close() {
    if (port_.is_open()) {
  std::error_code ignored;
  // Cancel any in-flight async operations so their handlers finish soon.
  port_.cancel(ignored);
  // Then close the OS handle. Both ignore errors because we're shutting down.
  port_.close(ignored);
    }
  }

  bool is_open() const noexcept { return port_.is_open(); }

  /**
   * @brief Encode a nanopb message, wrap it with COBS, and write it to the
   *        serial port with a trailing delimiter.
   *
   * The implementation is synchronous from the caller's perspective. Under the
   * hood we: (1) serialize the struct into a temporary buffer using nanopb,
   * (2) feed that byte stream through a COBS encoder to eliminate embedded
   * zero bytes, and (3) append a single 0x00 delimiter before writing the whole
   * frame. Any issue along the way returns an `Error` that explains the stage
   * that failed (encoding, framing, or the actual write).
   */
  template <typename Msg>
  tl::expected<void, Error> write_message(const Msg& msg) {
    static_assert(std::is_trivially_copyable_v<Msg>,
                  "Nanopb message types should be trivially copyable structs.");
    if (!port_.is_open()) {
      return tl::unexpected(Error::NotOpen);
    }

    const pb_msgdesc_t* descriptor = descriptor_for<Msg>();
    if (!descriptor) {
      return tl::unexpected(Error::DescriptorNotRegistered);
    }

    pb_ostream_t os = pb_ostream_from_buffer(proto_buf_.data(), proto_buf_.size());
    if (!pb_encode(&os, descriptor, &msg)) {
      return tl::unexpected(Error::EncodeFailed);
    }
    const std::size_t proto_len = os.bytes_written;

    cobs_encode_result enc = cobs_encode(
        framed_buf_.data(), framed_buf_.size() - 1,
        proto_buf_.data(),  proto_len);
    if (enc.status != COBS_ENCODE_OK) {
      return tl::unexpected(Error::CobsEncodeFailed);
    }

  std::error_code ec;
    framed_buf_[enc.out_len] = config::delimiter_byte;
  // Synchronous write: this blocks the calling thread until all bytes are
  // handed to the OS (or an error occurs). We do *not* use async I/O here
  // because callers expect `write_message` to return only once the frame is
  // fully queued for transmission.
  asio::write(port_, asio::buffer(framed_buf_.data(), enc.out_len + 1), ec);
    if (ec) {
      return tl::unexpected(Error::WriteFailed);
    }
    return {};
  }

  /**
   * @brief Read a single COBS-framed nanopb message, respecting a timeout.
   *
   * Although Asio is fundamentally asynchronous, this helper behaves like a
   * blocking read: it kicks off an async `read_until` that stops at the COBS
   * delimiter byte, drives the `io_context` for the specified duration, and
   * then either returns the decoded message or an error. Timeouts arise when
   * the delimiter never arrives before the deadline, at which point we cancel
   * the outstanding read.
   */
  template <typename Msg>
  tl::expected<Msg, Error> read_message(std::chrono::milliseconds timeout) {
    static_assert(std::is_trivially_copyable_v<Msg>,
                  "Nanopb message types should be trivially copyable structs.");

    if (!port_.is_open()) {
      return tl::unexpected(Error::NotOpen);
    }

    const pb_msgdesc_t* descriptor = descriptor_for<Msg>();
    if (!descriptor) {
      return tl::unexpected(Error::DescriptorNotRegistered);
    }

    // Start a timer that enforces the overall read timeout.
    //
    // Conceptually:
    //   1. We arm `timer_` to expire after `timeout`.
    //   2. If it actually expires (handler runs with `e == 0`), we call
    //      `port_.cancel`, which forces any pending async read on `port_` to
    //      complete early with `asio::error::operation_aborted`.
    //   3. Later, we translate that specific error into `Error::Timeout`.
    timer_.expires_after(timeout);
    timer_.async_wait([this](const std::error_code& e) {
      if (!e) { // only act on real expiry; ignore cancellation of the timer itself
        std::error_code ignored;
        port_.cancel(ignored);
      }
    });

    std::error_code read_ec{};
    std::size_t bytes = 0;
    bool done = false;

  // Kick off an asynchronous read that completes once we see the delimiter
  // byte (0x00). `async_read_until` internally accumulates bytes into
  // `inbuf_` until the predicate matches. When that happens *or* an error
  // occurs, the lambda below is invoked by the `io_context`.
  //
  // Important: the lambda does *not* run immediately here. It will run later
  // while `io_.run_for(...)` is executing.
    asio::async_read_until(port_, inbuf_, static_cast<char>(config::delimiter_byte),
      [&](const std::error_code& e, std::size_t n) {
    // Capture the outcome of the read:
    //   * `e` is success or an error code (including operation_aborted
    //      if the timer cancelled the port).
    //   * `n` is the number of bytes Asio reports up to and including
    //      the delimiter.
    read_ec = e;
    bytes = n;
    done = true;  // signal to the outer scope that the read finished

    // We no longer need the timeout timer because the read completed.
    std::error_code ignored;
    timer_.cancel(ignored);
      });

  // At this point we have *scheduled* a timer and an async read, but nothing
  // has actually happened yet from the caller's perspective. To let Asio
  // drive these operations, we "turn the crank" on the shared `io_context`.
  //
  // `restart()` clears the internal "stopped" flag that Asio sets once a
  // previous `run`/`run_for` has exhausted all work. Without this call,
  // `run_for` would simply return immediately and our handlers would never
  // fire.
  io_.restart();

  // `run_for` then processes events for *up to* the requested duration. It
  // will:
  //   * block waiting for either the timer to expire or bytes to arrive, and
  //   * invoke the corresponding handlers, and
  //   * return early if there is no more immediate work.
  //
  // The small `io_processing_margin` gives the event loop a little extra breathing
  // room to notice the timer expiring and propagate the resulting
  // `operation_aborted` to the read handler.
  io_.run_for(timeout + config::io_processing_margin);

    // At this point either the read handler ran (setting `done = true`) or it
    // did not. If it didn't, we treat that as a timeout and explicitly cancel
    // any outstanding I/O on the port.
    if (!done) {
      std::error_code ignored;
      port_.cancel(ignored);
      return tl::unexpected(Error::Timeout);
    }

    // If the handler did run, `read_ec` tells us whether it succeeded. A
    // cancelled read (caused by the timer) comes through as
    // `asio::error::operation_aborted`, which we map to `Error::Timeout`. Any
    // other error is treated as a generic read failure.
    if (read_ec) {
      return tl::unexpected((read_ec == asio::error::operation_aborted) ? Error::Timeout
                                                                        : Error::ReadFailed);
    }

    if (bytes == 0) {
      return tl::unexpected(Error::ReadFailed);
    }
    if (bytes > config::max_framed_bytes + 1) {
      inbuf_.consume(bytes);
      return tl::unexpected(Error::OversizeFrame);
    }
    
    asio::buffer_copy(asio::buffer(framed_buf_.data(), bytes), inbuf_.data(), bytes);
    inbuf_.consume(bytes);

    if (framed_buf_[bytes - 1] != config::delimiter_byte) {
      return tl::unexpected(Error::ReadFailed);
    }
    const std::size_t framed_len = bytes - 1;
    if (framed_len > config::max_framed_bytes) {
      return tl::unexpected(Error::OversizeFrame);
    }

  cobs_decode_result dec = cobs_decode(
    proto_buf_.data(),  proto_buf_.size(),
    framed_buf_.data(), framed_len);
    if (dec.status != COBS_DECODE_OK) {
      return tl::unexpected(Error::CobsDecodeFailed);
    }

    Msg out{};
    pb_istream_t is = pb_istream_from_buffer(proto_buf_.data(), dec.out_len);
    if (!pb_decode(&is, descriptor, &out)) {
      return tl::unexpected(Error::DecodeFailed);
    }

    return out;
  }

private:
  asio::io_context    io_;
  asio::serial_port   port_;
  asio::steady_timer  timer_;
  asio::streambuf     inbuf_;

  std::array<std::uint8_t, config::max_proto_bytes>        proto_buf_{};
  std::array<std::uint8_t, config::max_framed_bytes + 1>   framed_buf_{};
};

} // namespace usb_serial
