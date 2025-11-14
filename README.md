# usb_serial_transport

`usb_serial_transport` is a convenience wrapper for sending and receiving nanopb-generated
messages over a USB serial port using **standalone Asio**. It hides the moving pieces required
to frame payloads (COBS + 0x00 delimiter), encode/decode nanopb structs, and drive Asio’s
asynchronous APIs behind a simple, blocking-style interface. If you need full control over
framing, encoding, or non-blocking I/O, use the underlying components directly (nanopb,
COBS, and Asio’s primitives).

> **Namespace note**
>
> All symbols (the transport class, error enum, and helper macros) live in the
> `usb_serial` namespace. The CMake target is still named
> `usb_serial_transport::usb_serial_transport` for familiarity with other ROS
> packages, but the headers intentionally keep the shorter namespace.

---

## Getting Started

### 1. Add the dependency and include headers in your CMakeLists.txt

```cmake
find_package(usb_serial_transport REQUIRED)
target_link_libraries(your_target PRIVATE usb_serial_transport::usb_serial_transport)
```

Install prerequisites (including Asio) with rosdep:

```bash
rosdep install --from-paths src --ignore-src -r
```

> **Build requirements**
>
> - `asio_cmake_module` **and** `asio`. Running `rosdep install --from-paths src
>   --ignore-src -r` will pull them in automatically; they ship the
>   `FindASIO.cmake` module and the imported `asio::asio` target that this
>   library links against.
> - `ASIO_STANDALONE` define. The exported interface target already adds this
>   definition, so consumers typically do not need to set it manually.

---

### 2. Define your protocol and generate nanopb code

Create a `.proto` file describing your messages:

```proto
syntax = "proto3";

message SensorReading {
  uint32 id = 1;
  int64 timestamp_ms = 2;
  double value = 3;
}

message ControlCommand {
  string command_name = 1;
  bytes args = 2;
}

message Envelope {
  oneof payload {
    SensorReading sensor_reading = 1;
    ControlCommand control_command = 2;
  }
}
```

Then run the nanopb generator to produce `.c`/`.h` sources.  
**Do not redefine or modify schemas that the device firmware already uses.**  
Always reuse the device’s nanopb-generated files to stay wire-compatible.

Add the generated sources to your build.

---

### 3. Register nanopb descriptors

Register each message type exactly once (usually at file scope):

```cpp
USB_SERIAL_REGISTER_NANOPB_MESSAGE(SensorReading);
USB_SERIAL_REGISTER_NANOPB_MESSAGE(ControlCommand);
USB_SERIAL_REGISTER_NANOPB_MESSAGE(Envelope);
```

This allows the transport to automatically encode and decode your structs.

---

### 4. Create the transport and send/receive messages (with timeout)

```cpp
#include <usb_serial_transport/serial_transport.hpp>
#include <chrono>
#include <iostream>
#include "your_proto.pb.h"

using namespace std::chrono_literals;

// Register once per nanopb struct you plan to send/receive.
USB_SERIAL_REGISTER_NANOPB_MESSAGE(SensorReading);
USB_SERIAL_REGISTER_NANOPB_MESSAGE(ControlCommand);
USB_SERIAL_REGISTER_NANOPB_MESSAGE(Envelope);

int main() {
  usb_serial::SerialTransport serial;

  // Explicitly open the port (configure baud, parity, etc.).
  if (auto err = serial.open("/dev/ttyUSB0", 115200); err != usb_serial::Error::None) {
    std::cerr << "Failed to open port: " << usb_serial::error_to_string(err) << "\n";
    return 1;
  }

  // Build a message
  Envelope msg = Envelope_init_zero;
  msg.payload.which_payload = Envelope_sensor_reading_tag;
  msg.payload.sensor_reading.id = 1;
  msg.payload.sensor_reading.timestamp_ms = 1625079600000;
  msg.payload.sensor_reading.value = 42.0;

  // Send message (COBS framed + 0x00 delimiter)
  auto write_result = serial.write_message(msg);
  if (!write_result) {
    std::cerr << "Send failed: "
              << usb_serial::error_to_string(write_result.error()) << "\n";
    return 1;
  }

  // Receive one message (500 ms deadline)
  auto read_result = serial.read_message<Envelope>(500ms);
  if (read_result) {
    const Envelope &received = read_result.value();
    // Handle received message
  } else {
    std::cerr << "Receive failed: "
              << usb_serial::error_to_string(read_result.error()) << "\n";
  }

  return 0;
}
```

---

## Error Handling

All operations return `tl::expected` types:

- `write_message` → `tl::expected<void, Error>`
- `read_message(timeout)` → `tl::expected<Msg, Error>`

Use standard `expected` methods (`has_value()`, `operator bool()`,
`value()`, `error()`) and map errors to text with `error_to_string(Error)`.

### Common Errors
| Error | Meaning |
|-------|---------|
| `PortOpenFailed` | Device path invalid or Asio failed to configure the port |
| `Timeout` | No complete framed message before the deadline |
| `OversizeFrame` | Delimiter found but frame exceeded configured max size |
| `CobsDecodeFailed` | Frame payload was not valid COBS data |
| `DecodeFailed` | nanopb could not decode (schema mismatch / corrupt wire) |

---

## License

MIT License.
