#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace teensy_motor_io {

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kMaxFramePayloadBytes = 96;

enum class MessageType : std::uint8_t {
    kHeartbeat = 1,
    kDriveCommand = 2,
    kControlFlags = 3,
    kResetFaults = 4,
    kStatus = 0x80,
    kDrivetrainTelemetry = 0x81,
    kFaultEvent = 0x82,
};

struct Frame {
    std::uint8_t version{kProtocolVersion};
    MessageType type{MessageType::kHeartbeat};
    std::uint16_t sequence{0};
    std::vector<std::uint8_t> body{};
};

struct DecodeResult {
    bool ok{false};
    const char* error{nullptr};
    Frame frame{};
};

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t length);
std::vector<std::uint8_t> encode_frame(const Frame& frame);
DecodeResult decode_frame(const std::uint8_t* data, std::size_t length);

}  // namespace teensy_motor_io
