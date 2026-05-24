#include "teensy_motor_io/protocol.hpp"

#include <array>

namespace teensy_motor_io {
namespace {

constexpr std::size_t kHeaderBytes = 4;
constexpr std::size_t kCrcBytes = 2;

void append_u16_le(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

std::uint16_t read_u16_le(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0])
        | static_cast<std::uint16_t>(data[1] << 8);
}

std::vector<std::uint8_t> cobs_encode(const std::vector<std::uint8_t>& input) {
    std::vector<std::uint8_t> out;
    out.reserve(input.size() + 2);

    std::size_t code_index = 0;
    std::uint8_t code = 1;
    out.push_back(0);

    for (std::uint8_t byte : input) {
        if (byte == 0) {
            out[code_index] = code;
            code_index = out.size();
            out.push_back(0);
            code = 1;
            continue;
        }

        out.push_back(byte);
        ++code;
        if (code == 0xFF) {
            out[code_index] = code;
            code_index = out.size();
            out.push_back(0);
            code = 1;
        }
    }

    out[code_index] = code;
    out.push_back(0);
    return out;
}

bool cobs_decode(
    const std::uint8_t* data,
    std::size_t length,
    std::vector<std::uint8_t>& out
) {
    out.clear();
    if (length == 0 || data[length - 1] != 0) {
        return false;
    }

    std::size_t index = 0;
    const std::size_t end = length - 1;
    while (index < end) {
        const std::uint8_t code = data[index++];
        if (code == 0 || index + code - 1 > end) {
            return false;
        }

        for (std::uint8_t i = 1; i < code; ++i) {
            out.push_back(data[index++]);
        }
        if (code < 0xFF && index < end) {
            out.push_back(0);
        }
    }
    return true;
}

}  // namespace

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t length) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i] << 8);
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000) != 0) {
                crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

std::vector<std::uint8_t> encode_frame(const Frame& frame) {
    std::vector<std::uint8_t> payload;
    payload.reserve(kHeaderBytes + frame.body.size() + kCrcBytes);
    payload.push_back(frame.version);
    payload.push_back(static_cast<std::uint8_t>(frame.type));
    append_u16_le(payload, frame.sequence);
    payload.insert(payload.end(), frame.body.begin(), frame.body.end());
    append_u16_le(payload, crc16_ccitt(payload.data(), payload.size()));
    return cobs_encode(payload);
}

DecodeResult decode_frame(const std::uint8_t* data, std::size_t length) {
    std::vector<std::uint8_t> payload;
    if (!cobs_decode(data, length, payload)) {
        return {false, "invalid COBS frame", {}};
    }
    if (payload.size() < kHeaderBytes + kCrcBytes) {
        return {false, "frame too short", {}};
    }
    if (payload.size() > kHeaderBytes + kMaxFramePayloadBytes + kCrcBytes) {
        return {false, "frame too large", {}};
    }

    const std::size_t crc_offset = payload.size() - kCrcBytes;
    const std::uint16_t expected_crc = read_u16_le(payload.data() + crc_offset);
    const std::uint16_t actual_crc = crc16_ccitt(payload.data(), crc_offset);
    if (expected_crc != actual_crc) {
        return {false, "crc mismatch", {}};
    }
    if (payload[0] != kProtocolVersion) {
        return {false, "unsupported protocol version", {}};
    }

    Frame frame;
    frame.version = payload[0];
    frame.type = static_cast<MessageType>(payload[1]);
    frame.sequence = read_u16_le(payload.data() + 2);
    frame.body.assign(payload.begin() + kHeaderBytes, payload.begin() + crc_offset);
    return {true, nullptr, frame};
}

}  // namespace teensy_motor_io
