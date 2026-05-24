#include "teensy_motor_io/protocol.hpp"

#include <cassert>
#include <cstdint>

using teensy_motor_io::Frame;
using teensy_motor_io::MessageType;

int main() {
    Frame frame;
    frame.type = MessageType::kDriveCommand;
    frame.sequence = 42;
    frame.body = {0x01, 0x00, 0x7F, 0x00, 0x80};

    const auto encoded = teensy_motor_io::encode_frame(frame);
    assert(!encoded.empty());
    assert(encoded.back() == 0);

    const auto decoded = teensy_motor_io::decode_frame(encoded.data(), encoded.size());
    assert(decoded.ok);
    assert(decoded.frame.version == teensy_motor_io::kProtocolVersion);
    assert(decoded.frame.type == MessageType::kDriveCommand);
    assert(decoded.frame.sequence == 42);
    assert(decoded.frame.body == frame.body);

    auto corrupted = encoded;
    corrupted[2] ^= 0x55;
    const auto bad = teensy_motor_io::decode_frame(corrupted.data(), corrupted.size());
    assert(!bad.ok);

    return 0;
}
