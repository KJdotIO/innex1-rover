// INNEX-1 Teensy 4.1 drivetrain serial firmware.
//
// Jetson/Mac USB serial commands, 115200 baud:
//   V <left> <right>   Set side throttle targets, -127..127.
//   X                  Immediate hard stop, keeps motion allowed.
//   E                  Simulated/physical E-stop active: hard stop + latch inhibit.
//   U                  E-stop released, restart still required.
//   R                  Restart/reset inhibit, only accepted when E-stop is released.
//   Z                  Reset all encoder counters.
//   H                  Print command help.
//
// Telemetry, 10 Hz:
//   T <ms> <state> <estop> <inhibit> <targetL> <targetR> <cmdL> <cmdR> <FL> <RL> <FR> <RR>
//
// Wiring:
//   Left Sabertooth S1  <- Teensy pin 1  (Serial1 TX)
//   Right Sabertooth S1 <- Teensy pin 8  (Serial2 TX)
//   Sabertooth 0V       <-> Teensy GND
//   Encoders red/black  -> Teensy 3.3V/GND
//   Encoder A/B pins: FL 15/16, RL 17/18, FR 19/20, RR 21/22

constexpr uint8_t ENCODER_COUNT = 4;
constexpr uint8_t ENC_A[ENCODER_COUNT] = {15, 17, 19, 21};
constexpr uint8_t ENC_B[ENCODER_COUNT] = {16, 18, 20, 22};

constexpr uint8_t LEFT_ADDRESS = 128;
constexpr uint8_t RIGHT_ADDRESS = 128;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t SABER_BAUD = 9600;
constexpr uint16_t TELEMETRY_PERIOD_MS = 100;
constexpr uint16_t RAMP_PERIOD_MS = 50;
constexpr int RAMP_STEP = 4;
constexpr uint16_t COMMAND_TIMEOUT_MS = 500;

enum MotionState : uint8_t {
  STATE_READY = 0,
  STATE_DRIVING = 1,
  STATE_INHIBITED = 2,
  STATE_ESTOP = 3,
  STATE_TIMEOUT = 4,
};

volatile int32_t encoder_ticks[ENCODER_COUNT] = {0, 0, 0, 0};
volatile uint8_t last_encoder_state[ENCODER_COUNT] = {0, 0, 0, 0};

int target_left = 0;
int target_right = 0;
int command_left = 0;
int command_right = 0;
bool estop_active = false;
bool motion_inhibited = false;
bool timed_out = false;
uint32_t last_velocity_command_ms = 0;
uint32_t last_ramp_ms = 0;
uint32_t last_telemetry_ms = 0;
char line_buffer[80];
uint8_t line_length = 0;

void updateEncoder(uint8_t index) {
  const uint8_t a = digitalReadFast(ENC_A[index]);
  const uint8_t b = digitalReadFast(ENC_B[index]);
  const uint8_t state = (a << 1) | b;
  const uint8_t transition = (last_encoder_state[index] << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      ++encoder_ticks[index];
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      --encoder_ticks[index];
      break;
    default:
      break;
  }

  last_encoder_state[index] = state;
}

void updateEncoderFL() { updateEncoder(0); }
void updateEncoderRL() { updateEncoder(1); }
void updateEncoderFR() { updateEncoder(2); }
void updateEncoderRR() { updateEncoder(3); }

void saberSend(HardwareSerial &port, uint8_t address, uint8_t command, uint8_t value) {
  const uint8_t checksum = (address + command + value) & 0x7F;
  port.write(address);
  port.write(command);
  port.write(value);
  port.write(checksum);
}

void saberMotor(HardwareSerial &port, uint8_t address, bool motor2, int speed) {
  speed = constrain(speed, -127, 127);
  const uint8_t forward_cmd = motor2 ? 4 : 0;
  const uint8_t reverse_cmd = motor2 ? 5 : 1;
  if (speed >= 0) {
    saberSend(port, address, forward_cmd, static_cast<uint8_t>(speed));
  } else {
    saberSend(port, address, reverse_cmd, static_cast<uint8_t>(-speed));
  }
}

void writeMotorOutputs() {
  saberMotor(Serial1, LEFT_ADDRESS, false, command_left);
  saberMotor(Serial1, LEFT_ADDRESS, true, command_left);
  saberMotor(Serial2, RIGHT_ADDRESS, false, command_right);
  saberMotor(Serial2, RIGHT_ADDRESS, true, command_right);
  Serial1.flush();
  Serial2.flush();
}

void hardStop() {
  target_left = 0;
  target_right = 0;
  command_left = 0;
  command_right = 0;
  writeMotorOutputs();
}

void resetEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    encoder_ticks[i] = 0;
  }
  interrupts();
}

int stepTowards(int current, int target) {
  const int delta = target - current;
  if (abs(delta) <= RAMP_STEP) {
    return target;
  }
  return current + (delta > 0 ? RAMP_STEP : -RAMP_STEP);
}

void updateRamp(uint32_t now_ms) {
  if (now_ms - last_ramp_ms < RAMP_PERIOD_MS) {
    return;
  }
  last_ramp_ms = now_ms;

  if (estop_active || motion_inhibited) {
    if (command_left != 0 || command_right != 0 || target_left != 0 || target_right != 0) {
      hardStop();
    }
    return;
  }

  const int next_left = stepTowards(command_left, target_left);
  const int next_right = stepTowards(command_right, target_right);
  if (next_left != command_left || next_right != command_right) {
    command_left = next_left;
    command_right = next_right;
    writeMotorOutputs();
  }
}

MotionState currentState() {
  if (estop_active) {
    return STATE_ESTOP;
  }
  if (motion_inhibited) {
    return STATE_INHIBITED;
  }
  if (timed_out) {
    return STATE_TIMEOUT;
  }
  if (target_left != 0 || target_right != 0 || command_left != 0 || command_right != 0) {
    return STATE_DRIVING;
  }
  return STATE_READY;
}

void publishTelemetry(uint32_t now_ms) {
  if (now_ms - last_telemetry_ms < TELEMETRY_PERIOD_MS) {
    return;
  }
  last_telemetry_ms = now_ms;

  int32_t ticks[ENCODER_COUNT];
  noInterrupts();
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    ticks[i] = encoder_ticks[i];
  }
  interrupts();

  Serial.print("T ");
  Serial.print(now_ms);
  Serial.print(' ');
  Serial.print(static_cast<int>(currentState()));
  Serial.print(' ');
  Serial.print(estop_active ? 1 : 0);
  Serial.print(' ');
  Serial.print(motion_inhibited ? 1 : 0);
  Serial.print(' ');
  Serial.print(target_left);
  Serial.print(' ');
  Serial.print(target_right);
  Serial.print(' ');
  Serial.print(command_left);
  Serial.print(' ');
  Serial.print(command_right);
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    Serial.print(' ');
    Serial.print(ticks[i]);
  }
  Serial.println();
}

void printHelp() {
  Serial.println("OK H V <left> <right> | X | E | U | R | Z");
}

void engageEstop() {
  estop_active = true;
  motion_inhibited = true;
  timed_out = false;
  hardStop();
  Serial.println("OK E estop_active motion_inhibited");
}

void releaseEstop() {
  estop_active = false;
  Serial.println("OK U estop_released_restart_required");
}

void restartMotion() {
  if (estop_active) {
    Serial.println("ERR R estop_active");
    return;
  }
  motion_inhibited = false;
  timed_out = false;
  last_velocity_command_ms = millis();
  hardStop();
  Serial.println("OK R motion_allowed");
}

void setVelocityTargets(char *args) {
  if (estop_active || motion_inhibited) {
    hardStop();
    Serial.println("ERR V motion_inhibited");
    return;
  }

  char *left_text = strtok(args, " ");
  char *right_text = strtok(nullptr, " ");
  if (left_text == nullptr || right_text == nullptr) {
    Serial.println("ERR V expected_left_right");
    return;
  }

  target_left = constrain(atoi(left_text), -127, 127);
  target_right = constrain(atoi(right_text), -127, 127);
  last_velocity_command_ms = millis();
  timed_out = false;
  Serial.print("OK V ");
  Serial.print(target_left);
  Serial.print(' ');
  Serial.println(target_right);
}

void handleLine(char *line) {
  while (*line == ' ') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  char command = *line++;
  while (*line == ' ') {
    ++line;
  }

  switch (command) {
    case 'V':
    case 'v':
      setVelocityTargets(line);
      break;
    case 'X':
    case 'x':
      hardStop();
      timed_out = false;
      Serial.println("OK X stopped");
      break;
    case 'E':
    case 'e':
      engageEstop();
      break;
    case 'U':
    case 'u':
      releaseEstop();
      break;
    case 'R':
      restartMotion();
      break;
    case 'Z':
    case 'z':
      resetEncoders();
      Serial.println("OK Z encoders_reset");
      break;
    case 'H':
    case 'h':
    case '?':
      printHelp();
      break;
    default:
      Serial.print("ERR unknown_command ");
      Serial.println(command);
      break;
  }
}

void readCommands() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line_buffer[line_length] = '\0';
      handleLine(line_buffer);
      line_length = 0;
      continue;
    }
    if (static_cast<size_t>(line_length) + 1 < sizeof(line_buffer)) {
      line_buffer[line_length++] = c;
    } else {
      line_length = 0;
      Serial.println("ERR line_too_long");
    }
  }
}

void updateWatchdog(uint32_t now_ms) {
  if (estop_active || motion_inhibited) {
    return;
  }
  if (target_left == 0 && target_right == 0) {
    return;
  }
  if (now_ms - last_velocity_command_ms <= COMMAND_TIMEOUT_MS) {
    return;
  }
  timed_out = true;
  hardStop();
  Serial.println("WARN command_timeout stopped");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    last_encoder_state[i] = (digitalReadFast(ENC_A[i]) << 1) | digitalReadFast(ENC_B[i]);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), updateEncoderFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), updateEncoderFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), updateEncoderRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), updateEncoderRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), updateEncoderFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[2]), updateEncoderFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), updateEncoderRR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[3]), updateEncoderRR, CHANGE);

  Serial.begin(USB_BAUD);
  Serial1.begin(SABER_BAUD);
  Serial2.begin(SABER_BAUD);
  delay(1500);

  hardStop();
  saberSend(Serial1, LEFT_ADDRESS, 16, 20);
  saberSend(Serial2, RIGHT_ADDRESS, 16, 20);
  last_velocity_command_ms = millis();

  Serial.println("BOOT innex1_teensy_drivetrain_serial v0");
  printHelp();
}

void loop() {
  const uint32_t now_ms = millis();
  static uint32_t last_led_ms = 0;
  if (now_ms - last_led_ms >= 250) {
    last_led_ms = now_ms;
    digitalToggleFast(LED_BUILTIN);
  }

  readCommands();
  updateWatchdog(now_ms);
  updateRamp(now_ms);
  publishTelemetry(now_ms);
}
