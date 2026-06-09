// INNEX-1 Teensy 4.1 drivetrain serial firmware.
//
// Jetson/Mac USB serial commands, 115200 baud:
//   V <left> <right>   Set side throttle targets, -127..127.
//   D <motor> <speed>  Diagnostic single-motor command: FL/RL/FR/RR, -127..127.
//   C <dir1> <dir2>    Set Cytron MDD10A #1 actuator directions (-1=retract, 0=stop, 1=extend).
//   M <dir1> <dir2>    Set Cytron MDD10A #2 actuator directions (-1=retract, 0=stop, 1=extend).
//   B <speed>          Set BLDC (BLD-510B) speed -127..127 (0=stop/disable, +fwd, -rev).
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
//   Right Sabertooth S1 <- Teensy pin 29 (Serial7 TX)
//   Sabertooth 0V       <-> Teensy GND
//   Encoders red/black  -> Teensy 3.3V/GND
//   Encoder A/B pins: FL 15/16, RL 17/18, FR 19/20, RR 21/22
//   Cytron MDD10A #1 Act1: PWM <- pin 2, DIR <- pin 9
//   Cytron MDD10A #1 Act2: PWM <- pin 3, DIR <- pin 28  (pin 10 dead, solder fault)
//   Cytron MDD10A #2 Act3: PWM <- pin 4, DIR <- pin 11
//   Cytron MDD10A #2 Act4: PWM <- pin 7, DIR <- pin 8   (pins 5/12 dead, solder fault)
//   BLD-510B SV  <- pin 6  (PWM speed)
//   BLD-510B F/R <- pin 13 (direction, active-low)
//   BLD-510B EN  <- pin 14 (enable, active-low)

constexpr uint8_t ENCODER_COUNT = 4;
constexpr uint8_t ENC_A[ENCODER_COUNT] = {15, 17, 19, 21};
constexpr uint8_t ENC_B[ENCODER_COUNT] = {16, 18, 20, 22};

// Cytron MDD10A #1 — Actuators 1 & 2
constexpr uint8_t CYTRON_PWM[2] = {2, 3};
constexpr uint8_t CYTRON_DIR[2] = {9, 28};  // pin 10 dead (solder fault), moved DIR2 to pin 28

// Cytron MDD10A #2 — Actuators 3 & 4
// pin 5 and pin 12 confirmed dead (solder fault) — remapped to 7 and 8
constexpr uint8_t CYTRON2_PWM[2] = {4, 7};
constexpr uint8_t CYTRON2_DIR[2] = {11, 8};

// BLD-510B BLDC controller
constexpr uint8_t BLDC_SV_PIN = 6;   // PWM speed
constexpr uint8_t BLDC_FR_PIN = 13;  // Direction (active-low: LOW=fwd, HIGH=rev)
constexpr uint8_t BLDC_EN_PIN = 14;  // Enable (active-low: LOW=enabled)

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

int8_t cytron_dir[2] = {0, 0};   // per-channel: -1=retract, 0=stop, 1=extend
int8_t cytron2_dir[2] = {0, 0};  // Cytron #2 channels
int8_t bldc_speed = 0;            // -127..127

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
  saberMotor(Serial7, RIGHT_ADDRESS, false, command_right);
  saberMotor(Serial7, RIGHT_ADDRESS, true, command_right);
  Serial1.flush();
  Serial7.flush();
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
  Serial.println("OK H V <l> <r> | D <motor> <spd> | C <d1> <d2> | M <d1> <d2> | B <spd> | P <pin> <0|1|pwm> | X | E | U | R | Z");
}

void probePinCommand(char *args) {
  // P <pin> <value>  — value: 0=LOW, 1=HIGH, 2-255=analogWrite PWM
  char *pin_text = strtok(args, " ");
  char *val_text = strtok(nullptr, " ");
  if (pin_text == nullptr || val_text == nullptr) {
    Serial.println("ERR P expected_pin_value");
    return;
  }
  const int pin = atoi(pin_text);
  const int val = atoi(val_text);
  pinMode(pin, OUTPUT);
  if (val <= 1) {
    digitalWrite(pin, val ? HIGH : LOW);
  } else {
    analogWrite(pin, constrain(val, 0, 255));
  }
  Serial.print("OK P pin=");
  Serial.print(pin);
  Serial.print(" val=");
  Serial.println(val);
}

void engageEstop() {
  estop_active = true;
  motion_inhibited = true;
  timed_out = false;
  hardStop();
  bldc_speed = 0;
  setBldcOutputs();
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

void setDiagnosticMotor(char *args) {
  if (estop_active || motion_inhibited) {
    hardStop();
    Serial.println("ERR D motion_inhibited");
    return;
  }

  char *motor_text = strtok(args, " ");
  char *speed_text = strtok(nullptr, " ");
  if (motor_text == nullptr || speed_text == nullptr) {
    Serial.println("ERR D expected_motor_speed");
    return;
  }

  const int speed = constrain(atoi(speed_text), -127, 127);
  target_left = 0;
  target_right = 0;
  command_left = 0;
  command_right = 0;
  timed_out = false;
  last_velocity_command_ms = millis();

  if (strcasecmp(motor_text, "FL") == 0) {
    saberMotor(Serial1, LEFT_ADDRESS, false, speed);
  } else if (strcasecmp(motor_text, "RL") == 0) {
    saberMotor(Serial1, LEFT_ADDRESS, true, speed);
  } else if (strcasecmp(motor_text, "FR") == 0) {
    saberMotor(Serial7, RIGHT_ADDRESS, false, speed);
  } else if (strcasecmp(motor_text, "RR") == 0) {
    saberMotor(Serial7, RIGHT_ADDRESS, true, speed);
  } else {
    hardStop();
    Serial.print("ERR D unknown_motor ");
    Serial.println(motor_text);
    return;
  }

  Serial1.flush();
  Serial7.flush();
  Serial.print("OK D ");
  Serial.print(motor_text);
  Serial.print(' ');
  Serial.println(speed);
}

void setCytronOutputs() {
  for (uint8_t i = 0; i < 2; ++i) {
    if (cytron_dir[i] == 0) {
      analogWrite(CYTRON_PWM[i], 0);
    } else {
      digitalWrite(CYTRON_DIR[i], cytron_dir[i] > 0 ? HIGH : LOW);
      analogWrite(CYTRON_PWM[i], 255);
    }
  }
}

void setCytronCommand(char *args) {
  char *d1_text = strtok(args, " ");
  char *d2_text = strtok(nullptr, " ");
  if (d1_text == nullptr || d2_text == nullptr) {
    Serial.println("ERR C expected_dir1_dir2");
    return;
  }
  cytron_dir[0] = constrain(atoi(d1_text), -1, 1);
  cytron_dir[1] = constrain(atoi(d2_text), -1, 1);
  setCytronOutputs();
  Serial.print("OK C ");
  Serial.print(cytron_dir[0]);
  Serial.print(' ');
  Serial.println(cytron_dir[1]);
}

void setCytron2Outputs() {
  for (uint8_t i = 0; i < 2; ++i) {
    if (cytron2_dir[i] == 0) {
      analogWrite(CYTRON2_PWM[i], 0);
    } else {
      digitalWrite(CYTRON2_DIR[i], cytron2_dir[i] > 0 ? HIGH : LOW);
      analogWrite(CYTRON2_PWM[i], 255);
    }
  }
}

void setCytron2Command(char *args) {
  char *d1_text = strtok(args, " ");
  char *d2_text = strtok(nullptr, " ");
  if (d1_text == nullptr || d2_text == nullptr) {
    Serial.println("ERR M expected_dir1_dir2");
    return;
  }
  cytron2_dir[0] = constrain(atoi(d1_text), -1, 1);
  cytron2_dir[1] = constrain(atoi(d2_text), -1, 1);
  setCytron2Outputs();
  Serial.print("OK M ");
  Serial.print(cytron2_dir[0]);
  Serial.print(' ');
  Serial.println(cytron2_dir[1]);
}

void setBldcOutputs() {
  if (bldc_speed == 0) {
    analogWrite(BLDC_SV_PIN, 0);
    digitalWrite(BLDC_EN_PIN, HIGH);  // disable
  } else {
    digitalWrite(BLDC_FR_PIN, bldc_speed > 0 ? LOW : HIGH);
    analogWrite(BLDC_SV_PIN, map(abs(bldc_speed), 0, 127, 0, 255));
    digitalWrite(BLDC_EN_PIN, LOW);  // enable
  }
}

void setBldcCommand(char *args) {
  char *s_text = strtok(args, " ");
  if (s_text == nullptr) {
    Serial.println("ERR B expected_speed");
    return;
  }
  bldc_speed = constrain(atoi(s_text), -127, 127);
  setBldcOutputs();
  Serial.print("OK B ");
  Serial.println(bldc_speed);
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
    case 'D':
    case 'd':
      setDiagnosticMotor(line);
      break;
    case 'C':
    case 'c':
      setCytronCommand(line);
      break;
    case 'M':
    case 'm':
      setCytron2Command(line);
      break;
    case 'B':
    case 'b':
      setBldcCommand(line);
      break;
    case 'P':
    case 'p':
      probePinCommand(line);
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

  // Cytron MDD10A #1 — start stopped
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(CYTRON_PWM[i], OUTPUT);
    pinMode(CYTRON_DIR[i], OUTPUT);
    analogWrite(CYTRON_PWM[i], 0);
    digitalWrite(CYTRON_DIR[i], LOW);
  }

  // Cytron MDD10A #2 — start stopped
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(CYTRON2_PWM[i], OUTPUT);
    pinMode(CYTRON2_DIR[i], OUTPUT);
    analogWrite(CYTRON2_PWM[i], 0);
    digitalWrite(CYTRON2_DIR[i], LOW);
  }

  // BLD-510B — start disabled
  pinMode(BLDC_SV_PIN, OUTPUT);
  pinMode(BLDC_FR_PIN, OUTPUT);
  pinMode(BLDC_EN_PIN, OUTPUT);
  analogWrite(BLDC_SV_PIN, 0);
  digitalWrite(BLDC_FR_PIN, LOW);
  digitalWrite(BLDC_EN_PIN, HIGH);  // active-low: HIGH = disabled

  Serial.begin(USB_BAUD);
  Serial1.begin(SABER_BAUD);
  Serial7.begin(SABER_BAUD);
  delay(1500);

  hardStop();
  saberSend(Serial1, LEFT_ADDRESS, 16, 20);
  saberSend(Serial7, RIGHT_ADDRESS, 16, 20);
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
