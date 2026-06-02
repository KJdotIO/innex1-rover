// Teensy 4.1 + Sabertooth 2x32 one-motor bench test.
//
// Front-left test wiring:
//   Encoder red    -> Teensy 3.3V
//   Encoder black  -> Teensy GND
//   Encoder white  -> Teensy pin 15  (CH A)
//   Encoder yellow -> Teensy pin 16  (CH B)
//   Teensy pin 1   -> Sabertooth S1
//   Teensy GND     -> Sabertooth 0V
//   Motor thick wires -> Sabertooth M1A/M1B
//
// The sketch does not move the motor on boot. Use USB Serial commands:
//   f = M1 forward at TEST_SPEED
//   b = M1 reverse at TEST_SPEED
//   s = ramped stop
//   x = immediate stop
//   t = 2s forward, stop, 2s reverse, stop
//   y = 10s run with simulated E-stop at 5s
//   E = simulate E-stop press: immediate stop + latch inhibit
//   U = simulate E-stop release: inhibit remains latched
//   R = restart/reset motion inhibit, only works after E-stop release
//   r = reset encoder count

constexpr uint8_t ENCODER_COUNT = 4;
constexpr uint8_t ENC_A[ENCODER_COUNT] = {15, 17, 19, 21};
constexpr uint8_t ENC_B[ENCODER_COUNT] = {16, 18, 20, 22};
const char *const ENCODER_NAME[ENCODER_COUNT] = {"FL", "RL", "FR", "RR"};
constexpr uint8_t SABER_ADDRESS = 128;
constexpr int TEST_SPEED = 80;
constexpr int RAMP_STEP = 5;
constexpr uint16_t RAMP_STEP_MS = 100;
constexpr float COUNTS_PER_OUTPUT_REV = 720.0f;

volatile int32_t encoder_ticks[ENCODER_COUNT] = {0, 0, 0, 0};
volatile uint8_t last_state[ENCODER_COUNT] = {0, 0, 0, 0};
int current_speed = 0;
bool estop_active = false;
bool motion_inhibited = false;

void updateEncoder(uint8_t index) {
  const uint8_t a = digitalReadFast(ENC_A[index]);
  const uint8_t b = digitalReadFast(ENC_B[index]);
  const uint8_t state = (a << 1) | b;
  const uint8_t transition = (last_state[index] << 2) | state;

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

  last_state[index] = state;
}

void updateEncoderFL() { updateEncoder(0); }
void updateEncoderRL() { updateEncoder(1); }
void updateEncoderFR() { updateEncoder(2); }
void updateEncoderRR() { updateEncoder(3); }

void saberSend(uint8_t command, uint8_t value) {
  const uint8_t checksum = (SABER_ADDRESS + command + value) & 0x7F;
  Serial1.write(SABER_ADDRESS);
  Serial1.write(command);
  Serial1.write(value);
  Serial1.write(checksum);
  Serial1.flush();
}

void setMotor1(int speed) {
  speed = constrain(speed, -127, 127);
  current_speed = speed;
  if (speed >= 0) {
    saberSend(0, static_cast<uint8_t>(speed));
  } else {
    saberSend(1, static_cast<uint8_t>(-speed));
  }
}

void hardStopAll() {
  setMotor1(0);
  saberSend(4, 0);
}

void engageEstop() {
  estop_active = true;
  motion_inhibited = true;
  hardStopAll();
  Serial.println("ESTOP active: hard stop, motion inhibited");
}

void releaseEstop() {
  estop_active = false;
  Serial.println("ESTOP released: restart required before motion");
}

void restartMotion() {
  if (estop_active) {
    Serial.println("restart ignored: ESTOP is still active");
    return;
  }
  motion_inhibited = false;
  Serial.println("motion restart accepted");
}

bool motionAllowed() {
  if (estop_active || motion_inhibited) {
    hardStopAll();
    Serial.println("motion command ignored: inhibited");
    return false;
  }
  return true;
}

void rampTo(int target_speed) {
  if (!motionAllowed()) {
    return;
  }

  target_speed = constrain(target_speed, -127, 127);

  while (current_speed != target_speed) {
    if (estop_active || motion_inhibited) {
      hardStopAll();
      return;
    }
    const int delta = target_speed - current_speed;
    const int step = constrain(delta, -RAMP_STEP, RAMP_STEP);
    setMotor1(current_speed + step);
    delay(RAMP_STEP_MS);
  }
}

void rampedStopAll() {
  rampTo(0);
  saberSend(4, 0);
}

void resetEncoder() {
  noInterrupts();
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    encoder_ticks[i] = 0;
  }
  interrupts();
}

void printEncoder() {
  int32_t snapshot[ENCODER_COUNT];
  noInterrupts();
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    snapshot[i] = encoder_ticks[i];
  }
  interrupts();

  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    Serial.print(ENCODER_NAME[i]);
    Serial.print("=");
    Serial.print(snapshot[i]);
    Serial.print("(");
    Serial.print(static_cast<float>(snapshot[i]) / COUNTS_PER_OUTPUT_REV, 3);
    Serial.print("rev)");
    if (i + 1 < ENCODER_COUNT) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

void timedTest() {
  if (!motionAllowed()) {
    return;
  }

  Serial.println("timed test: ramp forward, hold 2s");
  resetEncoder();
  rampTo(TEST_SPEED);
  delay(2000);
  rampedStopAll();
  printEncoder();
  delay(500);

  Serial.println("timed test: ramp reverse, hold 2s");
  resetEncoder();
  rampTo(-TEST_SPEED);
  delay(2000);
  rampedStopAll();
  printEncoder();
  Serial.println("timed test: done");
}

void estopTimedTest() {
  if (!motionAllowed()) {
    return;
  }

  Serial.println("estop test: ramp forward, simulate ESTOP at 5s");
  resetEncoder();
  rampTo(TEST_SPEED);

  const uint32_t start_ms = millis();
  bool tripped = false;
  while ((millis() - start_ms) < 10000) {
    if (!tripped && (millis() - start_ms) >= 5000) {
      tripped = true;
      engageEstop();
    }
    delay(20);
  }

  printEncoder();
  Serial.println("estop test: done; send U then R before motion");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    last_state[i] = (digitalReadFast(ENC_A[i]) << 1) | digitalReadFast(ENC_B[i]);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), updateEncoderFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), updateEncoderFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), updateEncoderRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), updateEncoderRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), updateEncoderFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[2]), updateEncoderFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), updateEncoderRR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[3]), updateEncoderRR, CHANGE);

  Serial.begin(115200);
  Serial1.begin(9600);
  delay(1500);

  hardStopAll();
  saberSend(16, 20);

  Serial.println("INNEX-1 Teensy Sabertooth encoder bench");
  Serial.println("Commands: f=forward b=reverse s=ramped-stop x=hard-stop t=timed y=estop-test E=estop U=release R=restart r=reset p=print");
  Serial.print("TEST_SPEED=");
  Serial.println(TEST_SPEED);
  Serial.print("RAMP_STEP=");
  Serial.print(RAMP_STEP);
  Serial.print(" every ");
  Serial.print(RAMP_STEP_MS);
  Serial.println(" ms");
}

void loop() {
  static uint32_t last_print_ms = 0;
  static uint32_t last_led_ms = 0;
  const uint32_t now = millis();

  if (now - last_led_ms >= 250) {
    last_led_ms = now;
    digitalToggleFast(LED_BUILTIN);
  }

  if (now - last_print_ms >= 500) {
    last_print_ms = now;
    printEncoder();
  }

  if (!Serial.available()) {
    return;
  }

  const char command = Serial.read();
  if (command == 'f') {
    Serial.println("M1 ramp forward");
    rampTo(TEST_SPEED);
  } else if (command == 'b') {
    Serial.println("M1 ramp reverse");
    rampTo(-TEST_SPEED);
  } else if (command == 's') {
    Serial.println("ramped stop");
    rampedStopAll();
  } else if (command == 'x') {
    Serial.println("hard stop");
    hardStopAll();
  } else if (command == 't') {
    timedTest();
  } else if (command == 'y') {
    estopTimedTest();
  } else if (command == 'E') {
    engageEstop();
  } else if (command == 'U') {
    releaseEstop();
  } else if (command == 'R') {
    restartMotion();
  } else if (command == 'r') {
    resetEncoder();
    Serial.println("reset");
  } else if (command == 'p') {
    printEncoder();
  }
}
