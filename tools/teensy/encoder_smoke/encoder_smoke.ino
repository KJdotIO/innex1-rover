// Teensy 4.1 encoder smoke test for one GR-WM4-V3 drivetrain motor.
//
// Wiring for the first test motor:
//   Encoder red    -> Teensy 3.3V
//   Encoder black  -> Teensy GND
//   Encoder white  -> Teensy pin 15  (CH A)
//   Encoder yellow -> Teensy pin 16  (CH B)
//
// Keep the encoder on 3.3V so its outputs are safe for Teensy GPIO.

constexpr uint8_t ENC_A = 15;
constexpr uint8_t ENC_B = 16;
constexpr float COUNTS_PER_OUTPUT_REV = 720.0f;

volatile int32_t encoder_count = 0;
volatile uint8_t last_state = 0;

void updateEncoder() {
  const uint8_t a = digitalReadFast(ENC_A);
  const uint8_t b = digitalReadFast(ENC_B);
  const uint8_t state = (a << 1) | b;
  const uint8_t transition = (last_state << 2) | state;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      ++encoder_count;
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      --encoder_count;
      break;
    default:
      break;
  }

  last_state = state;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  last_state = (digitalReadFast(ENC_A) << 1) | digitalReadFast(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, CHANGE);

  Serial.begin(115200);
  delay(1500);
  Serial.println("INNEX-1 Teensy encoder smoke test");
  Serial.println("Commands: r = reset count, p = print now");
}

void printEncoder() {
  noInterrupts();
  const int32_t count = encoder_count;
  interrupts();

  Serial.print("count=");
  Serial.print(count);
  Serial.print(" rev=");
  Serial.println(static_cast<float>(count) / COUNTS_PER_OUTPUT_REV, 4);
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

  if (Serial.available()) {
    const char command = Serial.read();
    if (command == 'r') {
      noInterrupts();
      encoder_count = 0;
      interrupts();
      Serial.println("reset");
    } else if (command == 'p') {
      printEncoder();
    }
  }
}
