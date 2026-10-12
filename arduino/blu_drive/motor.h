#pragma once

// Electrical sign: + means IN1 HIGH / IN2 LOW; - means the opposite.
// BT deliberately preserves the original wiring map, including F = (-, -).
int motorLeft = 0, motorRight = 0;

void writeWheel(uint8_t first, uint8_t second, uint8_t enable, int value) {
  analogWrite(enable, 0);  // Remove drive before changing direction.
  digitalWrite(first, value > 0 ? HIGH : LOW);
  digitalWrite(second, value < 0 ? HIGH : LOW);
  analogWrite(enable, value < 0 ? -value : value);
}

void setElectrical(int left, int right) {
  motorLeft = constrain(left, -255, 255);
  motorRight = constrain(right, -255, 255);
  writeWheel(L1, L2, ENA, motorLeft);
  writeWheel(R1, R2, ENB, motorRight);
}

void setupMotorPins() {
  const uint8_t pins[] = {L1, L2, R1, R2, ENA, ENB};
  for (uint8_t i = 0; i < sizeof(pins); ++i) pinMode(pins[i], OUTPUT);
  setElectrical(0, 0);  // PWM zero and both direction inputs LOW.
}
