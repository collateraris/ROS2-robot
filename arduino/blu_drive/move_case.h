#pragma once

void move_case(char command) {
  // Hardware PWM replaces the original software pulse loop. Speed is separate
  // from direction: changing speed cannot take ownership or revive S/D/E stop.
  if (command >= '0' && command <= '9') {
    setBluetoothSpeed((command - '0') * 255 / 10);
    return;
  }
  switch (command) {
    case 'F': takeBluetooth(-1, -1); break;
    case 'B': takeBluetooth( 1,  1); break;
    case 'L': takeBluetooth(-1,  1); break;
    case 'R': takeBluetooth( 1, -1); break;
    case 'G': takeBluetooth( 0,  1); break;
    case 'I': takeBluetooth( 1,  0); break;
    case 'H': takeBluetooth( 1, -1); break;
    case 'J': takeBluetooth(-1,  1); break;
    case 'S': case 'D': releaseBluetooth(); break;
    case 'q': setBluetoothSpeed(255); break;
    case 'X': case 'x': break;  // Original no-ops.
    default: break;
  }
}
