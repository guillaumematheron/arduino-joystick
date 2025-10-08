#import <string.h>
#include <EEPROM.h>

const int pin_JoystickLX = A6;
const int pin_JoystickLY = A7;
const int pin_JoystickRX = A2;
const int pin_JoystickRY = A3;

class Button {
  int debounce_steps = 10;
  int pin1;
  int pin2;
  int since_last_change;
  bool flip;
  unsigned long hold_start;

  public:
  int state;
  bool click;
  float hold_duration;

  Button(const int p_pin1, const int p_pin2, const bool p_flip) {
    debounce_steps = 10;
    pin1 = p_pin1;
    pin2 = p_pin2;
    flip = p_flip;
    click = false;

    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, LOW);
    pinMode(pin2, INPUT_PULLUP);

    state = flip ^ digitalRead(pin2);
    since_last_change = 0;
  }

  void loop() {
    const int new_state = flip ^ digitalRead(pin2);
    click = false;
    if (new_state == state) {
      since_last_change = 0;
      if (state)
        hold_duration = float(millis() - hold_start)/1000;
    }
    else {
      if (since_last_change < debounce_steps) {
        ++since_last_change;
      }
      else {
        state = new_state;
        since_last_change = 0;
        if (!state) {
          hold_duration = 0;
        }
        else {
          hold_start = millis();
          click = true;
        }
      }
    }
    //Serial.println(String("s ")+state+" "+hold_duration);
  }
};

float map_float(float value, uint16_t fromLow, uint16_t fromHigh, float toLow, float toHigh) {
 // Serial.println(String("")+value+" "+fromLow+" "+fromHigh+" "+toLow+" "+toHigh);
  return toLow + (value - (float)fromLow) / ((float)fromHigh - (float)fromLow) * (toHigh - toLow);
}

class Lever {
  Button const* at_disc;
  int pin;
  float out_idle = 0.000;
  float out_cl =   0.625;
  float out_mct =  0.800;
  float out_toga = 1.000;

  uint16_t in_rev = 826;
  uint16_t in_idle = 748;
  uint16_t in_cl = 661;
  uint16_t in_mct = 620;
  uint16_t in_toga = 579;
  uint16_t margin = 5;

  bool flip_axis;
  uint8_t slot;

  // 0 -> normal
  // 1 -> setting REV
  // 2 -> setting IDLE
  // 3 -> setting CL
  // 4 -> setting MCT
  // 5 -> setting TOGA
  int fsm;

  public:
  float state;
  bool is_reverse;

  Lever(const uint8_t p_slot, const int p_pin, Button const* p_at_disc, bool p_flip_axis) {
    slot = p_slot;
    pin = p_pin;
    at_disc = p_at_disc;
    flip_axis = p_flip_axis;
    fsm = 0;

    if (EEPROM.read(slot * 11 + 0) == 42) {
      EEPROM.get(slot * 11 + 1, in_rev);
      EEPROM.get(slot * 11 + 3, in_idle);
      EEPROM.get(slot * 11 + 5, in_cl);
      EEPROM.get(slot * 11 + 7, in_mct);
      EEPROM.get(slot * 11 + 9, in_toga);
    }

    pinMode(pin, INPUT);
  }

  uint16_t readPin() {
    if (!flip_axis)
      return analogRead(pin);
    return 65535 - analogRead(pin);
  }

  void loop() {
    if (fsm == 0) {
      if (at_disc->hold_duration > 2)
        fsm = 1;

      uint16_t v = readPin();
      if (v < in_idle - margin) {
        is_reverse = true;
        state = map_float(max(v, in_rev), in_rev, in_idle, out_toga, out_idle);
        return;
      }
      else {
        is_reverse = false;
      }

      if (v < in_idle + margin) {
        // Idle
        state = out_idle;
      }
      else if (v < in_cl - margin) {
        // Manual
        state = map_float(v, in_idle, in_cl, out_idle, out_cl);
      }
      else if (v < in_cl + margin) {
        // CL
        state = out_cl;
      }
      else if (v < in_mct - margin) {
        // Between CL and MCT ?
        state = map_float(v, in_cl, in_mct, out_cl, out_mct);
      }
      else if (v < in_mct + margin) {
        // MCT
        state = out_mct;
      }
      else if (v < in_toga - margin) {
        // Between MCT and TOGA ?
        state = map_float(min(v,in_toga), in_mct, in_toga, out_mct, out_toga);
      }
      else {
        // TOGA
        state = out_toga;
      }
    }
    else if (fsm == 1) {
      state = out_toga;
      is_reverse = true;
      if (at_disc->click) {
        in_rev = readPin();
        fsm = 2;
      }
    }
    else if (fsm == 2) {
      state = out_idle;
      is_reverse = false;
      if (at_disc->click) {
        in_idle = readPin();
        fsm = 3;
      }
    }
    else if (fsm == 3) {
      state = out_cl;
      is_reverse = false;
      if (at_disc->click) {
        in_cl = readPin();
        fsm = 4;
      }
    }
    else if (fsm == 4) {
      state = out_mct;
      is_reverse = false;
      if (at_disc->click) {
        in_mct = readPin();
        fsm = 5;
      }
    }
    else if (fsm == 5) {
      state = out_toga;
      is_reverse = false;
      if (at_disc->click) {
        in_toga = readPin();
        fsm = 0;

        Serial.println(String("in_rev = ") + in_rev);
        Serial.println(String("in_idle = ") + in_idle);
        Serial.println(String("in_cl = ") + in_cl);
        Serial.println(String("in_mct = ") + in_mct);
        Serial.println(String("in_toga = ") + in_toga);

        EEPROM.update(slot * 11 + 0, 42);
        EEPROM.put(slot * 11 + 1, in_rev);
        EEPROM.put(slot * 11 + 3, in_idle);
        EEPROM.put(slot * 11 + 5, in_cl);
        EEPROM.put(slot * 11 + 7, in_mct);
        EEPROM.put(slot * 11 + 9, in_toga);
      }
    }
  }
};


// D-PAD + ABXY + START & SELECT + L1 & R1 + L2 & R2
#define NUM_BUTTONS 8

int buttons[NUM_BUTTONS] = {
  3, 3, 3, 3, 4, 5, 6, 7
};

bool IS_REVERSE = false;
uint16_t REVERSE_BUTTON = 0;
uint16_t REVERSE_ON_BUTTON = 0;
uint16_t REVERSE_OFF_BUTTON = 0;

Button AT_DISC_L(2, 3, true);
Button AT_DISC_R(5, 4, true);
Lever LEVER_L(0, A7, &AT_DISC_L, true);
Lever LEVER_R(1, A6, &AT_DISC_R, true);

void setup() {
  Serial.begin(115200);
/*
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);*/
}

void loop() {  
  AT_DISC_L.loop();
  AT_DISC_R.loop();
  LEVER_L.loop();
  LEVER_R.loop();
  // Serial.println(String("")+LEVER_L.state+" "+LEVER_R.state+" "+AT_DISC_R.state+" "+digitalRead(3)+" "+digitalRead(4)+" "+analogRead(A6)+" "+analogRead(A7));

  uint16_t joystickState[4];
  //readJoystickState(joystickState);


  uint16_t buttonState = 0;
  buttonState |= LEVER_L.is_reverse << 0;
  buttonState |= LEVER_R.is_reverse << 1;
  buttonState |= !LEVER_L.is_reverse << 2;
  buttonState |= !LEVER_R.is_reverse << 3;
  buttonState |= AT_DISC_L.state << 4;
  buttonState |= AT_DISC_R.state << 5;
  buttonState |= (AT_DISC_L.state | AT_DISC_R.state) << 6;


  uint16_t startShort = 65535; // 16 1s
  writeShort(startShort);
  writeShort(buttonState);
  writeShort(uint16_t(LEVER_L.state * 512 + 512));
  writeShort(uint16_t(LEVER_R.state * 512 + 512));
  writeShort(0);
  writeShort(0);

  delay(10);
}

void readJoystickState(uint16_t joystickState[4]) {
//  joystickState[0] = analogRead(pin_JoystickLX);
  joystickState[1] = analogRead(pin_JoystickLY);
  joystickState[2] = analogRead(pin_JoystickRX);
  joystickState[3] = analogRead(pin_JoystickRY);


  constexpr uint16_t in_rev =   638;
  constexpr uint16_t in_idle =  661;
  constexpr uint16_t in_cl =   766;
  constexpr uint16_t in_mct =  831;
  constexpr uint16_t in_toga = 892;
  constexpr uint16_t margin = 5;

  constexpr uint16_t out_idle = 0.000 * 1024/2+512;
  constexpr uint16_t out_cl =   0.625 * 1024/2+512;
  constexpr uint16_t out_mct =  0.800 * 1024/2+512;
  constexpr uint16_t out_toga = 1.000 * 1024/2+512;

  uint16_t v = analogRead(pin_JoystickLX);
  if (v < in_idle - margin) {
    // Reverse
    if (!IS_REVERSE) {
      REVERSE_BUTTON = 10;
      REVERSE_ON_BUTTON = 10;
    }
    IS_REVERSE = true;
    joystickState[0] = map(max(v, in_rev), in_rev, in_idle, out_toga, out_idle);
    return;
  }
  else {
    if (IS_REVERSE) {
      REVERSE_BUTTON = 10;
      REVERSE_OFF_BUTTON = 10;
    }
    IS_REVERSE = false;
  }

  if (v < in_idle + margin) {
    // Idle
    joystickState[0] = out_idle;
  }
  else if (v < in_cl - margin) {
    // Manual
    joystickState[0] = map(v, in_idle, in_cl, out_idle, out_cl);
  }
  else if (v < in_cl + margin) {
    // CL
    joystickState[0] = out_cl;
  }
  else if (v < in_mct - margin) {
    // Between CL and MCT ?
    joystickState[0] = map(v, in_cl, in_mct, out_cl, out_mct);
  }
  else if (v < in_mct + margin) {
    // MCT
    joystickState[0] = out_mct;
  }
  else if (v < in_toga - margin) {
    // Between MCT and TOGA ?
    joystickState[0] = map(min(v,in_toga), in_mct, in_toga, out_mct, out_toga);
  }
  else {
    // TOGA
    joystickState[0] = out_toga;
  }
  //Serial.println(v);
  //Serial.println(joystickState[0]);
}

void writeShort(uint16_t value) {
  /*char s[20];
  sprintf(s, "%hu", value);
  Serial.println(s);
  return;*/
  Serial.write((uint8_t) value);
  Serial.write((uint8_t) (value >> 8));
  //Serial.write('\n');
}
