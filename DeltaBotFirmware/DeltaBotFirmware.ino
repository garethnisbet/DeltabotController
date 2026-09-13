/*
   DeltaBotFirmware - serial motion controller for the 3d printed DeltaBot.

   Hardware: Arduino Nano + GRBL CNC shield (V3/V4), three 28BYJ-48 steppers
   rewired as bipolar, each turning an M3 x 20 hex bolt in a captive nut so
   that it acts as a screw jack under one corner of the flexure top plate.

   The firmware deliberately knows nothing about the geometry of the machine:
   it speaks in steps and leaves millimetres, tilts and Z to the Python host
   (see ../python/deltabot).

   Protocol: one ASCII command per line, terminated with '\n'.  Every command
   answers with one or more data lines followed by "ok" or "err <reason>".
   When a move finishes the firmware emits an unsolicited "done" line.

     M a b c   move to absolute step positions
     R a b c   move by a relative number of steps
     J n d     jog axis n (0..2) by d steps
     P         report position          -> "pos a b c"
     ?         report full status       -> "status ..."
     V v       set max speed (steps/s)
     A a       set acceleration (steps/s^2)
     E 0|1     disable / enable the drivers
     Z         set the current position as zero
     Z a b c   force the current position to a b c (no motion)
     B lo hi   set the soft limits, in steps (lo == hi disables them)
     S         stop, decelerating
     X         emergency stop, immediate
     $         print this command summary
*/

#include <AccelStepper.h>

#define FIRMWARE_ID "DeltaBot 1.0"

// ---------------------------------------------------------------- pin map
// GRBL shield: EN is active low and shared by all three drivers.
// These step/dir pairs match the original ThreeMotorStepper demo sketch; if
// an axis refuses to move, swap the two numbers for it.
const uint8_t EN_PIN = 8;
const uint8_t STEP_PIN[3] = {7, 6, 5};
const uint8_t DIR_PIN[3]  = {4, 3, 2};

// Flip an entry to true if positive steps drive that corner the wrong way.
const bool DIR_INVERT[3] = {false, false, false};

// ------------------------------------------------------------- parameters
const float DEFAULT_SPEED = 600.0;    // steps/s   (28BYJ-48 stalls if pushed)
const float DEFAULT_ACCEL = 2000.0;   // steps/s^2

AccelStepper stepper[3] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2])
};

float maxSpeed = DEFAULT_SPEED;
float accel    = DEFAULT_ACCEL;
long  softMin  = -400000L;
long  softMax  =  400000L;
bool  enabled  = true;
bool  moving   = false;

char  line[64];
uint8_t lineLen = 0;

// --------------------------------------------------------------- helpers
void setEnabled(bool on) {
  enabled = on;
  digitalWrite(EN_PIN, on ? LOW : HIGH);   // active low
}

bool withinLimits(long target) {
  if (softMin == softMax) return true;     // limits disabled
  return target >= softMin && target <= softMax;
}

// Scale each axis so that all three finish together: the axis with the
// longest travel runs at the full speed, the others in proportion.  This
// keeps the platform moving along a straight line in joint space while
// still getting AccelStepper's acceleration ramps.
void startMove(long target[3]) {
  long span = 0;
  for (uint8_t i = 0; i < 3; i++) {
    long d = target[i] - stepper[i].currentPosition();
    if (d < 0) d = -d;
    if (d > span) span = d;
  }
  for (uint8_t i = 0; i < 3; i++) {
    long d = target[i] - stepper[i].currentPosition();
    if (d < 0) d = -d;
    float f = (span > 0) ? (float)d / (float)span : 1.0;
    if (f < 0.001) f = 0.001;
    stepper[i].setMaxSpeed(maxSpeed * f);
    stepper[i].setAcceleration(accel * f);
    stepper[i].moveTo(target[i]);
  }
  moving = span > 0;
}

void reportPosition() {
  Serial.print(F("pos "));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(stepper[i].currentPosition());
    Serial.print(i < 2 ? ' ' : '\n');
  }
}

void reportStatus() {
  Serial.print(F("status "));
  Serial.print(moving ? F("moving") : F("idle"));
  Serial.print(F(" pos "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(stepper[i].currentPosition()); Serial.print(' '); }
  Serial.print(F("tgt "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(stepper[i].targetPosition()); Serial.print(' '); }
  Serial.print(F("en ")); Serial.print(enabled ? 1 : 0);
  Serial.print(F(" speed ")); Serial.print(maxSpeed);
  Serial.print(F(" accel ")); Serial.print(accel);
  Serial.print(F(" limits ")); Serial.print(softMin); Serial.print(' '); Serial.println(softMax);
}

void printHelp() {
  Serial.println(F("M a b c | R a b c | J n d | P | ? | V v | A a | E 0|1"));
  Serial.println(F("Z [a b c] | B lo hi | S | X | $"));
}

// Pull up to `count` whitespace separated longs out of `s`.
// Returns the number actually parsed.
uint8_t parseLongs(char *s, long *out, uint8_t count) {
  uint8_t n = 0;
  char *tok = strtok(s, " ,\t");
  while (tok != NULL && n < count) {
    out[n++] = atol(tok);
    tok = strtok(NULL, " ,\t");
  }
  return n;
}

// ---------------------------------------------------------- command entry
void handleLine(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;

  char cmd = toupper(*s);
  char *args = s + 1;
  long v[3];

  switch (cmd) {
    case 'M':
    case 'R': {
      if (parseLongs(args, v, 3) != 3) { Serial.println(F("err need 3 values")); return; }
      long target[3];
      for (uint8_t i = 0; i < 3; i++)
        target[i] = (cmd == 'M') ? v[i] : stepper[i].currentPosition() + v[i];
      for (uint8_t i = 0; i < 3; i++)
        if (!withinLimits(target[i])) { Serial.println(F("err soft limit")); return; }
      if (!enabled) setEnabled(true);
      startMove(target);
      Serial.println(F("ok"));
      break;
    }
    case 'J': {
      if (parseLongs(args, v, 2) != 2) { Serial.println(F("err need axis and steps")); return; }
      if (v[0] < 0 || v[0] > 2) { Serial.println(F("err axis must be 0..2")); return; }
      long target[3];
      for (uint8_t i = 0; i < 3; i++) target[i] = stepper[i].targetPosition();
      target[v[0]] = stepper[v[0]].currentPosition() + v[1];
      if (!withinLimits(target[v[0]])) { Serial.println(F("err soft limit")); return; }
      if (!enabled) setEnabled(true);
      startMove(target);
      Serial.println(F("ok"));
      break;
    }
    case 'P': reportPosition(); Serial.println(F("ok")); break;
    case '?': reportStatus();   Serial.println(F("ok")); break;
    case 'V': {
      float f = atof(args);
      if (f <= 0) { Serial.println(F("err speed must be > 0")); return; }
      maxSpeed = f;
      for (uint8_t i = 0; i < 3; i++) stepper[i].setMaxSpeed(maxSpeed);
      Serial.println(F("ok"));
      break;
    }
    case 'A': {
      float f = atof(args);
      if (f <= 0) { Serial.println(F("err accel must be > 0")); return; }
      accel = f;
      for (uint8_t i = 0; i < 3; i++) stepper[i].setAcceleration(accel);
      Serial.println(F("ok"));
      break;
    }
    case 'E': {
      if (parseLongs(args, v, 1) != 1) { Serial.println(F("err need 0 or 1")); return; }
      setEnabled(v[0] != 0);
      Serial.println(F("ok"));
      break;
    }
    case 'Z': {
      uint8_t n = parseLongs(args, v, 3);
      for (uint8_t i = 0; i < 3; i++) {
        long p = (n == 3) ? v[i] : 0L;
        stepper[i].setCurrentPosition(p);   // also clears speed and target
      }
      moving = false;
      Serial.println(F("ok"));
      break;
    }
    case 'B': {
      if (parseLongs(args, v, 2) != 2) { Serial.println(F("err need lo and hi")); return; }
      if (v[1] < v[0]) { Serial.println(F("err hi below lo")); return; }
      softMin = v[0];
      softMax = v[1];
      Serial.println(F("ok"));
      break;
    }
    case 'S':
      for (uint8_t i = 0; i < 3; i++) stepper[i].stop();   // decelerate to a halt
      Serial.println(F("ok"));
      break;
    case 'X':
      for (uint8_t i = 0; i < 3; i++) stepper[i].setCurrentPosition(stepper[i].currentPosition());
      moving = false;
      Serial.println(F("ok"));
      break;
    case '$': printHelp(); Serial.println(F("ok")); break;
    default:  Serial.println(F("err unknown command")); break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        line[lineLen] = '\0';
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;                       // overlong line, drop it
      Serial.println(F("err line too long"));
    }
  }
}

// ------------------------------------------------------------------ main
void setup() {
  pinMode(EN_PIN, OUTPUT);
  setEnabled(true);

  for (uint8_t i = 0; i < 3; i++) {
    stepper[i].setPinsInverted(DIR_INVERT[i], false, false);
    stepper[i].setMaxSpeed(maxSpeed);
    stepper[i].setAcceleration(accel);
    stepper[i].setCurrentPosition(0);
  }

  Serial.begin(115200);
  Serial.print(F(FIRMWARE_ID));
  Serial.println(F(" ready"));
}

void loop() {
  pollSerial();

  bool busy = false;
  for (uint8_t i = 0; i < 3; i++) {
    stepper[i].run();
    if (stepper[i].distanceToGo() != 0) busy = true;
  }

  if (moving && !busy) {
    moving = false;
    Serial.println(F("done"));
  }
}
