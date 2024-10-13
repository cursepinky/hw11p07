#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

// Target Distance
#define _TARGET_LOW  180.0
#define _TARGET_HIGH 360.0

// Servo duty cycle (NEEDS TUNING)
#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)

// global variables
float dist_ema = _DIST_MAX, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;                  // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  // attach servo and set to neutral
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  // wait until next sampling time
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // Apply range filter
  if (dist_raw == 0.0 || dist_raw > _DIST_MAX || dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;          // Use previous distance when out of range
    digitalWrite(PIN_LED, HIGH);   // LED OFF
  } else {
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);    // LED ON
  }

  // Apply EMA filter
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // Map distance to servo angle (0-180 degrees based on distance)
  int servo_angle = map(dist_ema, _DIST_MIN, _DIST_MAX, 0, 180);
  int duty = map(servo_angle, 0, 180, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(duty);

  // Output data to serial port
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(", dist_raw:"); Serial.print(dist_raw);
  Serial.print(", dist_ema:"); Serial.print(dist_ema);
  Serial.print(", Servo Angle:"); Serial.print(servo_angle);
  Serial.print(", Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
