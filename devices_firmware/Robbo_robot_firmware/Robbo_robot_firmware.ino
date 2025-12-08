#include "ServoTimer2.h"
#include <Arduino.h>
#include <EEPROM.h>

#define FIRMWARE_VERSION "00010"

#define SERIAL_SPEED 115200
#define SERIAL_ADDRESS 0
#define ADDRCALIB 1000
#define BLUE_CHECK 500

// Constants for motor control
#define MAX_UINT16 65535
#define PWM_MULTIPLIER 4
#define DIRECTION_BIT_MASK 64

// Constants for ultrasonic sensor
#define SONIC_RESET_DELAY_US 30000
#define SONIC_TIMEOUT_US 100000
#define SONIC_PULSE_WIDTH_US 9
#define SONIC_SPEED_OF_SOUND_CM_PER_US 0.034f
#define SONIC_DISTANCE_OFFSET_CM 14

// Constants for color sensor
#define COLOR_SYNC_THRESHOLD_US 1540 // (110 * 14)
#define COLOR_BIT_DIVISOR 14
#define COLOR_ADAPTIVE_TRIES_MAX 25
#define COLOR_ADAPTIVE_TRIES_MIN 5
#define COLOR_CHANGE_THRESHOLD 32
#define COLOR_STABILITY_THRESHOLD 5
#define COLOR_LOOP_TIMEOUT_US 10000
#define COLOR_INITIAL_INTERVAL_BIT 111

// Constants for calibration
#define CALIBRATION_DELAY_MS 800
#define CALIBRATION_CHECK_INTERVAL_MS 200

#define portOfPin(P)                                                           \
  (((P) >= 0 && (P) < 8) ? &PORTD : (((P) > 7 && (P) < 14) ? &PORTB : &PORTC))
#define ddrOfPin(P)                                                            \
  (((P) >= 0 && (P) < 8) ? &DDRD : (((P) > 7 && (P) < 14) ? &DDRB : &DDRC))
#define pinOfPin(P)                                                            \
  (((P) >= 0 && (P) < 8) ? &PIND : (((P) > 7 && (P) < 14) ? &PINB : &PINC))
#define pinIndex(P) ((uint8_t)(P > 13 ? P - 14 : P & 7))
#define pinMask(P) ((uint8_t)(1 << pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P)) &= ~pinMask(P)
#define pinAsInputPullUp(P)                                                    \
  *(ddrOfPin(P)) &= ~pinMask(P);                                               \
  digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P)) |= pinMask(P)
#define digitalLow(P) *(portOfPin(P)) &= ~pinMask(P)
#define digitalHigh(P) *(portOfPin(P)) |= pinMask(P)
#define isHigh(P) ((*(pinOfPin(P)) & pinMask(P)) > 0)
#define isLow(P) ((*(pinOfPin(P)) & pinMask(P)) == 0)
#define digitalState(P) ((uint8_t)isHigh(P))

#define CONNECTION_LOST_TIME_INTERVAL 2000

#define SENSOR_TYPE_NONE 0
#define SENSOR_TYPE_LINE 1
#define SENSOR_TYPE_LED 2
#define SENSOR_TYPE_LIGHT 3
#define SENSOR_TYPE_TOUCH 4
#define SENSOR_TYPE_PROXIMITY 5
#define SENSOR_TYPE_ULTRASONIC 6
#define SENSOR_TYPE_COLOR 7

#define DIGITAL_PIN_1 4
#define DIGITAL_PIN_2 7
#define DIGITAL_PIN_3 8
#define DIGITAL_PIN_4 11
#define DIGITAL_PIN_5 12

#define SENSOR_COUNT 5
#define SENSOR_RESPONSE_LENGTH 4

ServoTimer2 myservo;

char blue_num;
const byte MOTOR_STATE_DISABLED = 0;
const byte MOTOR_STATE_ENABLED = 1;
const byte MOTOR_STATE_STEPS_LIMIT = 2;

const byte DIRECTION_FORWARD = 0;
const byte DIRECTION_BACKWARD = 1;

const unsigned long ACCELERATION_INTERVAL_MILLISECONDS =
    5; // интервал между изменениями скорости, мс

struct Motor {
public:
  byte direction;
  volatile byte currentSpeed;
  byte maxSpeed;
  volatile unsigned int stepsLimit;
  volatile unsigned int PIDpath; // счётчик для ПИД регулятора
  volatile unsigned int stepsCnt;
  volatile unsigned int stepsDistance;
  volatile unsigned int stepsPath;
  volatile unsigned int PIDspeed; // начальная скорость для корректировки ПИДа
  byte directionPin;
  byte speedPin;

  unsigned long lastPwmUpdateMs;

  // Применение текущей скорости и направления к драйверу мотора.
  // При нулевой скорости обе линии держим в HIGH, чтобы гарантированно
  // не было ложного вращения при включении питания.
  void applyPwm() {
    if (currentSpeed == 0) {
      digitalWrite(speedPin, HIGH);
      digitalWrite(directionPin, HIGH);
    } else {
      if (direction == DIRECTION_FORWARD) {
        analogWrite(speedPin, currentSpeed * PWM_MULTIPLIER);
        analogWrite(directionPin, 0);
      } else {
        analogWrite(speedPin, 0);
        analogWrite(directionPin, currentSpeed * PWM_MULTIPLIER);
      }
    }
  }

  void initMotor(byte dPin, byte sPin) {
    direction = DIRECTION_FORWARD;
    currentSpeed = 0;
    // По умолчанию цель скорости 0, чтобы при старте не было автоподхвата в
    // updatePwm()
    maxSpeed = 0;
    stepsLimit = 0;
    PIDpath = 0;
    PIDspeed = 0;
    stepsCnt = 0;

    directionPin = dPin;
    speedPin = sPin;

    lastPwmUpdateMs = 0;

    pinMode(directionPin, OUTPUT);
    pinMode(speedPin, OUTPUT);

    // Сразу зажимаем оба пина в HIGH, чтобы моторы были гарантированно
    // выключены.
    digitalWrite(speedPin, HIGH);
    digitalWrite(directionPin, HIGH);
  }

  void stop() {
    currentSpeed = 0;
    maxSpeed = 0;
    applyPwm();
  }

  void enableAndSetStepsLimit(unsigned int iStepsLimit) {
    stepsCnt = 0;
    stepsLimit = iStepsLimit;
  }

  void setSpeedAndDirection(byte speed, byte dir) {
    direction = dir;
    maxSpeed = speed;

    if (speed == 0) {
      currentSpeed = 0;
      applyPwm(); // мгновенно останавливаем
    }
    // Для ненулевой скорости не прыгаем сразу на maxSpeed — плавно выйдем в
    // updatePwm().
  }

  void onStep() {
    if (stepsLimit > 0) {
      stepsCnt++;
    }

    stepsPath++;
    PIDpath++;

    if (direction == DIRECTION_FORWARD) {
      stepsDistance++;
    } else {
      stepsDistance--;
    }

    // Handle overflow for 16-bit counters
    if (stepsCnt > MAX_UINT16) {
      stepsCnt = 0;
    }
    if (stepsPath > MAX_UINT16) {
      stepsPath = 0;
    }

    if (PIDpath > MAX_UINT16) {
      PIDpath = 0;
    }

    if (stepsDistance > MAX_UINT16) {
      stepsDistance = 0;
    }
  }

  // Плавное изменение currentSpeed к maxSpeed и обновление выходов
  void updatePwm() {
    unsigned long now = millis();
    if (now - lastPwmUpdateMs < ACCELERATION_INTERVAL_MILLISECONDS) {
      return;
    }
    lastPwmUpdateMs = now;

    if (currentSpeed == maxSpeed) {
      return; // уже достигли целевой скорости
    }

    const byte accelStep = 1; // максимально изменяем скорость за шаг

    if (currentSpeed < maxSpeed) {
      byte delta = maxSpeed - currentSpeed;
      if (delta > accelStep)
        delta = accelStep;
      currentSpeed += delta;
    } else { // currentSpeed > maxSpeed
      byte delta = currentSpeed - maxSpeed;
      if (delta > accelStep)
        delta = accelStep;
      currentSpeed -= delta;
    }

    applyPwm();
  }
};

float calibrationRightToLeft = 1.0f;
float calibrationLeftToRight = 1.0f;
unsigned int lastEncoderLeft = 0;
unsigned int lastEncoderRight = 0;
byte encoderDelta = 1;
byte commandState;
const byte COMMAND_STATE_WAITING_COMMAND = 0;
const byte COMMAND_STATE_WAITING_COMMAND_TYPE = 1;
const byte COMMAND_STATE_WAITING_DATA = 2;
const byte COMMAND_STATE_WAITING_CRC = 3;
const byte COMMAND_STATE_EXECUTING = 4;

byte commandType;
byte motorControlFlag = 0;

Motor leftMotor;
Motor rightMotor;

float speed_proportion = 1;

byte globalLeftMotorSpeed;
byte globalRightMotorSpeed;

byte globalLeftDir;
byte globalRightDir;

float m_lastInput = 0;
float m_lastResult = 0;

// Common function to handle motor step limit reached
void handleMotorStepLimit(Motor &motor, Motor &otherMotor) {
  if ((motor.stepsLimit > 0) && (motor.stepsCnt >= motor.stepsLimit)) {
    motor.stepsLimit = 0;
    motor.stop();
    otherMotor.stepsLimit = 0;
    otherMotor.stop();
  }
}

void onLeftMotorStep() {
  leftMotor.onStep();
  handleMotorStepLimit(leftMotor, rightMotor);
}

void onRightMotorStep() {
  rightMotor.onStep();
  handleMotorStepLimit(rightMotor, leftMotor);
}

char chararrSerialRaw[50];
char chararrModel[21];
char chararrVersion[21];
char chararrPart[21];
char chararrSerial[21];
int MODEL_ID;

void parseSerialNumber() {
  EEPROM.get(SERIAL_ADDRESS, chararrSerialRaw);

  const int maxSerialLength = sizeof(chararrSerialRaw) - 1;
  const int maxModelLength = sizeof(chararrModel) - 1;
  const int maxVersionLength = sizeof(chararrVersion) - 1;
  const int maxPartLength = sizeof(chararrPart) - 1;
  const int maxSerialNumLength = sizeof(chararrSerial) - 1;

  int iPointer = 0;
  // Find first delimiter
  while (iPointer < maxSerialLength && chararrSerialRaw[iPointer] != '-' &&
         chararrSerialRaw[iPointer] != 0) {
    iPointer++;
  }
  if (iPointer >= maxSerialLength)
    return;
  iPointer++;

  // Parse model
  int iModelOffset = 0;
  while (iPointer < maxSerialLength && chararrSerialRaw[iPointer] != '-' &&
         chararrSerialRaw[iPointer] != 0 && iModelOffset < maxModelLength) {
    chararrModel[iModelOffset] = chararrSerialRaw[iPointer];
    iModelOffset++;
    iPointer++;
  }
  chararrModel[iModelOffset] = 0;
  if (iPointer >= maxSerialLength)
    return;
  iPointer++;

  // Parse version
  int iVersionOffset = 0;
  while (iPointer < maxSerialLength && chararrSerialRaw[iPointer] != '-' &&
         chararrSerialRaw[iPointer] != 0 && iVersionOffset < maxVersionLength) {
    chararrVersion[iVersionOffset] = chararrSerialRaw[iPointer];
    iVersionOffset++;
    iPointer++;
  }
  chararrVersion[iVersionOffset] = 0;
  if (iPointer >= maxSerialLength)
    return;
  iPointer++;

  // Parse part
  int iPartOffset = 0;
  while (iPointer < maxSerialLength && chararrSerialRaw[iPointer] != '-' &&
         chararrSerialRaw[iPointer] != 0 && iPartOffset < maxPartLength) {
    chararrPart[iPartOffset] = chararrSerialRaw[iPointer];
    iPartOffset++;
    iPointer++;
  }
  chararrPart[iPartOffset] = 0;
  if (iPointer >= maxSerialLength)
    return;
  iPointer++;

  // Parse serial number
  int iSerialOffset = 0;
  while (iPointer < maxSerialLength && chararrSerialRaw[iPointer] != 0 &&
         (chararrSerialRaw[iPointer] >= '0' &&
          chararrSerialRaw[iPointer] <= '9') &&
         iSerialOffset < maxSerialNumLength) {
    chararrSerial[iSerialOffset] = chararrSerialRaw[iPointer];
    iSerialOffset++;
    iPointer++;
  }
  chararrSerial[iSerialOffset] = 0;

  if (strcmp(chararrModel, "R") == 0 && strcmp(chararrVersion, "1") == 0) {
    MODEL_ID = 0;
  } else if (strcmp(chararrModel, "L") == 0 &&
             strcmp(chararrVersion, "1") == 0) {
    MODEL_ID = 1;
  } else if (strcmp(chararrModel, "L") == 0 &&
             strcmp(chararrVersion, "3") == 0) {
    MODEL_ID = 2;
  } else {
    MODEL_ID = 9999;
  }
}

byte byteActiveColorSensor = 0;

class ISensor {
public:
  virtual byte getType();
  virtual void reset();
  virtual void iteration(byte data, byte data2);
  virtual boolean isReady();
  virtual byte *getResult();

  virtual void debugSetValue(byte value);
};

class AnalogSensor : public ISensor {
  int pin;
  byte type = 0;

  byte byteResult = 0;

public:
  AnalogSensor(int pin, byte type) {
    this->pin = pin;
    this->type = type;
    pinMode(pin, INPUT);

    // Let's set 1 to digital out
    // So that the the lamps light
    switch (pin) {
    case A1: {
      pinMode(DIGITAL_PIN_1, OUTPUT);
      break;
    }
    case A2: {
      pinMode(DIGITAL_PIN_2, OUTPUT);
      break;
    }
    case A3: {
      pinMode(DIGITAL_PIN_3, OUTPUT);
      break;
    }
    case A4: {
      pinMode(DIGITAL_PIN_4, OUTPUT);
      break;
    }
    case A5: {
      pinMode(DIGITAL_PIN_5, OUTPUT);
      break;
    }
    }
  };

  void debugSetValue(byte value) { byteResult = value; }

  byte getType() { return type; }

  void reset() { byteResult = 0; }

  void iteration(byte data, byte data2) {
    byteResult = byte(analogRead(pin) >> 2);
  }

  boolean isReady() { return true; }

  byte *getResult() {
    return new byte[SENSOR_RESPONSE_LENGTH]{0, 0, 0, byteResult};
  }
};

class SonicSensor : public ISensor {
  boolean resetMode = true;
  boolean measurementWaiting;
  unsigned long time;
  int result = 0;
  int pin;

  boolean bReady = false;

#define SONIC_FILTER_LENGTH 9
  int history[SONIC_FILTER_LENGTH] = {};

public:
  SonicSensor(int pin) {
    this->pin = pin;

    for (int i = 0; i < SONIC_FILTER_LENGTH; i++) {
      history[i] = 0;
    }
  };

  byte getType() { return SENSOR_TYPE_ULTRASONIC; }

  void reset() {
    measurementWaiting = false;
    resetMode = true;
    bReady = false;
    time = micros();
  }

  void iteration(byte data, byte data2) {
    // Reset mode with delay
    if (resetMode) {
      if (micros() - time > SONIC_RESET_DELAY_US) {
        pinAsOutput(pin);
        digitalWrite(pin, HIGH);
        delayMicroseconds(SONIC_PULSE_WIDTH_US);
        digitalWrite(pin, LOW);
        pinAsInput(pin);
        time = micros();
        resetMode = false;
      }
      return;
    }

    // Timeout - no response received
    if (micros() - time > SONIC_TIMEOUT_US) {
      bReady = true;
      return;
    }

    if (measurementWaiting) {
      if (LOW == digitalState(pin)) {
        // Calculate distance: (time * speed_of_sound) / 2 - offset
        unsigned long pulseTime = micros() - time;
        result = (int)((pulseTime * SONIC_SPEED_OF_SOUND_CM_PER_US) / 2.0f -
                       SONIC_DISTANCE_OFFSET_CM);
        if (result < 0) {
          result = 0;
        }

        // Shift history buffer
        for (int i = 1; i < SONIC_FILTER_LENGTH; i++) {
          history[i - 1] = history[i];
        }
        history[SONIC_FILTER_LENGTH - 1] = result;

        // Find median using partial sort (more efficient than full bubble sort)
        int sorted[SONIC_FILTER_LENGTH];
        for (int i = 0; i < SONIC_FILTER_LENGTH; i++) {
          sorted[i] = history[i];
        }

        // Simple insertion sort for small array (more efficient than bubble
        // sort)
        for (int i = 1; i < SONIC_FILTER_LENGTH; i++) {
          int key = sorted[i];
          int j = i - 1;
          while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
          }
          sorted[j + 1] = key;
        }

        // Use median value (middle element)
        result = sorted[SONIC_FILTER_LENGTH / 2];
        bReady = true;
      }
    } else {
      if (digitalState(pin) == HIGH) {
        measurementWaiting = true;
      }
    }
  }

  void debugSetValue(byte value) {}

  boolean isReady() { return bReady; }

  byte *getResult() {
    return new byte[SENSOR_RESPONSE_LENGTH]{0, 0, 0, (byte)result};
  }
};

class ColorSensor : public ISensor {
  int pin;

  unsigned int INTERVAL_BIT;
  unsigned int INTERVAL_BYTE;
  boolean booleanReady;

  unsigned long timeLowStart = 0;
  unsigned long timeLowEnd = 0;
  unsigned long timeHighStart = 0;
  unsigned long timeHighEnd = 0;

  unsigned long timeSync = 0;
  int INTERVAL_BIT_ALL = 0;
  int INTERVAL_BIT_NEW = 0;
  byte TRIES = 0;
  boolean adaptiveCalibrationEnabled = false;
  byte byteCounter = 0;
  byte bitCounter = 0;
  byte result[3] = {0, 0, 0};
  byte _result[3] = {0, 0, 0};

  byte iSynchronized = 0;

  int iSyncAttempts = 0;

public:
  ColorSensor(int pin) {
    this->pin = pin;
    pinAsInput(pin);
    INTERVAL_BIT = COLOR_INITIAL_INTERVAL_BIT;
    INTERVAL_BYTE = COLOR_INITIAL_INTERVAL_BIT * 8;
  };

  void debugSetValue(byte value) {
    // Debug function - currently unused
  }

  byte getType() { return SENSOR_TYPE_COLOR; };

  void reset() {
    booleanReady = false;
    timeLowStart = 0;
    timeLowEnd = 0;
    timeHighStart = 0;
    timeHighEnd = 0;
    timeSync = 0;
    byteCounter = 0;
    bitCounter = 0;
    _result[0] = 0;
    _result[1] = 0;
    _result[2] = 0;
    iSynchronized = 0;

    // Adaptive calibration of bit interval
    if (TRIES < COLOR_ADAPTIVE_TRIES_MAX && adaptiveCalibrationEnabled) {
      TRIES++;
      if (TRIES > COLOR_ADAPTIVE_TRIES_MIN) {
        INTERVAL_BIT_ALL += INTERVAL_BIT_NEW;
        INTERVAL_BIT =
            round(INTERVAL_BIT_ALL / (TRIES - COLOR_ADAPTIVE_TRIES_MIN));
        INTERVAL_BYTE = INTERVAL_BIT * 8;
      }
    } else {
      adaptiveCalibrationEnabled = true;
    }
  }

  void iteration(byte data, byte data2) {
    switch (iSynchronized) {
    case 0: {
      if (digitalState(pin) == HIGH) {
        iSynchronized = 1;
        timeLowStart = micros();
      } else {
        iSynchronized = 0;
      }
      break;
    }
    case 1: {
      if (digitalState(pin) == LOW) {
        timeLowEnd = micros();
        if (timeLowEnd - timeLowStart >= COLOR_SYNC_THRESHOLD_US) {
          iSynchronized = 2;
        }
      } else {
        iSynchronized = 0;
      }
      break;
    }
    case 2: {
      if (digitalState(pin) == HIGH) {
        timeHighStart = micros();
        iSynchronized = 3;
      } else {
        timeLowEnd = micros();
      }
      break;
    }
    case 3: {
      INTERVAL_BIT_NEW = round((timeLowEnd - timeLowStart) / COLOR_BIT_DIVISOR);
      timeSync = timeLowEnd + INTERVAL_BIT + 50;
      iSynchronized = 4;
    }
    case 4: {
      if (timeSync <= micros()) {
        timeSync += INTERVAL_BIT;
        if (digitalState(pin) == HIGH) {
          _result[byteCounter] |= (1 << bitCounter);
        } else {
          _result[byteCounter] &= ~(1 << bitCounter);
        }
        bitCounter++;
        if (bitCounter > 7) {
          bitCounter = 0;
          byteCounter++;
          if (byteCounter > 2) {
            // Reading complete - validate and update result
            byteCounter = 0;
            iSynchronized = 0;

            // Filter out readings with large single-channel changes (likely
            // noise)
            bool hasLargeChange =
                (abs(result[0] - _result[0]) >= COLOR_CHANGE_THRESHOLD &&
                 abs(result[1] - _result[1]) < COLOR_STABILITY_THRESHOLD &&
                 abs(result[2] - _result[2]) < COLOR_STABILITY_THRESHOLD) ||
                (abs(result[0] - _result[0]) < COLOR_STABILITY_THRESHOLD &&
                 abs(result[1] - _result[1]) >= COLOR_CHANGE_THRESHOLD &&
                 abs(result[2] - _result[2]) < COLOR_STABILITY_THRESHOLD) ||
                (abs(result[0] - _result[0]) < COLOR_STABILITY_THRESHOLD &&
                 abs(result[1] - _result[1]) < COLOR_STABILITY_THRESHOLD &&
                 abs(result[2] - _result[2]) >= COLOR_CHANGE_THRESHOLD);

            if (!hasLargeChange) {
              result[0] = _result[0];
              result[1] = _result[1];
              result[2] = _result[2];
            }
            booleanReady = true;
          }
        }
      }
      break;
    }
    }
  }

  boolean isReady() { return booleanReady; }

  byte *getResult() {
    return new byte[SENSOR_RESPONSE_LENGTH]{0, result[0], result[1], result[2]};
  }
};

ISensor *sensors[5];

#ifdef TESTMODE_COLOR_SENSOR
boolean testModeToggle = false;
#endif

void calibrating() {
  // Read encoders and calculate calibration coefficients
  delay(CALIBRATION_DELAY_MS);

  while (encoderDelta > 0) {
    delay(CALIBRATION_CHECK_INTERVAL_MS);
    encoderDelta = leftMotor.stepsPath - lastEncoderLeft +
                   rightMotor.stepsPath - lastEncoderRight;
    lastEncoderLeft = leftMotor.stepsPath;
    lastEncoderRight = rightMotor.stepsPath;
  }

  if (leftMotor.stepsPath > rightMotor.stepsPath) {
    calibrationLeftToRight = 1.0f;
    calibrationRightToLeft =
        (float)rightMotor.stepsPath / (float)leftMotor.stepsPath;
    EEPROM.put(ADDRCALIB, calibrationRightToLeft);
  } else {
    calibrationRightToLeft = 1.0f;
    calibrationLeftToRight =
        (float)leftMotor.stepsPath / (float)rightMotor.stepsPath;
    EEPROM.put(ADDRCALIB, calibrationLeftToRight * (-1));
  }
}

void setup() {
  // Let's load S/N
  parseSerialNumber();

  Serial.begin(SERIAL_SPEED);

  // Let's set the PWR timer
  bitSet(TCCR1B, WGM12);

  myservo.attach(13);
  myservo.write(500);

  commandState = COMMAND_STATE_WAITING_COMMAND;

  leftMotor.initMotor(10, 9);
  rightMotor.initMotor(6, 5);
  // Явно останавливаем после инициализации, чтобы исключить любой стартовый
  // рывок
  leftMotor.stop();
  rightMotor.stop();

  attachInterrupt(1, onLeftMotorStep, CHANGE);
  attachInterrupt(0, onRightMotorStep, CHANGE);

  m_lastInput = 0;
  m_lastResult = 0;

  EEPROM.get(BLUE_CHECK, blue_num);
  if (!blue_num) {
    Serial.write("$$$");
    delay(100);
    char a[3];
    a[0] = Serial.read();
    a[1] = Serial.read();
    a[2] = Serial.read();
    a[2] = Serial.read();
    delay(1000);
    if (a[0] == 'C') {
      Serial.println("SQ,16");
      delay(100);
      a[0] = Serial.read();
      a[1] = Serial.read();
      a[2] = Serial.read();
      a[2] = Serial.read();
      delay(100);
      if (a[1] == 'A') {
        digitalWrite(10, HIGH);
        delay(100);
        digitalWrite(10, LOW);
        EEPROM.put(BLUE_CHECK,
                   '1'); //   <----------------------------- У EEPROM
                         //   ограничеено количество циклов записи, сначала
                         //   думай, потом раскомменчивай!
        Serial.println("---");
      } else {
        digitalWrite(9, HIGH);
        delay(2000);
        digitalWrite(9, LOW);
      }
    }
  }

  sensors[0] = new AnalogSensor(A1, SENSOR_TYPE_LINE);
  sensors[1] = new AnalogSensor(A2, SENSOR_TYPE_LINE);
  sensors[2] = new AnalogSensor(A3, SENSOR_TYPE_LINE);
  sensors[3] = new AnalogSensor(A4, SENSOR_TYPE_LINE);
  sensors[4] = new AnalogSensor(A5, SENSOR_TYPE_LINE);
}

void printSensors() {
  Serial.write('#');
  Serial.write((byte)((leftMotor.stepsCnt >> 8) & 0xff));
  Serial.write((byte)((leftMotor.stepsCnt) & 0xff));
  Serial.write((byte)((rightMotor.stepsCnt >> 8) & 0xff));
  Serial.write((byte)((rightMotor.stepsCnt) & 0xff));

  Serial.write((byte)((leftMotor.stepsPath >> 8) & 0xff));
  Serial.write((byte)((leftMotor.stepsPath) & 0xff));
  Serial.write((byte)((rightMotor.stepsPath >> 8) & 0xff));
  Serial.write((byte)((rightMotor.stepsPath) & 0xff));

  for (int i = 0; i < SENSOR_COUNT; i++) {
    byte *result = sensors[i]->getResult();
    Serial.write(result, SENSOR_RESPONSE_LENGTH);
    delete[] result;
  }

  byte sensorValue = (byte)(analogRead(A0) >> 2);
  Serial.write(sensorValue);

  /*
    //Checking the UltraSonic progress
    for (int i = 0; i < SENSOR_COUNT; i++) {
      byte byteType = sensors[i] -> getType();
      if (byteType == SENSOR_TYPE_ULTRASONIC && pendingSensors[i] && sensors[i]
    -> isReady()) { sensors[i] -> reset(); sensors[i] -> iteration(0, 0);
      }
    }
  */

  // analog
  for (int i = 0; i < SENSOR_COUNT;
       i++) { // modified_by_yaroslav //for PID test purpose //i=0
    byte byteType = sensors[i]->getType();
    if (byteType != SENSOR_TYPE_ULTRASONIC && byteType != SENSOR_TYPE_COLOR) {
      sensors[i]->reset();
      sensors[i]->iteration(0, 0);
    }
  }

  // sensors[0] -> reset();
  // while(!sensors[0] -> isReady()){
  //    sensors[0] -> iteration(0, 2);
  // }
  // sensors[1] -> reset();
  // while(!sensors[1] -> isReady()){
  //    sensors[1] -> iteration(1, 2);
  // }

  // color
  byte tmp = 0;
  byte totalColor = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    byte byteType = sensors[i]->getType();
    if (byteType == SENSOR_TYPE_COLOR) {
      if (totalColor == byteActiveColorSensor) {
        sensors[i]->reset();
      }
      totalColor++;
    }
  }

  tmp = 0;
  unsigned long timeLimit = micros();

  while (totalColor > 0) {
    boolean allDone = true;

    for (int i = 0; i < SENSOR_COUNT; i++) {
      byte byteType = sensors[i]->getType();
      if (byteType == SENSOR_TYPE_COLOR) {
        if (tmp < byteActiveColorSensor) {
          tmp++;
          continue;
        }

        unsigned long colorLoopStart = micros();
        while (!sensors[i]->isReady()) {
          sensors[i]->iteration(i, totalColor);

          if ((micros() - colorLoopStart) > COLOR_LOOP_TIMEOUT_US) {
            break;
          }
        }
        break;
      }
    }

    if (allDone || abs(micros() - timeLimit) > COLOR_LOOP_TIMEOUT_US)
      break;
  }
#ifdef TESTMODE_COLOR_SENSOR
  digitalLow(11);
#endif

  byteActiveColorSensor++;
  if (byteActiveColorSensor == totalColor) {
    byteActiveColorSensor = 0;
  }

  // Ultrasonic sensors
  for (int i = 0; i < SENSOR_COUNT; i++) {
    byte byteType = sensors[i]->getType();
    if (byteType == SENSOR_TYPE_ULTRASONIC) {
      if (sensors[i]->isReady()) {
        sensors[i]->reset();
      }
    }
  }
}

void printSensorDebug() {
  Serial.print("\nL:[steps=" + String(leftMotor.stepsCnt) +
               " limit=" + String(leftMotor.stepsLimit) +
               " path=" + String(leftMotor.stepsPath) +
               " distance=" + String(leftMotor.stepsDistance) + "]");
  Serial.print("\nR:[stepsR=" + String(rightMotor.stepsCnt) +
               " limit=" + String(rightMotor.stepsLimit) +
               " path=" + String(rightMotor.stepsPath) +
               " distance=" + String(rightMotor.stepsDistance) + "]");
}

byte bytearrayData[20];
byte byteDataTail = 0;
byte command = 0;

unsigned long lastReceivedCommandTime = millis();

void loop() {
  if (millis() - lastReceivedCommandTime > CONNECTION_LOST_TIME_INTERVAL) {
    leftMotor.stop();
    rightMotor.stop();
  }

  if (Serial.available()) {

    byte b = Serial.read();
    lastReceivedCommandTime = millis();

    if (commandState == COMMAND_STATE_WAITING_COMMAND) {
      switch (b) {
      case ' ': {
        Serial.print(F("ROBBO-"));
        if (MODEL_ID < 10000) {
          Serial.write('0');
        }
        if (MODEL_ID < 1000) {
          Serial.write('0');
        }
        if (MODEL_ID < 100) {
          Serial.write('0');
        }
        if (MODEL_ID < 10) {
          Serial.write('0');
        }
        Serial.print(MODEL_ID);

        Serial.write('-');
        Serial.print(F(FIRMWARE_VERSION));

        Serial.write('-');
        Serial.print(chararrModel);

        Serial.print('-');
        for (int f = strlen(chararrVersion); f < 5; f++) {
          Serial.write('0');
        }
        Serial.print(chararrVersion);

        Serial.print('-');
        for (int f = strlen(chararrPart); f < 5; f++) {
          Serial.write('0');
        }
        Serial.print(chararrPart);

        Serial.print('-');
        for (int f = strlen(chararrSerial); f < 20; f++) {
          Serial.write('0');
        }
        Serial.print(chararrSerial);

        break;
      }
      case '!': {
        Serial.print(chararrSerialRaw);

        break;
      }
      case 'a': {
        command = b;
        commandState = COMMAND_STATE_WAITING_CRC;
        break;
      }
      case 'b': {
        command = b;
        commandState = COMMAND_STATE_WAITING_CRC;
        break;
      }
      case 'c': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'd': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'e': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'f': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'g': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'h': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'i': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      case 'j': {
        commandState = COMMAND_STATE_WAITING_DATA;
        byteDataTail = 0;
        command = b;
        break;
      }
      }
    } else if (commandState == COMMAND_STATE_WAITING_DATA) {
      bytearrayData[byteDataTail] = b;
      byteDataTail++;

      switch (command) {
      case 'c': {
        if (byteDataTail > 1) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'd': {
        if (byteDataTail > 1) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'e': {
        if (byteDataTail > 1) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'f': {
        if (byteDataTail > 0) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'g': {
        if (byteDataTail > 3) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'h': {
        if (byteDataTail > 0) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'i': {
        if (byteDataTail > 4) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      case 'j': {
        if (byteDataTail > 0) {
          commandState = COMMAND_STATE_WAITING_CRC;
        }
        break;
      }
      }
    } else if (commandState == COMMAND_STATE_WAITING_CRC) {

      if (b == '$') {
        switch (command) {
        case 'a': {
          printSensors();
          break;
        }
        case 'b': {
          printSensorDebug();
          break;
        }
        case 'c': {
          byte leftSpeed = bytearrayData[0];
          byte rightSpeed = bytearrayData[1];
          motorControlFlag = 0;
          byte leftDir = DIRECTION_FORWARD;
          byte rightDir = DIRECTION_FORWARD;

          if (leftSpeed >= DIRECTION_BIT_MASK) {
            leftDir = DIRECTION_BACKWARD;
            leftSpeed -= DIRECTION_BIT_MASK;
          }
          if (rightSpeed >= DIRECTION_BIT_MASK) {
            rightDir = DIRECTION_BACKWARD;
            rightSpeed -= DIRECTION_BIT_MASK;
          }

          globalLeftMotorSpeed = leftSpeed;
          globalRightMotorSpeed = rightSpeed;
          globalLeftDir = leftDir;
          globalRightDir = rightDir;

          leftMotor.setSpeedAndDirection(leftSpeed, leftDir);
          rightMotor.setSpeedAndDirection(rightSpeed, rightDir);
          printSensors();
          break;
        }
        case 'd': {
          byte leftSpeed = bytearrayData[0] - '0';
          byte rightSpeed = bytearrayData[1] - '0';

          byte leftDir = DIRECTION_FORWARD;
          byte rightDir = DIRECTION_FORWARD;

          if (leftSpeed >= DIRECTION_BIT_MASK) {
            leftDir = DIRECTION_BACKWARD;
            leftSpeed -= DIRECTION_BIT_MASK;
          }
          if (rightSpeed >= DIRECTION_BIT_MASK) {
            rightDir = DIRECTION_BACKWARD;
            rightSpeed -= DIRECTION_BIT_MASK;
          }

          leftMotor.setSpeedAndDirection(leftSpeed, leftDir);
          rightMotor.setSpeedAndDirection(rightSpeed, rightDir);

          printSensorDebug();
          break;
        }
        case 'e': {
          byte stepsHigh = bytearrayData[0];
          byte stepsLow = bytearrayData[1];

          unsigned int iStepsLimit = stepsHigh;
          iStepsLimit = iStepsLimit << 8;
          iStepsLimit += stepsLow;

          leftMotor.enableAndSetStepsLimit(iStepsLimit);
          rightMotor.enableAndSetStepsLimit(iStepsLimit);

          printSensors();
          break;
        }
        case 'f': {
          byte steps = bytearrayData[0] - '0';

          leftMotor.enableAndSetStepsLimit(steps);
          rightMotor.enableAndSetStepsLimit(steps);

          printSensorDebug();
          break;
        }
        case 'g': {
          byte leftSpeed = bytearrayData[0];
          byte rightSpeed = bytearrayData[1];
          byte stepsHigh = bytearrayData[2];
          byte stepsLow = bytearrayData[3];
          byte leftDir = DIRECTION_FORWARD;
          byte rightDir = DIRECTION_FORWARD;
          motorControlFlag = 0;

          if (leftSpeed >= DIRECTION_BIT_MASK) {
            leftDir = DIRECTION_BACKWARD;
            leftSpeed -= DIRECTION_BIT_MASK;
          }
          if (rightSpeed >= DIRECTION_BIT_MASK) {
            rightDir = DIRECTION_BACKWARD;
            rightSpeed -= DIRECTION_BIT_MASK;
          }

          unsigned int iStepsLimit = stepsHigh;
          iStepsLimit = iStepsLimit << 8;
          iStepsLimit += stepsLow;

          globalLeftDir = leftDir;
          globalRightDir = rightDir;
          globalLeftMotorSpeed = leftSpeed;
          globalRightMotorSpeed = rightSpeed;

          leftMotor.setSpeedAndDirection(leftSpeed, leftDir);
          rightMotor.setSpeedAndDirection(rightSpeed, rightDir);
          leftMotor.enableAndSetStepsLimit(iStepsLimit);
          rightMotor.enableAndSetStepsLimit(iStepsLimit);

          printSensors();
          break;
        }
        case 'h': {
          byte lamps = bytearrayData[0];
          if (lamps & 1) {
            digitalWrite(4, HIGH);
          } else {
            digitalWrite(4, LOW);
          }

          if (lamps & 2) {
            digitalWrite(7, HIGH);
          } else {
            digitalWrite(7, LOW);
          }

          if (lamps & 4) {
            digitalWrite(8, HIGH);
          } else {
            digitalWrite(8, LOW);
          }

          if (lamps & 8) {
            digitalWrite(11, HIGH);
          } else {
            digitalWrite(11, LOW);
          }

          if (lamps & 16) {
            digitalWrite(12, HIGH);
          } else {
            digitalWrite(12, LOW);
          }

          printSensors();
          break;
        }
        case 'i': {
          // IMPORTANT: Color sensor count can be changed!
          byteActiveColorSensor = 0;

          // Pin mappings for analog and digital sensors
          const int analogPins[SENSOR_COUNT] = {A1, A2, A3, A4, A5};
          const int digitalPins[SENSOR_COUNT] = {DIGITAL_PIN_1, DIGITAL_PIN_2,
                                                 DIGITAL_PIN_3, DIGITAL_PIN_4,
                                                 DIGITAL_PIN_5};

          for (int f = 0; f < SENSOR_COUNT; f++) {
            delete sensors[f];

            switch (bytearrayData[f]) {
            case SENSOR_TYPE_NONE:
            case SENSOR_TYPE_LINE:
            case SENSOR_TYPE_LED:
            case SENSOR_TYPE_LIGHT:
            case SENSOR_TYPE_TOUCH:
            case SENSOR_TYPE_PROXIMITY: {
              sensors[f] = new AnalogSensor(analogPins[f], bytearrayData[f]);
              break;
            }
            case SENSOR_TYPE_ULTRASONIC: {
              sensors[f] = new SonicSensor(digitalPins[f]);
              break;
            }
            case SENSOR_TYPE_COLOR: {
              sensors[f] = new ColorSensor(digitalPins[f]);
              break;
            }
            }
          }

          printSensors();
          break;
        }
        case 'j': {
          myservo.write(bytearrayData[0] * 5 + 500);

          printSensors();
          break;
        }
        }
      }
      commandState = COMMAND_STATE_WAITING_COMMAND;
    }
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    byte byteType = sensors[i]->getType();
    if (byteType == SENSOR_TYPE_ULTRASONIC) {
      if (sensors[i]->isReady()) {
      } else {
        sensors[i]->iteration(0, 0);
      }
    }
  }

#ifdef TESTMODE_COLOR_SENSOR
  pinMode(11, OUTPUT);
  if (testModeToggle) {
    digitalHigh(11);
    testModeToggle = false;
  } else {
    digitalLow(11);
    testModeToggle = true;
  }
#endif

  // Smooth PWM update for motors (soft start/braking)
  leftMotor.updatePwm();
  rightMotor.updatePwm();
}
