//#include "HC_SR04.h"
#include "Arduino.h"
#include <Servo.h>

#define SERIAL_SPEED 38400
#define SERIAL_ADDRESS 0
#define data 2
#define clock 4

#define ECHO_INT 0

// ===== Октава 3 =====
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247

// ===== Октава 4 =====
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494

// ===== Октава 5 =====
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 830
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988

// ===== Октава 6 =====
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976


// Robot pinout
#define PIN_BLUE_0     0
#define PIN_BLUE_1     1

#define PIN_R          3
#define PIN_G          5
#define PIN_B          6

#define PIN_SONIC_ECHO 2
#define PIN_SONIC_TRIG 9

#define PIN_MATRIX_CS  10
#define PIN_MATRIX_CLK 11
#define PIN_MATRIX_DIN 12

#define PIN_BUZZER     13

#define PIN_LEFT_FOOT   14
#define PIN_RIGHT_FOOT  15
#define PIN_LEFT_LEG  16
#define PIN_RIGHT_LEG 17
#define PIN_LEFT_HAND  6
#define PIN_RIGHT_HAND 7

#define PIN_MICROPHONE  A7

#define TRIM_LEFT_LEG   0
#define TRIM_RIGHT_LEG  0
#define TRIM_LEFT_FOOT  0
#define TRIM_RIGHT_FOOT 0


bool musicPlaying = false;
unsigned long noteStartTime = 0;
int noteIndex = 0;
int melody[] = {
  // Начало: G G G Eb Bb G Eb Bb G (up above tuning C)
  NOTE_G4, NOTE_G4, NOTE_G4, NOTE_DS4, NOTE_B3, NOTE_G3, NOTE_DS4, NOTE_B3, NOTE_G5,
  
  // D D D Eb (back down) Bb G Eb Bb G
  NOTE_D4, NOTE_D4, NOTE_D4, NOTE_DS4, NOTE_B3, NOTE_G3, NOTE_DS4, NOTE_B3, NOTE_G4,
  
  // G G low GG high G F# F E Eb E Ab C# C B Bb A Bb
  NOTE_G3, NOTE_G3, NOTE_G3, NOTE_G4, NOTE_G3, NOTE_FS4, NOTE_F4, NOTE_E4, NOTE_DS4,
  NOTE_E4, NOTE_AS4, NOTE_CS5, NOTE_C5, NOTE_B4, NOTE_AS4, NOTE_A4, NOTE_AS4,
  
  // Eb F# Bb G Bb D G G G F# F E Eb E Ab C# C B Bb A Bb Eb Bb G
  NOTE_DS4, NOTE_FS4, NOTE_B4, NOTE_G4, NOTE_B4, NOTE_D5, NOTE_G4, NOTE_G4, NOTE_G4,
  NOTE_FS4, NOTE_F4, NOTE_E4, NOTE_DS4, NOTE_E4, NOTE_AS4, NOTE_CS5, NOTE_C5, NOTE_B4,
  NOTE_AS4, NOTE_A4, NOTE_AS4, NOTE_DS4, NOTE_B4, NOTE_G4
};

int noteDurations[] = {
  300, 300, 300, 450, 150, 300, 450, 150,
  600, 300, 300, 300, 450, 150, 300, 450,
  150, 600, 300, 450, 150, 300, 450, 150,
  150, 150, 150, 150, 300, 150, 150, 150,
  150, 150, 150, 450, 150, 150, 300, 150,
  150, 300, 150, 150, 300, 150, 150, 300,
  150, 150, 300, 150, 150, 300, 150, 150,
  300
};
// ===== Функция обновления музыки (вызывается в loop) =====
void updateMusic() {
  if (!musicPlaying) return;

  if (millis() - noteStartTime >= noteDurations[noteIndex]) {
    noteIndex++;
    if (noteIndex >= sizeof(melody) / sizeof(melody[0])) {
      noteIndex = 0; // Сбрасываем на первую ноту
    }
    tone(PIN_BUZZER, melody[noteIndex], noteDurations[noteIndex]);
    noteStartTime = millis();
  }
}

void startMusic() {
  musicPlaying = true;
  noteIndex = 0;
  noteStartTime = millis();
  tone(PIN_BUZZER, melody[noteIndex], noteDurations[noteIndex]);
}

// sidewave variables
bool sidewave_active = false;
unsigned long sidewave_timer = 0;
int sidewave_step = 0;
int sidewave_total_steps = 0;
float sidewave_phase = 0;
float sidewave_speed = 0;
int sidewave_height = 0;
int sidewave_dir = 0;

bool bend_active = false;
unsigned long bend_timer = 0;
int bend_step = 0;
int bend_total_steps = 0;
float bend_phase = 0;
float bend_speed = 0;
int bend_height = 0;
int bend_dir = 0;

bool dance3_active = false;
unsigned long dance3_timer = 0;
int dance3_step = 0;
int dance3_total_steps = 0;
float dance3_phase = 0;
float dance3_speed = 0;
int dance3_height = 0;
int dance3_dir = 0;

bool walking_active = false;
unsigned long walking_timer = 0;
float walking_phase = 0;
float walking_speed = 1.0;

int stepWalking = 0;
unsigned long stepWalkingTimer = 0;

int r = 0, g = 0, b = 0;
int dirR = 1, dirG = 1, dirB = 1;
unsigned long lastUpdate = 0;
int updateDelay = 20; // Задержка между сменой цветов (мс)
bool colourswap = 0;

//int dist=0;
//HC_SR04 sensor(PIN_SONIC_TRIG, PIN_SONIC_ECHO, ECHO_INT);
//int getDist(){
//  if(sensor.isFinished()){
//    dist = sensor.getRange();
//    sensor.start();
//  }
//  return dist;
//}

unsigned char i;
unsigned char j;
unsigned char now_led[8]={0,0,0,0,0,0,0,0};

Servo HandL;
Servo HandR;

Servo myservo [6];
int pos[6];
int need_pos[6];
unsigned long times[6];
short turn_speed = 20;

unsigned char disp1[50][8] = {
0x0,  0x7C,  0xFE,  0x82,  0x82,  0xFE,  0x7C,  0x0, //0
0x0,  0x0,  0x2,  0xFE,  0xFE,  0x42,  0x0,  0x0, //1
0x0,  0x62,  0xF2,  0x92,  0x9A,  0xCE,  0x46,  0x0,//2
0x0,  0x6C,  0xFE,  0x92,  0x92,  0xC6,  0x44,  0x0,//3
0x8,  0xFE,  0xFE,  0xC8,  0x68,  0x38,  0x18,  0x0,//4
0x0,  0x9C,  0xBE,  0xA2,  0xA2,  0xE6,  0xE4,  0x0,//5
  0x30, 0x48, 0x44, 0x22, 0x22, 0x44, 0x48, 0x30, // рисунок сердца
  0x00,0x10,0x18,0x14,0x14,0x18,0x10,0x00,         //узкая улыбка
  0x10,0x18,0x14,0x14,0x14,0x14,0x18,0x10,         //широкая улыбка
  0x00,0x10,0x28,0x44,0x44,0x28,0x10,0x00,         //буква 0
  0x0,  0x7C,  0xFE,  0x82,  0x82,  0xFE,  0x7C,  0x0, //0
0x0,  0x0,  0x2,  0xFE,  0xFE,  0x42,  0x0,  0x0, //1
0x0,  0x62,  0xF2,  0x92,  0x9A,  0xCE,  0x46,  0x0,//2
0x0,  0x6C,  0xFE,  0x92,  0x92,  0xC6,  0x44,  0x0,//3
0x8,  0xFE,  0xFE,  0xC8,  0x68,  0x38,  0x18,  0x0,//4
0x0,  0x9C,  0xBE,  0xA2,  0xA2,  0xE6,  0xE4,  0x0,//5
  0x30, 0x48, 0x44, 0x22, 0x22, 0x44, 0x48, 0x30, // рисунок сердца
  0x00,0x10,0x18,0x14,0x14,0x18,0x10,0x00,         //узкая улыбка
  0x10,0x18,0x14,0x14,0x14,0x14,0x18,0x10,         //широкая улыбка
  0x00,0x10,0x28,0x44,0x44,0x28,0x10,0x00,         //буква 0
  0x0,  0x7C,  0xFE,  0x82,  0x82,  0xFE,  0x7C,  0x0, //0
0x0,  0x0,  0x2,  0xFE,  0xFE,  0x42,  0x0,  0x0, //1
0x0,  0x62,  0xF2,  0x92,  0x9A,  0xCE,  0x46,  0x0,//2
0x0,  0x6C,  0xFE,  0x92,  0x92,  0xC6,  0x44,  0x0,//3
0x8,  0xFE,  0xFE,  0xC8,  0x68,  0x38,  0x18,  0x0,//4
0x0,  0x9C,  0xBE,  0xA2,  0xA2,  0xE6,  0xE4,  0x0,//5
  0x30, 0x48, 0x44, 0x22, 0x22, 0x44, 0x48, 0x30, // рисунок сердца
  0x00,0x10,0x18,0x14,0x14,0x18,0x10,0x00,         //узкая улыбка
  0x10,0x18,0x14,0x14,0x14,0x14,0x18,0x10,         //широкая улыбка
  0x00,0x10,0x28,0x44,0x44,0x28,0x10,0x00,         //буква 0
  0x0,  0x0,  0x2,  0xFE,  0xFE,  0x42,  0x0,  0x0, //1
0x0,  0x62,  0xF2,  0x92,  0x9A,  0xCE,  0x46,  0x0,//2
0x0,  0x6C,  0xFE,  0x92,  0x92,  0xC6,  0x44,  0x0,//3
0x8,  0xFE,  0xFE,  0xC8,  0x68,  0x38,  0x18,  0x0,//4
0x0,  0x9C,  0xBE,  0xA2,  0xA2,  0xE6,  0xE4,  0x0,//5
  0x30, 0x48, 0x44, 0x22, 0x22, 0x44, 0x48, 0x30, // рисунок сердца
  0x00,0x10,0x18,0x14,0x14,0x18,0x10,0x00,         //узкая улыбка
  0x10,0x18,0x14,0x14,0x14,0x14,0x18,0x10,         //широкая улыбка
  0x00,0x10,0x28,0x44,0x44,0x28,0x10,0x00,         //буква 0
};

void Init_MAX7219(void) {
  Write_Max7219(0x09, 0x00);
  Write_Max7219(0x0a, 0x03);
  Write_Max7219(0x0b, 0x07);
  Write_Max7219(0x0c, 0x01);
  Write_Max7219(0x0f, 0x00);
}

void Write_Max7219_byte(unsigned char DATA) {
  unsigned char i;
  digitalWrite(PIN_MATRIX_CS, LOW);
  for (i = 8; i >= 1; i--) {
    digitalWrite(PIN_MATRIX_CLK, LOW);
    digitalWrite(PIN_MATRIX_DIN, DATA & 0x80);
    DATA = DATA << 1;
    digitalWrite(PIN_MATRIX_CLK, HIGH);
  }
}

void Write_Max7219(unsigned char address, unsigned char dat) {
  digitalWrite(PIN_MATRIX_CS, LOW);
  Write_Max7219_byte(address);
  Write_Max7219_byte(dat);
  digitalWrite(PIN_MATRIX_CS, HIGH);
}

void cleaR() {
    for (i = 1; i < 9; i++)
    Write_Max7219(i, 0x00);
}

void updatE() {
    for (i = 1; i < 9; i++)
    Write_Max7219(i, now_led[i-1]);
}

void sidewave_start(int period, int height, int dir) {
  sidewave_active = true;
  sidewave_height = height;
  sidewave_dir = dir;
  sidewave_phase = 0;
  sidewave_speed = 6.28 / (period / 100.0); // 2*PI / (T/100)
  sidewave_timer = millis();
}

void sidewave_update() {
  if (!sidewave_active) return;
  
  // Обновляем каждые 50 мс
  if (millis() - sidewave_timer < 50) return;
  sidewave_timer = millis();
  
  // Вычисляем позиции для ног
  float phase1 = sidewave_phase + (-sidewave_dir * 1.57); // -90° в радианах
  float phase2 = sidewave_phase + ((-60 - 90 * sidewave_dir) * 0.01745);
  
  int leg_left = 90 + (sidewave_height/2 + 2) + sidewave_height * sin(phase1);
  int leg_right = 90 - (sidewave_height/2 + 2) + sidewave_height * sin(phase2);
  
  // Ограничиваем значения
  leg_left = constrain(leg_left, 30, 150);
  leg_right = constrain(leg_right, 30, 150);
  
  // Устанавливаем целевые позиции
  need_pos[0] = leg_left;
  need_pos[1] = leg_right;
  need_pos[2] = 90;   // LEFT_LEG
  need_pos[3] = 90;  // RIGHT_LEG
  
  // Руки для баланса
  need_pos[4] = 90 + sidewave_height * sin(sidewave_phase);  // LEFT_HAND
  need_pos[5] = 90 - sidewave_height * sin(sidewave_phase);  // RIGHT_HAND
  
  // Обновляем фазу
  sidewave_phase += 0.5*sidewave_speed;
}

void bend_start(int period, int height, int dir) {
  bend_active = true;
  bend_height = height;
  bend_dir = dir;
  bend_phase = 0;
  bend_speed = 6.28 / (period / 100.0); // 2*PI / (T/100)
  bend_timer = millis();
}

void bend_update() {
  if (!bend_active) return;

  if (millis() - bend_timer < 50) return;
  bend_timer = millis();

  float s    = sin(bend_phase);
  float s_op = sin(bend_phase + PI);

  // ---- НОГИ (16, 17) ----
  int leg_left  = 90 + bend_dir * 8 * s;
  int leg_right = 90 - bend_dir * 8 * s;

  // ---- СТУПНИ (14, 15) — ОЧЕНЬ БОЛЬШОЙ УГОЛ ----
  float lift_left  = max(0, s);
  float lift_right = max(0, s_op);

  // Резкость
  lift_left  = lift_left * lift_left * lift_left;
  lift_right = lift_right * lift_right * lift_right;

  // Амплитуда ×2.5
  int foot_left  = 90 + 10.0 * bend_height * lift_left;
  int foot_right = 90 + 10.0 * bend_height * lift_right;

  // Ограничения
  leg_left   = constrain(leg_left,  60, 120);
  leg_right  = constrain(leg_right, 60, 120);
  foot_left  = constrain(foot_left,  0, 180);
  foot_right = constrain(foot_right, 0, 180);

  // ---- need_pos ----
  need_pos[0] = leg_left;
  need_pos[1] = leg_right;
  need_pos[2] = foot_left;
  need_pos[3] = foot_right;

  float arm_lift_left  = max(0, s);
  float arm_lift_right = max(0, s_op);

  // Резкость
  arm_lift_left  = arm_lift_left * arm_lift_left * arm_lift_left;
  arm_lift_right = arm_lift_right * arm_lift_right * arm_lift_right;

  // ---- РУКИ (6, 7) ----
  // Почти крайние положения
  need_pos[4] = 20  + 140 * arm_lift_left;   // LEFT_HAND (pin 6)
  need_pos[5] = 20  + 140 * arm_lift_right;  // RIGHT_HAND (pin 7)

  bend_phase += bend_speed;
}


void dance3_start(int period, int height, int dir) {
  dance3_active = true;
  dance3_height = height;
  dance3_dir = dir;
  dance3_phase = 0;
  dance3_speed = 6.28 / (period / 100.0); // 2*PI / (T/100)
  dance3_timer = millis();
}
void dance3_update() {
  if (!dance3_active) return;

  unsigned long now = millis();
  if (now - dance3_timer < 40) return; // таймер обновления
  dance3_timer = now;

  // ========= Шаги танца =========
  static int step = 0;
  static unsigned long stepStart = 0;
  const unsigned long stepInterval = 500; // длительность такта
  if (stepStart == 0) stepStart = now;

  float progress = float(now - stepStart) / stepInterval; // 0..1
  if (progress >= 1.0) {
    step = (step + 1) % 8;
    stepStart = now;
    progress = 0;
  }

  // ========= Заданные позиции по шагам =========
  float s_target = 0, s_op_target = 0, c_target = 0;
  switch(step) {
    case 0: s_target=0.5; s_op_target=-0.5; c_target=0; break;
    case 1: s_target=0.7; s_op_target=-0.7; c_target=0.3; break;
    case 2: s_target=1.0; s_op_target=-1.0; c_target=0.5; break;
    case 3: s_target=-0.6; s_op_target=0.6; c_target=-0.3; break;
    case 4: s_target=0.8; s_op_target=-0.8; c_target=0.8; break;
    case 5: s_target=1.0; s_op_target=-1.0; c_target=0; break;
    case 6: s_target=0.3; s_op_target=-0.3; c_target=0; break;
    case 7: s_target=0; s_op_target=0; c_target=0; break;
  }

  // ========= Плавная интерполяция =========
  static float s_current = 0, s_op_current = 0, c_current = 0;
  s_current    += (s_target - s_current) * 0.2;
  s_op_current += (s_op_target - s_op_current) * 0.2;
  c_current    += (c_target - c_current) * 0.2;

  // ========= Ноги ==========
  int leg_left   = constrain(90 + 40 * s_current, 30, 150);
  int leg_right  = constrain(90 - 40 * s_op_current, 30, 150);
  int foot_left  = constrain(90 + 45 * max(0, s_current), 0, 180);
  int foot_right = constrain(90 + 45 * max(0, s_op_current), 0, 180);

  need_pos[0] = leg_left;
  need_pos[1] = leg_right;
  need_pos[2] = foot_left;
  need_pos[3] = foot_right;

  // ========= Руки ==========
  int hand_left  = constrain(20 + 160 * pow(max(0, s_op_current), 3) + 10 * c_current, 0, 180);
  int hand_right = constrain(20 + 160 * pow(max(0, s_current), 3) - 10 * c_current, 0, 180);

  need_pos[4] = hand_left;
  need_pos[5] = hand_right;

  // ========= Фаза ==========
  dance3_phase += dance3_speed * (0.85 + 0.15 * abs(c_current));
}

void walking_start() {
  walking_active = true;       // включаем ходьбу
  walking_timer  = millis();   // сброс таймера обновления
  walking_phase  = 0;          // сброс фазы
  stepWalking    = 0;          // сброс шага
  stepWalkingTimer = millis(); // сброс таймера шага
}
void walking_update() {
    if (!walking_active) return;

    // Ограничение частоты обновления
    if (millis() - walking_timer < 40) return;
    walking_timer = millis();

    // ===== Шаги ходьбы по фазе =====
    static float phase = 0; // общая фаза шага
    const float phase_increment = 0.2 * walking_speed; // скорость ходьбы

    phase += phase_increment;
    if (phase > 2 * PI) phase -= 2 * PI; // зацикливание

    // ===== Плавное движение ног =====
    float s_current    = sin(phase);       // левая нога (вперед/назад)
    float s_op_current = sin(phase + PI);  // правая нога
    float c_current    = sin(phase * 0.5); // небольшой наклон корпуса

    // ===== Положение ног =====
    int leg_left   = constrain(90 + 40 * s_current, 30, 150);
    int leg_right  = constrain(90 + 40 * s_op_current, 30, 150);
    int foot_left  = constrain(90 + 70 * max(0.0, s_current), 0, 180);   // увеличен подъем
    int foot_right = constrain(90 + 70 * max(0.0, s_op_current), 0, 180); // увеличен подъем

    need_pos[0] = leg_left;    // LEFT_FOOT
    need_pos[1] = leg_right;   // RIGHT_FOOT
    need_pos[2] = foot_left;   // LEFT_LEG
    need_pos[3] = foot_right;  // RIGHT_LEG

    // ===== Руки =====
    int hand_left  = constrain(20 + 160 * pow(max(0.0, s_op_current), 3) + 10 * c_current, 0, 180);
    int hand_right = constrain(20 + 160 * pow(max(0.0, s_current), 3) - 10 * c_current, 0, 180);

    need_pos[4] = hand_left;   // LEFT_HAND
    need_pos[5] = hand_right;  // RIGHT_HAND
}
 

void update_servos() {
  for(int zz = 0; zz < 6; zz++) { 
    if((need_pos[zz] != pos[zz]) && (millis() > times[zz])) {
      if(need_pos[zz] > pos[zz])
        pos[zz]++;
      else
        pos[zz]--;
      times[zz] = millis() + turn_speed;
    }
    myservo[zz].write(pos[zz]);
  }
}

bool Animation = false;

unsigned long lasteUpdate = 0; // время последнего обновления
int currentFrame = 0;
int animationFrames[] = {7, 8, 9}; // узкая улыбка, широкая улыбка, буква O
int frameCount = 3;
const unsigned long frameDelay = 500; // задержка между кадрами в мс

void showAnimationLoop() {
  unsigned long currentMillis = millis();

  // Проверяем, пора ли сменить кадр
  if (currentMillis - lasteUpdate >= frameDelay) {
    lasteUpdate = currentMillis;          // обновляем время последнего кадра
    int frameIndex = animationFrames[currentFrame];

    for (int i = 0; i < 8; i++) {
      uint8_t temp = 0;
      for (int j = 0; j < 8; j++) {
        // Берем бит из столбца j и строки i
        uint8_t bit = (disp1[frameIndex][j] >> (7 - i)) & 0x01;

        // Помещаем его в перевернутый столбец и строку
        temp |= bit << (7 - j);
      }
      // Переворачиваем строки
      now_led[7 - i] = temp;
    }

    updatE(); // обновляем матрицу

    // Переходим к следующему кадру
    currentFrame++;
    if (currentFrame >= frameCount) {
      currentFrame = 0;
    }
  }
}

void setup(){
  myservo[0].attach(14);  // LEFT_FOOT
  myservo[1].attach(15);  // RIGHT_FOOT
  myservo[2].attach(16);  // LEFT_LEG
  myservo[3].attach(17);  // RIGHT_LEG
  myservo[4].attach(7);   // LEFT_HAND
  myservo[5].attach(8);   // RIGHT_HAND
  
  for(int i = 0; i < 6; i++) {
    myservo[i].write(90);
    need_pos[i] = 90;
    pos[i] = 90;
    times[i] = 0;
  }
  
//  sensor.begin();
  Serial.begin(SERIAL_SPEED);
  
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  
  pinMode(PIN_MATRIX_CLK, OUTPUT);
  pinMode(PIN_MATRIX_CS, OUTPUT);
  pinMode(PIN_MATRIX_DIN, OUTPUT); 
  
  pinMode(PIN_MICROPHONE, INPUT);
  pinMode(13, OUTPUT);
  
  delay(50);
  Init_MAX7219();
  cleaR();
// 
}

void loop() {
  // Обновляем движение sidewave если активно
  if (sidewave_active) {
    sidewave_update();
  }

  if (bend_active) {
    bend_update();
  }

  if (dance3_active) {
  dance3_update();
  }

  if (walking_active) {
  walking_update();
  }

  if (musicPlaying) {
  updateMusic();
  }
  
  // Основное управление сервоприводами
  update_servos();
  
  // Обработка команд с Serial
  if (Serial.available() > 0) {
    int incoming_message = Serial.read();
    
    switch(incoming_message) {
      case 'r': { // Запуск sidewave
        // Параметры: steps, period(ms), height, direction
        
        sidewave_start(1500, 50, 1);
        break;
      }
      case 'l': { // sidewave в другую сторону
        sidewave_start(1500, 50, -1);
        break;
      }
      case 's': { // Стоп все движения
        sidewave_active = false;
        bend_active = false;
        dance3_active = false;
        colourswap = false;
        musicPlaying = false;
        walking_active = false;
        Animation = false;
        analogWrite(PIN_R, 0);
        analogWrite(PIN_G, 0);
        analogWrite(PIN_B, 0);
        for(int i = 0; i < 6; i++) {
          need_pos[i] = 90;
        }
        for (int i = 0; i < 8; i++) {
          now_led[i] = 0;  // устанавливаем все биты строки в 0
        }
        updatE();  // обновляем матрицу
        break;
      }
      case 'a': { // Запуск sidewave
        // Параметры: steps, period(ms), height, direction
        bend_start(1500, 50, 20);
        break;
      }
      case 'd': { // sidewave в другую сторону
        dance3_start(1500, 50, 10);
        break;
      }
      case 'x': { // sidewave в другую сторону
        walking_start();
        break;
      }
      case '0': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[0][j] >> (7 - i)) & 0x01) << j;
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '1': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[1][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '2': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[2][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '3': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[3][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '4': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[4][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '5': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[5][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '6': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[6][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '7': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[7][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '8': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[8][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case '9': {
                uint8_t temp;
                for(i = 0; i < 8; i++) {
                  temp = 0;
                  for(int j = 0; j < 8; j++) {
                    // Берем бит из столбца j и строки i, помещаем в строку i и столбец (7-j)
                    temp |= ((disp1[9][j] >> i) & 0x01) << (7 - j);
                  }
                  now_led[i] = temp;
                }
                updatE();
                break;
               }
      case 'y':{
    startMusic();
    break;
  }
  case 'h':
  {
    Animation = true;
    break;
    }
  case 'f':{
    colourswap = 1;
    break;
  }
}
    }
    if (Animation){
      showAnimationLoop();
    }
  
  // Проверка расстояния (опционально)
//  if(sensor.isFinished()) {
//    dist = sensor.getRange();
//    sensor.start();
//  }
  if (colourswap){
    if (millis() - lastUpdate > updateDelay) {
    lastUpdate = millis();
    
    // Плавно меняем красный цвет (0 → 255 → 0)
    r += dirR;
    if (r >= 255 || r <= 0) dirR = -dirR;
    
    // Зеленый меняется со сдвигом
    g += dirG;
    if (g >= 255 || g <= 0) dirG = -dirG;
    
    // Синий меняется со своим сдвигом
    b += dirB;
    if (b >= 255 || b <= 0) dirB = -dirB;
    
    // Применяем цвета
    analogWrite(PIN_R, r);
    analogWrite(PIN_G, g);
    analogWrite(PIN_B, b);
    }
  }
}
