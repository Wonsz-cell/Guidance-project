#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>
#include "TrajectoryFollowing.h"

// ======================= USTAWIENIA DO ZMIANY (TU) =======================
// Tryb jazdy po starcie. Zmiana wymaga ponownego wgrania kodu (brak komend przez Serial).
//   Idle     = serwo na środku
//   Straight = utrzymanie kursu z chwili startu
//   Circle   = okrąg (przeciwnie do ruchu wskazówek zegara)
//   Eight    = ósemka (lemniskata)
enum class Mode { Idle, Straight, Circle, Eight };
const Mode START_MODE = Mode::Straight;

// true  = wypisuj dane na Serial (debug przy kablu)
// false = nic nie wypisuj i w ogóle nie inicjalizuj Serial (jazda bez dostępu do Serial)
#define SERIAL_OUTPUT true

#if SERIAL_OUTPUT
  #define LOG_BEGIN(baud)  Serial.begin(baud)
  #define LOGF(...)        Serial.printf(__VA_ARGS__)
  #define LOGLN(x)         Serial.println(x)
#else
  #define LOG_BEGIN(baud)  ((void)0)
  #define LOGF(...)        ((void)0)
  #define LOGLN(x)         ((void)0)
#endif

// ======================= KONFIGURACJA =======================
#define SDA_PIN   14
#define SCL_PIN   15
#define MPU_ADDR  0x68
#define QMC_ADDR  0x2C      // QMC5883P

#define ACC_LSB_PER_G     16384.0f   // ±2 g
#define GYRO_LSB_PER_DPS  65.5f      // ±500 °/s

#define LOOP_DT           0.01f     // 100 Hz
#define TELEMETRY_MS      100       // jedna linia telemetrii co 100 ms
#define MAG_TIMEOUT_MS    200       // po tylu ms bez próbki magnetometr uznajemy za nieważny

// 1 = kalibruj magnetometr przy starcie (obracaj czujnik 20 s),
// 0 = użyj wartości wpisanych poniżej
#define MAG_CAL_AT_BOOT   0
#define MAG_CAL_MS        20000
// 1 = tylko podgląd osi magnetometru i akcelerometru (do testu mapowania), bez filtru
#define DEBUG_MAG_AXES    0

// --- Serwo ---
#define SERVO_PIN         12
#define SERVO_CENTER      90.0f
#define MAX_SERVO_DEV     30.0f
// +1 / -1: znak skrętu. Dodatni yaw (obrót w lewo) => serwo > 90 gdy STEER_SIGN = +1.
// Sprawdź na stole, w którą stronę skręcają koła, i w razie potrzeby zmień na -1.
#define STEER_SIGN        1.0f

Servo servo;

float magOffset[3] = {-10.00f, 174.00f, 114.00f};
float magScale[3]  = {0.9729f, 1.1444f, 0.9105f};
float gyroBias[3]  = {0, 0, 0};      // °/s

const float DEG2RAD = 0.017453292f;
const float RAD2DEG = 57.29578f;

// ======================= FILTR =======================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.1f;
const float BETA_RUN   = 0.1f;
const float BETA_START = 1.0f;       // szybka zbieżność po starcie
const uint32_t START_MS = 3000;

uint32_t lastMicros = 0, lastPrintMs = 0, lastMagMs = 0, bootMs = 0;
float mx = 0, my = 0, mz = 0;        // ostatni skalibrowany pomiar magnetometru
bool magValid = false;
float roll = 0, pitch = 0, yaw = 0;
float ax, ay, az, gx, gy, gz;

// ======================= TRYBY JAZDY =======================
// Tryb wybierasz na górze pliku (START_MODE); program wchodzi w niego sam po zbieżności filtru (START_MS).
Mode actualMode = Mode::Idle;
bool armed = false;                  // true po zbieżności filtru

// Tryb Straight: regulator P kursu
float kP = 0.2f;                     // [deg serwa / deg błędu]
float setPoint = 0.0f;               // zadany kurs [deg]; ustawiany przy wejściu w tryb
float output = 0.0f;

TrajectoryOutput lastResult = {};

TrajectoryConfig trajectoryConfig = {
    3.0f,           // scale [m]            (ósemka ~6 m x 2 m)
    1.5f,           // circleRadius [m]
    1.0f,           // vehicleSpeed [m/s]   (zmniejszone: ósemka a=2 przy 2 m/s dawała ~0,6 g)

    0.5f,           // K_heading [1/s]
    5.0f,           // K_yawRate [deg serwa / (rad/s)]

    0.30f,          // wheelbase [m]  <-- USTAW rzeczywisty rozstaw osi (0 = wyłącza feedforward)
    1.0f,           // servoDegPerWheelDeg (przełożenie serwo -> koła)
    STEER_SIGN,     // steerSign

    SERVO_CENTER,   // servoCenter
    MAX_SERVO_DEV   // maxServoDeviation
};

// ======================= NARZĘDZIA =======================
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Ustawia serwo w stopniach (0..180), z ograniczeniem do zakresu roboczego.
// writeMicroseconds daje rozdzielczość lepszą niż 1 stopień (zakres 1000-2000 us = 0-180 deg).
void setServoAngle(float deg) {
  deg = clampf(deg, SERVO_CENTER - MAX_SERVO_DEV, SERVO_CENTER + MAX_SERVO_DEV);
  servo.writeMicroseconds((int)(1000.0f + deg * (1000.0f / 180.0f)));
}

// ======================= I2C =======================
void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Zwraca: a w [g], g w [°/s]
bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();   // osobne odczyty = jednoznaczna kolejność

  ax = (int16_t)((b[0]  << 8) | b[1])  / ACC_LSB_PER_G;
  ay = (int16_t)((b[2]  << 8) | b[3])  / ACC_LSB_PER_G;
  az = (int16_t)((b[4]  << 8) | b[5])  / ACC_LSB_PER_G;
  gx = (int16_t)((b[8]  << 8) | b[9])  / GYRO_LSB_PER_DPS;
  gy = (int16_t)((b[10] << 8) | b[11]) / GYRO_LSB_PER_DPS;
  gz = (int16_t)((b[12] << 8) | b[13]) / GYRO_LSB_PER_DPS;
  return true;
}

// QMC5883P: status 0x09 (bit0 = DRDY), dane od 0x01: XL XH YL YH ZL ZH (signed)
bool readMagRaw(float &x, float &y, float &z) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)1) != 1) return false;
  uint8_t status = Wire.read();
  if (!(status & 0x01)) return false;                // brak nowych danych

  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6) return false;

  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  x = (int16_t)((b[1] << 8) | b[0]);
  y = (int16_t)((b[3] << 8) | b[2]);
  z = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

// Przemapowanie osi magnetometru (QMC5883P) na układ MPU6050 (prawoskrętny, Z w górę).
// To obrót (wyznacznik macierzy = +1), więc układ pozostaje prawoskrętny.
// Jeśli yaw rośnie w złą stronę lub |M|/cos(A,M) zachowują się dziwnie - użyj DEBUG_MAG_AXES.
void remapMag(float rx, float ry, float rz, float &ox, float &oy, float &oz) {
  ox = -ry;
  oy = -rx;
  oz = -rz;
}

// ======================= KALIBRACJE =======================
void calibrateGyro() {
  LOGLN("Kalibracja zyroskopu - nie ruszaj czujnika...");
  float sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < 500; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) { sx += gx; sy += gy; sz += gz; n++; }
    delay(3);
  }
  if (n > 0) { gyroBias[0] = sx / n; gyroBias[1] = sy / n; gyroBias[2] = sz / n; }
  LOGF("Bias zyro [dps]: %.3f %.3f %.3f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
}

void calibrateMag() {
  LOGLN("Kalibracja magnetometru: przez 20 s obracaj czujnik we WSZYSTKICH osiach (osemki)...");
  float mn[3] = { 1e9f,  1e9f,  1e9f};
  float mxv[3] = {-1e9f, -1e9f, -1e9f};
  bool got = false;
  uint32_t t0 = millis();
  while (millis() - t0 < MAG_CAL_MS) {
    float rx, ry, rz;
    if (readMagRaw(rx, ry, rz)) {
      float v[3];
      remapMag(rx, ry, rz, v[0], v[1], v[2]);
      for (int i = 0; i < 3; i++) {
        if (v[i] < mn[i])  mn[i]  = v[i];
        if (v[i] > mxv[i]) mxv[i] = v[i];
      }
      got = true;
    }
    delay(5);
  }
  if (!got) {
    LOGLN("!!! Brak danych z magnetometru - kalibracja pominieta");
    return;
  }
  float r[3], avg = 0;
  for (int i = 0; i < 3; i++) {
    magOffset[i] = (mxv[i] + mn[i]) * 0.5f;      // hard-iron
    r[i] = (mxv[i] - mn[i]) * 0.5f;
    avg += r[i];
  }
  avg /= 3.0f;
  for (int i = 0; i < 3; i++) magScale[i] = (r[i] > 1.0f) ? avg / r[i] : 1.0f;   // soft-iron (uproszczone)

  LOGLN("Gotowe. Mozesz wkleic te wartosci na stale i ustawic MAG_CAL_AT_BOOT 0:");
  LOGF("float magOffset[3] = {%.2ff, %.2ff, %.2ff};\n", magOffset[0], magOffset[1], magOffset[2]);
  LOGF("float magScale[3]  = {%.4ff, %.4ff, %.4ff};\n", magScale[0], magScale[1], magScale[2]);
}

// ======================= MADGWICK =======================
static void integrateQuat(float d0, float d1, float d2, float d3, float dt) {
  q0 += d0 * dt; q1 += d1 * dt; q2 += d2 * dt; q3 += d3 * dt;
  float r = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= r; q1 *= r; q2 *= r; q3 *= r;
}

// 6DOF (zyroskop [rad/s] + akcelerometr)
void madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float d0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float d1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float d2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float d3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an > 0.0f) {
    ax /= an; ay /= an; az /= an;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + 8.0f * q1q1 * q1 + 8.0f * q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + 8.0f * q1q1 * q2 + 8.0f * q2q2 * q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 0.0f) {
      d0 -= beta * s0 / sn; d1 -= beta * s1 / sn; d2 -= beta * s2 / sn; d3 -= beta * s3 / sn;
    }
  }
  integrateQuat(d0, d1, d2, d3, dt);
}

// 9DOF (zyroskop [rad/s] + akcelerometr + magnetometr)
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float mx, float my, float mz, float dt) {
  float mn = sqrtf(mx * mx + my * my + mz * mz);
  if (mn == 0.0f) { madgwickUpdateIMU(gx, gy, gz, ax, ay, az, dt); return; }   // fallback

  float d0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float d1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float d2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float d3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an > 0.0f) {
    ax /= an; ay /= an; az /= an;
    mx /= mn; my /= mn; mz /= mn;

    float _2q0mx = 2.0f * q0 * mx, _2q0my = 2.0f * q0 * my, _2q0mz = 2.0f * q0 * mz, _2q1mx = 2.0f * q1 * mx;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2, _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    // Kierunek pola Ziemi w układzie Ziemi (kompensacja nachylenia)
    float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    // UWAGA: nazwy _2bx/_2bz oznaczają 2*bx i 2*bz (tak zakładają funkcja błędu i jakobian poniżej).
    // W oryginalnym kodzie referencyjnym brakowało tu czynnika 2, przez co błąd magnetometru
    // nie zerował się w prawdziwej orientacji (sprawdzone numerycznie).
    float _2bx = 2.0f * sqrtf(hx * hx + hy * hy);
    float _2bz = 2.0f * (-_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3);
    float _4bx = 2.0f * _2bx;
    float _4bz = 2.0f * _2bz;

    // Błędy: grawitacja i pole magnetyczne
    float fax = 2.0f * q1q3 - _2q0q2 - ax;
    float fay = 2.0f * q0q1 + _2q2q3 - ay;
    float faz = 1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az;
    float fmx = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
    float fmy = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
    float fmz = _2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz;

    // Gradient
    float s0 = -_2q2 * fax + _2q1 * fay - _2bz * q2 * fmx + (-_2bx * q3 + _2bz * q1) * fmy + _2bx * q2 * fmz;
    float s1 =  _2q3 * fax + _2q0 * fay - 4.0f * q1 * faz + _2bz * q3 * fmx + (_2bx * q2 + _2bz * q0) * fmy + (_2bx * q3 - _4bz * q1) * fmz;
    float s2 = -_2q0 * fax + _2q3 * fay - 4.0f * q2 * faz + (-_4bx * q2 - _2bz * q0) * fmx + (_2bx * q1 + _2bz * q3) * fmy + (_2bx * q0 - _4bz * q2) * fmz;
    float s3 =  _2q1 * fax + _2q2 * fay + (-_4bx * q3 + _2bz * q1) * fmx + (-_2bx * q0 + _2bz * q2) * fmy + _2bx * q1 * fmz;

    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 0.0f) {
      d0 -= beta * s0 / sn; d1 -= beta * s1 / sn; d2 -= beta * s2 / sn; d3 -= beta * s3 / sn;
    }
  }
  integrateQuat(d0, d1, d2, d3, dt);
}

void getEuler(float &roll, float &pitch, float &yaw) {
  roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
  float s = 2.0f * (q0 * q2 - q3 * q1);
  s = fmaxf(-1.0f, fminf(1.0f, s));
  pitch = asinf(s) * RAD2DEG;
  yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}

// ======================= STEROWANIE =======================

// Zmiana trybu: każdy tryb startuje od aktualnego kursu pojazdu,
// więc ścieżka zawsze zaczyna się "tam, gdzie stoi auto".
void setMode(Mode m) {
  actualMode = m;
  switch (m) {
    case Mode::Idle:
      setServoAngle(SERVO_CENTER);
      break;
    case Mode::Straight:
      setPoint = yaw;                                          // trzymaj kurs z chwili włączenia
      break;
    case Mode::Circle:
      trajectoryFollowingStart(PathType::Circle, yaw);
      break;
    case Mode::Eight:
      trajectoryFollowingStart(PathType::Lemniscate, yaw);
      break;
  }
  LOGF("Tryb: %d\n", (int)m);
}

// Jeden krok sterowania; wywoływany PO aktualizacji filtru (świeży yaw).
void controlStep(float dt) {
  switch (actualMode) {
    case Mode::Idle:
      setServoAngle(SERVO_CENTER);
      break;

    case Mode::Straight: {
      float err = wrap180(setPoint - yaw);                     // zawinięty błąd kursu
      output = clampf(err * kP, -MAX_SERVO_DEV, MAX_SERVO_DEV);
      setServoAngle(SERVO_CENTER + STEER_SIGN * output);
      break;
    }

    case Mode::Circle:
    case Mode::Eight:
      lastResult = trajectoryFollowingUpdate(yaw, gz * DEG2RAD, dt);
      setServoAngle(lastResult.servoCommand);
      break;
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  LOG_BEGIN(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(200);

  // --- MPU6050 ---
  writeReg(MPU_ADDR, 0x6B, 0x80);  delay(100);   // reset
  writeReg(MPU_ADDR, 0x6B, 0x01);  delay(10);    // wybudzenie, zegar PLL
  writeReg(MPU_ADDR, 0x6A, 0x00);                // I2C master OFF
  writeReg(MPU_ADDR, 0x37, 0x02);                // BYPASS -> magnetometr widoczny na magistrali
  writeReg(MPU_ADDR, 0x19, 0x04);                // próbkowanie 200 Hz
  writeReg(MPU_ADDR, 0x1A, 0x03);                // DLPF ~44 Hz
  writeReg(MPU_ADDR, 0x1B, 0x08);                // żyro ±500 °/s
  writeReg(MPU_ADDR, 0x1C, 0x00);                // akcel ±2 g
  LOGF("MPU WHO_AM_I: 0x%02X\n", readReg(MPU_ADDR, 0x75));

  // --- QMC5883P ---
  LOGF("QMC ID (rej. 0x00): 0x%02X  (oczekiwane 0x80)\n", readReg(QMC_ADDR, 0x00));
  writeReg(QMC_ADDR, 0x0B, 0x80);  delay(10);    // soft reset
  writeReg(QMC_ADDR, 0x0B, 0x02);
  writeReg(QMC_ADDR, 0x0A, 0xCD);                // tryb ciągły, 200 Hz, OSR 8
  delay(50);

  calibrateGyro();
#if MAG_CAL_AT_BOOT && !DEBUG_MAG_AXES
  calibrateMag();
#elif DEBUG_MAG_AXES
  calibrateMag();                                // do testu osi też potrzebny offset
#endif

  servo.attach(SERVO_PIN, 1000, 2000);
  setServoAngle(SERVO_CENTER);                   // serwo na środku, dopóki filtr się nie ustabilizuje
  trajectoryFollowingInit(trajectoryConfig);

  bootMs = millis();
  lastMicros = micros();
  LOGF("Start. Tryb startowy: %d\n", (int)START_MODE);
}

void loop() {
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt < LOOP_DT) return;

  // --- Czujniki ---
  if (!readMPU(ax, ay, az, gx, gy, gz)) return;  // lastMicros nie zmienione -> dt nie "gubi" czasu
  lastMicros = now;
  if (dt > 0.1f) dt = LOOP_DT;                   // zabezpieczenie po zawieszeniu pętli

  gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];

  float rx, ry, rz;
  if (readMagRaw(rx, ry, rz)) {
    float tx, ty, tz;
    remapMag(rx, ry, rz, tx, ty, tz);
    mx = (tx - magOffset[0]) * magScale[0];
    my = (ty - magOffset[1]) * magScale[1];
    mz = (tz - magOffset[2]) * magScale[2];
    magValid = true;
    lastMagMs = millis();
  } else if (magValid && (millis() - lastMagMs > MAG_TIMEOUT_MS)) {
    magValid = false;                            // magnetometr zamilkł -> filtr 6DOF
  }

#if DEBUG_MAG_AXES
  if (millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();
    float an = sqrtf(ax*ax + ay*ay + az*az);
    float mn = sqrtf(mx*mx + my*my + mz*mz);
    float c  = (an > 0 && mn > 0) ? (ax*mx + ay*my + az*mz) / (an * mn) : 0;
    LOGF("A[g]: %6.2f %6.2f %6.2f | M: %8.1f %8.1f %8.1f | |M|: %7.1f | cos(A,M): %5.2f\n",
                  ax, ay, az, mx, my, mz, mn, c);
  }
  return;
#endif

  // --- Filtr orientacji ---
  beta = (millis() - bootMs < START_MS) ? BETA_START : BETA_RUN;

  if (magValid) madgwickUpdate(gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD, ax, ay, az, mx, my, mz, dt);
  else          madgwickUpdateIMU(gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD, ax, ay, az, dt);

  getEuler(roll, pitch, yaw);                    // w KAŻDEJ iteracji, tuż po filtrze

  // --- Uzbrojenie po zbieżności filtru ---
  if (!armed && (millis() - bootMs >= START_MS)) {
    armed = true;
    setMode(START_MODE);
  }

  // --- Regulator (świeży yaw i gz, ten sam dt co filtr) ---
  controlStep(dt);

#if SERIAL_OUTPUT
  // --- Telemetria (rzadko, jedna linia) ---
  if (millis() - lastPrintMs >= TELEMETRY_MS) {
    lastPrintMs = millis();
    LOGF("M:%d | R:%7.2f P:%7.2f Y:%7.2f | MAG:%s | href:%7.2f herr:%7.2f k:%6.3f | servo:%6.1f\n",
                  (int)actualMode, roll, pitch, yaw, magValid ? "OK" : "--",
                  lastResult.headingReference, lastResult.headingError, lastResult.curvature,
                  (actualMode == Mode::Straight) ? SERVO_CENTER + STEER_SIGN * output
                                                 : lastResult.servoCommand);
  }
#endif
}
