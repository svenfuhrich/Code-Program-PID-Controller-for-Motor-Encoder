#include <Arduino.h>
#include <math.h>

// ================= 1. PIN HARDWARE L298N CHANNEL 1 (OPSI B) =================
const int PIN_ENA = 25; // Pin PWM Kecepatan -> ENA L298N (Jumper ENA Dicabut)
const int PIN_IN1 = 26; // Pin Digital Arah 1 -> IN1 L298N
const int PIN_IN2 = 27; // Pin Digital Arah 2 -> IN2 L298N

const int FREQ_PWM = 5000;
const int RES_PWM  = 8;
const int CH_PWM   = 0;

// ================= 2. PIN ENCODER 6 KABEL (MOTOR DC 12V 1000 RPM) =================
const int PIN_ENC_A = 34; // Kabel Kuning (C1 / Phase A) -> GPIO 34 ESP32
const int PIN_ENC_B = 35; // Kabel Hijau  (C2 / Phase B) -> GPIO 35 ESP32

// Pulsa magnet x Rasio Gearbox (Default 66.0 untuk rasio 1:6)
float PPR_TOTAL = 66.0;

// ================= 3. PARAMETER KENDALI & PID =================
float targetRPM = 750.0; // Setpoint awal (Input R)
float aktualRPM = 0.0;
float rpmFilter = 0.0;

// Nilai PID Awal
float Kp = 1.65;
float Ki = 3.80;
float Kd = 0.01;
String namaKondisi = "UNDERDAMPED";

float integralErr = 0.0;
float prevErr = 0.0;  

// Interrupt pembacaan pulsa encoder
volatile long pulsaEncoder = 0;
void IRAM_ATTR hitungPulsaISR() {
  pulsaEncoder++;
}

// ================= 4. VARIABEL KALKULASI TABEL MODUL PRAKTIKUM =================
// Menghitung: Rise Time, Overshoot, Settling Time, S-S Error (%)
float peakRPM = 0.0;
float sumSteadyRPM = 0.0;
int countSteadySample = 0;

unsigned long waktuMulaiUji = 0;
unsigned long waktuCapai10  = 0;
unsigned long waktuCapai90  = 0;
unsigned long waktuSettling = 0;

bool lewat10Persen      = false;
bool lewat90Persen      = false;
bool dalamBatasSettling = false;
bool laporanCetakSelesai = false;
bool motorAktif         = true;

// Fungsi penulisan PWM kompatibel ESP32 Core v2.x dan v3.x
void tulisPWM_ENA(int nilaiPWM) {
  nilaiPWM = constrain(nilaiPWM, 0, 255);
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_ENA, nilaiPWM);
  #else
    ledcWrite(CH_PWM, nilaiPWM);
  #endif
}

// Kontrol Motor Channel 1 Opsi B (IN1 = HIGH, IN2 = LOW, ENA = PWM)
void gerakkanMotorOpsiB(int nilaiPWM) {
  if (nilaiPWM > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    tulisPWM_ENA(nilaiPWM);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    tulisPWM_ENA(0);
  }
}

// Reset variabel pengujian & mulai pengujian step response baru
void resetPengujian(String labelKondisi) {
  namaKondisi = labelKondisi;
  motorAktif = true;

  gerakkanMotorOpsiB(0);
  delay(800); // Tahan motor 0.8 detik agar kurva merosot ke 0 RPM

  noInterrupts();
  pulsaEncoder = 0;
  interrupts();

  aktualRPM = 0.0;
  rpmFilter = 0.0;
  integralErr = 0.0;
  prevErr = 0.0;

  peakRPM = 0.0;
  sumSteadyRPM = 0.0;
  countSteadySample = 0;

  lewat10Persen = false;
  lewat90Persen = false;
  dalamBatasSettling = false;
  laporanCetakSelesai = false;

  waktuMulaiUji = millis();
  waktuCapai10  = 0;
  waktuCapai90  = 0;
  waktuSettling = 0;
}

// Cetak ringkasan perhitungan tabel modul ke Serial Monitor
void cetakHasilTabelModul() {
  // 1. Rise Time (Waktu naik dari 10% ke 90% Setpoint)
  float riseTime = (waktuCapai90 > waktuCapai10) ? ((waktuCapai90 - waktuCapai10) / 1000.0) : 0.0;

  // 2. Overshoot (%)
  float overshootPersen = 0.0;
  if (peakRPM > targetRPM) {
    overshootPersen = ((peakRPM - targetRPM) / targetRPM) * 100.0;
  }

  // 3. Settling Time (Waktu masuk pita toleransi +-5% Setpoint)
  float settlingTime = waktuSettling / 1000.0;

  // 4. Steady-State Error dalam PERSEN (%) -> S-S Error % = (|Setpoint - Steady_RPM| / Setpoint) * 100
  float steadyStateRPM = (countSteadySample > 0) ? (sumSteadyRPM / countSteadySample) : rpmFilter;
  float ssErrorPersen = (fabs(targetRPM - steadyStateRPM) / targetRPM) * 100.0;

  Serial.println("\n====================== HASIL TABEL PERCOBAAN MODUL ======================");
  Serial.printf("Kondisi Respon        : %s\n", namaKondisi.c_str());
  Serial.printf("Parameter PID         : Kp = %.2f | Ki = %.2f | Kd = %.2f\n", Kp, Ki, Kd);
  Serial.printf("Target Setpoint (R)   : %.2f RPM\n", targetRPM);
  Serial.printf("Output Tunak (Y_ss)   : %.2f RPM (Puncak / Peak: %.2f RPM)\n", steadyStateRPM, peakRPM);
  Serial.println("-------------------------------------------------------------------------");
  Serial.println("| Rise Time (s) | Overshoot (%) | Settling Time (s) | S-S Error (%)     |");
  Serial.printf("| %-13.3f | %-13.2f | %-17.3f | %-17.2f |\n", riseTime, overshootPersen, settlingTime, ssErrorPersen);
  Serial.println("=========================================================================\n");
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_ENA, FREQ_PWM, RES_PWM);
  #else
    ledcSetup(CH_PWM, FREQ_PWM, RES_PWM);
    ledcAttachPin(PIN_ENA, CH_PWM);
  #endif

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), hitungPulsaISR, RISING);

  delay(1000);
  resetPengujian("UNDERDAMPED");
}

void loop() {
  // ================= 5. INPUT SERIAL DINAMIS (Kp, Ki, Kd, S, Preset) =================
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    // Perintah Preset Respon
    if (inputStr == "1") {
      Kp = 1.65; Ki = 3.80; Kd = 0.01;
      resetPengujian("UNDERDAMPED");
    } else if (inputStr == "2") {
      Kp = 0.75; Ki = 1.25; Kd = 0.08;
      resetPengujian("CRITICALLY DAMPED");
    } else if (inputStr == "3") {
      Kp = 0.22; Ki = 0.38; Kd = 0.05;
      resetPengujian("OVERDAMPED");
    } else if (inputStr == "0") {
      motorAktif = false;
      gerakkanMotorOpsiB(0);
    } 
    // Ubah Kp dinamis (contoh: P1.5 atau P300)
    else if (inputStr.startsWith("P") || inputStr.startsWith("p")) {
      Kp = inputStr.substring(1).toFloat();
      resetPengujian("CUSTOM (Kp=" + String(Kp) + ")");
    } 
    // Ubah Ki dinamis (contoh: I2.5 atau I550)
    else if (inputStr.startsWith("I") || inputStr.startsWith("i")) {
      Ki = inputStr.substring(1).toFloat();
      resetPengujian("CUSTOM (Ki=" + String(Ki) + ")");
    } 
    // Ubah Kd dinamis (contoh: D0.05 atau D35)
    else if (inputStr.startsWith("D") || inputStr.startsWith("d")) {
      Kd = inputStr.substring(1).toFloat();
      resetPengujian("CUSTOM (Kd=" + String(Kd) + ")");
    } 
    // Ubah Setpoint R dinamis (contoh: S600)
    else if (inputStr.startsWith("S") || inputStr.startsWith("s")) {
      float spBaru = inputStr.substring(1).toFloat();
      if (spBaru > 0) {
        targetRPM = spBaru;
        resetPengujian(namaKondisi);
      }
    }
  }

  // Loop sampling konstan setiap 20 ms (50 Hz)
  static unsigned long waktuLalu = 0;
  unsigned long sekarang = millis();

  if (sekarang - waktuLalu >= 20) {
    float dt = (sekarang - waktuLalu) / 1000.0;
    waktuLalu = sekarang;

    if (motorAktif) {
      // 1. Baca pulsa Encoder & Hitung RPM
      noInterrupts();
      long pulsa = pulsaEncoder;
      pulsaEncoder = 0;
      interrupts();

      aktualRPM = (pulsa / PPR_TOTAL) * (60.0 / dt);
      rpmFilter = (0.75 * rpmFilter) + (0.25 * aktualRPM);

      // 2. Kalkulasi Kontroler PID (e = R - Y)
      float error = targetRPM - rpmFilter;
      integralErr = constrain(integralErr + (error * dt), -400.0, 400.0);
      float derivatif = (error - prevErr) / dt;
      prevErr = error;

      float outputPID = (Kp * error) + (Ki * integralErr) + (Kd * derivatif);
      int nilaiPWM = constrain((int)(outputPID * 0.255), 0, 255);
      gerakkanMotorOpsiB(nilaiPWM);

      // 3. Rekam Nilai untuk Kolom Tabel Modul
      unsigned long durasiUji = sekarang - waktuMulaiUji;

      // a) Rise Time (10% ke 90%)
      if (!lewat10Persen && rpmFilter >= (0.10 * targetRPM)) {
        lewat10Persen = true;
        waktuCapai10 = durasiUji;
      }
      if (!lewat90Persen && rpmFilter >= (0.90 * targetRPM)) {
        lewat90Persen = true;
        waktuCapai90 = durasiUji;
      }

      // b) Peak RPM (untuk Overshoot)
      if (rpmFilter > peakRPM) {
        peakRPM = rpmFilter;
      }

      // c) Settling Time (Masuk dan bertahan di pita +-5% Setpoint)
      float selisihRelatif = fabs(targetRPM - rpmFilter) / targetRPM;
      if (selisihRelatif <= 0.05) {
        if (!dalamBatasSettling) {
          dalamBatasSettling = true;
          waktuSettling = durasiUji;
        }
      } else {
        dalamBatasSettling = false; // Jika keluar pita toleransi, reset flag  
      }

      // d) Sampel data Steady-State (Detik ke-3 hingga ke-4) untuk S-S Error
      if (durasiUji >= 3000 && durasiUji <= 4000) {
        sumSteadyRPM += rpmFilter;
        countSteadySample++;
      }

      // 4. Format Output Serial Plotter (4 detik pertama)
      if (durasiUji <= 700) {
        Serial.print("Target_Setpoint:");
        Serial.print(targetRPM);
        Serial.print(",");
        Serial.print("RPM_Respon_Aktual:");
        Serial.print(rpmFilter);
        Serial.print(",");
        Serial.print("Peak_Overshoot:");
        Serial.println(peakRPM);
      } else if (!laporanCetakSelesai) {
        cetakHasilTabelModul(); // Tampilkan tabel hasil kalkulasi di Serial Monitor
        laporanCetakSelesai = true;
      }
    } else {
      rpmFilter = 0.0;
      peakRPM = 0.0;
      Serial.print("Target_Setpoint:");
      Serial.print(targetRPM);
      Serial.print(",");
      Serial.print("RPM_Respon_Aktual:");
      Serial.print(0);
      Serial.print(",");
      Serial.print("Peak_Overshoot:");
      Serial.println(0);
    }
  }
}