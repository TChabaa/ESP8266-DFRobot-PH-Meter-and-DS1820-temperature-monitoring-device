#define BLYNK_TEMPLATE_ID "TMPL6Pm-IUzTD"
#define BLYNK_TEMPLATE_NAME "pH Monitor"
#define BLYNK_AUTH_TOKEN "meS0d8foG8LVAnb6yL27xVXTZIh4Kx4x"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- WiFi ---
char ssid[] = "PUTRA_BZ";
char pass[] = "71757175";

// --- Pins ---
#define SDA_PIN D2
#define SCL_PIN D1
const uint8_t LED_R = D5;
const uint8_t LED_G = D6;
const uint8_t LED_B = D7;
const uint8_t BUZZER = D8;
const uint8_t ONE_WIRE_BUS = D3;

// --- OneWire & ADS ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_ADS1115 ads;

// --- Calibration ---
float PH7 = 15000.0;
float PH4 = 19140.0;

// --- Thresholds ---
float minPH = 6.0;
float maxPH = 8.5;
float minTempC = 20.0;
float maxTempC = 25.0;

// --- Buzzer state machine ---
unsigned long buzzerStart = 0;
unsigned long lastToggle = 0;
bool buzzerActive = false;
bool buzzerPhaseOn = false;


// --- Functions ---
float readPH(float adc) {
  float slope = (7.0 - 4.0) / (PH7 - PH4);
  return slope * (adc - PH7) + 7.0;
}

float readTemperatureC() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void LEDControl(float tempC) {
  if (tempC == -127.0) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    delay(100);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    return;
  }

  if (tempC < minTempC) { // dingin
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  } else if (tempC > maxTempC) { // panas
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  } else { // normal
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  }
}

void buzzerControl(float phValue) {
  bool unsafe = (phValue < minPH || phValue > maxPH);
  unsigned long now = millis();

  if (unsafe) {
    // Start buzzer cycle if not active
    if (!buzzerActive) {
      buzzerActive = true;
      buzzerStart = now;
      lastToggle = now;
      buzzerPhaseOn = true;
      digitalWrite(BUZZER, HIGH);
    }

    // Active cycle for 6 seconds
    if (now - buzzerStart <= 6000) {
      // Toggle every 1 second
      if (now - lastToggle >= 1000) {
        buzzerPhaseOn = !buzzerPhaseOn;
        digitalWrite(BUZZER, buzzerPhaseOn ? HIGH : LOW);
        lastToggle = now;
      }
    }
    // Completed 6 seconds → Cooldown 10 seconds
    else if (now - buzzerStart <= 6000 + 10000) {
      digitalWrite(BUZZER, LOW);  // off during cooldown
    }
    // Restart cycle after 16 seconds total (6 work + 10 rest)
    else {
      buzzerActive = false; 
    }
  } 
  else {
    // Safe → Completely reset
    buzzerActive = false;
    digitalWrite(BUZZER, LOW);
  }
}

// --- BLYNK Inputs (Sliders) ---
BLYNK_WRITE(V2) { minTempC = param.asFloat(); }
BLYNK_WRITE(V3) { maxTempC = param.asFloat(); }
BLYNK_WRITE(V4) { minPH = param.asFloat(); }
BLYNK_WRITE(V5) { maxPH = param.asFloat(); }

// Optional: Manual buzzer control
void setup() {
  Serial.begin(9600);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(BUZZER, LOW);

  sensors.begin();
  Wire.begin(SDA_PIN, SCL_PIN);
  ads.begin();
  ads.setGain(GAIN_ONE);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("Blynk Connected!");
}

void loop() {
  Blynk.run();

  int16_t raw_adc = ads.readADC_SingleEnded(0);
  float phValue = readPH(raw_adc);
  float tempC = readTemperatureC();

  LEDControl(tempC);
  buzzerControl(phValue);
  Serial.print("Temperature (C): ");
  Serial.print(tempC);
  Serial.print(" pH: ");
  Serial.println(phValue);

  Serial.print("minTempC: ");
  Serial.print(minTempC);
  Serial.print(" maxTempC: ");
  Serial.print(maxTempC);
  Serial.print(" minPH: ");
  Serial.print(minPH);
  Serial.print(" maxPH: ");
  Serial.println(maxPH);
  // Kirim data ke Blynk
  Blynk.virtualWrite(V0, tempC);
  Blynk.virtualWrite(V1, phValue);

  delay(1000);
}
