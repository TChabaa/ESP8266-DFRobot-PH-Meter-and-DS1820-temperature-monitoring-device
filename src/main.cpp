#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>

// --- Pin Definitions ---
#define SDA_PIN D2 // GPIO 4 (I2C SDA)
#define SCL_PIN D1 // GPIO 5 (I2C SCL)
const uint8_t LED_R = D5;
const uint8_t LED_G = D6;
const uint8_t LED_B = D7;
const uint8_t BUZZER = D8;
const uint8_t ONE_WIRE_BUS = D3; // GPIO 0 (DS18B20)

// --- OneWire Setup ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- ADS1115 Setup ---
Adafruit_ADS1115 ads;

// --- Calibration Values (from your raw ADC readings) ---
float PH7 = 15000.0; // Raw ADC value at pH 7.0
float PH4 = 19140.0; // Raw ADC value at pH 4.0

// --- Thresholds (will be set by user) ---
float minPH;
float maxPH;
float minTempC;
float maxTempC;

// --- Serial Command Buffer ---
char serialBuffer[50];
size_t bufferIndex = 0; // Use size_t for array indexing, matches sizeof()

// --- Function Declarations ---
void printHelp();
void parseCommand(String cmd);
void checkSerialCommands();

/**
 * @brief Calculates pH using the point-slope formula.
 */
float readPH(float adc) {
    float slope = (7.0 - 4.0) / (PH7 - PH4);
    float phValue = slope * (adc - PH7) + 7.0;
    return phValue;
}

/**
 * @brief Reads temperature from the DS18B20 sensor.
 */
float readTemperatureC() {
  sensors.requestTemperatures(); 
  float tempC = sensors.getTempCByIndex(0); 
  return tempC;
}

/**
 * @brief Controls the RGB LED based on temperature.
 */
void LEDControl(float tempC) {
  if (tempC == -127.0) {
    // Sensor error: Flash all LEDs
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    delay(100);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    return;
  }

  if (tempC < minTempC) {
    // Cold: Blue
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  } else if (tempC >= minTempC && tempC <= maxTempC) {
    // Normal: Green
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  } else {
    // Hot: Red
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }
}

/**
 * @brief Controls the buzzer based on pH level.
 */
void buzzerControl(float phValue) {
  if (phValue < minPH || phValue > maxPH) {
    // pH out of safe range, turn on buzzer
    digitalWrite(BUZZER, HIGH);
  } else {
    // pH in safe range, turn off buzzer
    digitalWrite(BUZZER, LOW);
  }
}

void setup() {
  Serial.begin(9600); // Use a standard high speed for ESP8266
  Serial.println("\n\nESP8266 Sensor Controller");
  Serial.println("Initializing...");

  // Set pin modes
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Turn off all indicators
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(BUZZER, LOW);

  // Initialize sensors
  sensors.begin();
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ads.begin()) {
    Serial.println("Failed to find ADS1115. Check wiring.");
    while (1); // Stop execution
  }
  
  ads.setGain(GAIN_ONE);
  Serial.println("ADS1115 connection successful.");

  // Set default thresholds
  minPH = 6.0;
  maxPH = 8.5;
  minTempC = 20.0;
  maxTempC = 25.0;

  // Test beep
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);

  // Print help menu
  printHelp();
  Serial.println("Reading sensors...");
}

void loop() {
  // 1. Check for user commands
  checkSerialCommands();
  
  // 2. Read Sensors
  int16_t raw_adc_value = ads.readADC_SingleEnded(0);
  float volts = ads.computeVolts(raw_adc_value);
  float phValue = readPH(raw_adc_value);
  float tempC = readTemperatureC();

  // 3. Print values
  Serial.print("Temp: ");
  if (tempC == -127.00) {
    Serial.print("Error! Check wiring.");
  } else {
    Serial.print(tempC, 2);
    Serial.print(" C");
  }

  Serial.print("   |   pH: ");
  Serial.print(phValue, 2);

  Serial.print("   |   Raw: ");
  Serial.print(raw_adc_value);

  Serial.print("   |   Volt: ");
  Serial.print(volts, 4);
  Serial.println(" V");

  // 4. Control outputs
  buzzerControl(phValue);
  LEDControl(tempC);

  delay(1000); // Wait a second before the next reading
}

/**
 * @brief Checks for new data on the Serial port and builds a command string.
 */
void checkSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') { // End of command
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0'; // Null-terminate the string
        parseCommand(String(serialBuffer));
        bufferIndex = 0; // Reset buffer
      }
    } else if (bufferIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[bufferIndex++] = c; // Add char to buffer
    }
  }
}

/**
 * @brief Parses the completed command string and updates variables.
 */
void parseCommand(String cmd) {
  cmd.toUpperCase(); // Make command case-insensitive
  Serial.println();
  Serial.print("Received command: ");
  Serial.println(cmd);

  if (cmd.startsWith("SETPH")) {
    // sscanf is a C function to parse formatted strings
    // %f reads a floating-point number
    int success = sscanf(cmd.c_str(), "SETPH %f %f", &minPH, &maxPH);
    if (success == 2) {
      Serial.print("OK. New pH range: ");
      Serial.print(minPH, 2);
      Serial.print(" to ");
      Serial.println(maxPH, 2);
    } else {
      Serial.println("Error. Format: SETPH <min> <max>");
    }
  } 
  else if (cmd.startsWith("SETTEMP")) {
    int success = sscanf(cmd.c_str(), "SETTEMP %f %f", &minTempC, &maxTempC);
    if (success == 2) {
      Serial.print("OK. New Temp range: ");
      Serial.print(minTempC, 1);
      Serial.print("C to ");
      Serial.print(maxTempC, 1);
      Serial.println("C");
    } else {
      Serial.println("Error. Format: SETTEMP <min> <max>");
    }
  }
  else if (cmd == "STATUS") {
    Serial.println("--- Current Settings ---");
    Serial.print("pH Range:     ");
    Serial.print(minPH, 2);
    Serial.print(" to ");
    Serial.println(maxPH, 2);
    Serial.print("Temp Range:   ");
    Serial.print(minTempC, 1);
    Serial.print("C to ");
    Serial.print(maxTempC, 1);
    Serial.println("C");
  }
  else if (cmd == "HELP") {
    printHelp();
  }
  else {
    Serial.println("Unknown command. Type 'HELP' for options.");
  }
  Serial.println();
}

/**
 * @brief Prints the user help menu.
 */
void printHelp() {
  Serial.println("\n--- Serial Command Menu ---");
  Serial.println("Type commands and press Enter:");
  Serial.println("  SETPH <min> <max>    (e.g., SETPH 6.5 7.8)");
  Serial.println("  SETTEMP <min> <max>  (e.g., SETTEMP 20.0 28.5)");
  Serial.println("  STATUS               (View current settings)");
  Serial.println("  HELP                 (Show this menu)");
  Serial.println("---------------------------");
}

