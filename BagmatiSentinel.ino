/*
 * Bagmati Sentinel: Robotic Water Quality Monitor
 * Authors: Abriya Shrestha, Nirvana Singh Rayamajhi, Kiran Khatri
 * United School - YSS-2025
 * 
 * Hardware Requirements:
 * - Arduino UNO R3
 * - DS18B20 Temperature Sensor
 * - Analog pH Sensor Kit
 * - Analog TDS Sensor Kit
 * - HC-05 Bluetooth Module
 * - Buck Converter for power management
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>

// Pin Definitions
#define TDS_PIN A0
#define PH_PIN A1
#define TEMP_PIN 2
#define BT_RX 10
#define BT_TX 11

// Sensor Calibration Constants
#define VREF 5.0              // Arduino reference voltage
#define SCOUNT 30             // Sample count for averaging
#define PH_OFFSET 0.0         // pH sensor calibration offset

// Temperature Compensation - DISABLED
// #define TEMP_COEFFICIENT 0.02 // TDS temperature compensation coefficient
// #define STANDARD_TEMP 25.0    // Standard temperature for TDS normalization

// Chemical Proxy Algorithm Thresholds
#define TDS_CLEAN_MAX 300
#define TDS_SEWAGE_MIN 400
#define TDS_INDUSTRIAL_MIN 600
#define PH_NEUTRAL_MIN 6.0
#define PH_NEUTRAL_MAX 7.5
#define PH_SAFE_MIN 6.5
#define PH_SAFE_MAX 8.0
#define PH_ACID_MAX 5.5
#define PH_ALKALINE_MIN 9.0

// Spill Detection
#define SPILL_THRESHOLD 100   // TDS change threshold in ppm
#define SAMPLING_INTERVAL 2000 // 2 seconds between readings

// Initialize sensors
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Global variables
float temperature = 25.0;
float tdsValue = 0.0;
float phValue = 7.0;
float previousTDS = 0.0;
unsigned long lastReadTime = 0;
bool farmerMode = false;

// Buffers for sensor averaging
int tdsBuffer[SCOUNT];
int tdsIndex = 0;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  tempSensor.begin();
  
  pinMode(TDS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  
  // Initialize TDS buffer
  for (int i = 0; i < SCOUNT; i++) {
    tdsBuffer[i] = 0;
  }
  
  Serial.println("Bagmati Sentinel Initialized");
  Serial.println("Commands: 'STANDARD' or 'FARMER'");
  bluetooth.println("Bagmati Sentinel Ready");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for Bluetooth commands
  checkBluetoothCommands();
  
  // Sample sensors at defined interval
  if (currentTime - lastReadTime >= SAMPLING_INTERVAL) {
    lastReadTime = currentTime;
    
    // Read all sensors
    temperature = readTemperature();
    tdsValue = readTDS(temperature);
    phValue = readPH();
    
    // Display readings
    displayReadings();
    
    // Chemical Proxy Algorithm
    String diagnosis = chemicalProxyAlgorithm(tdsValue, phValue);
    
    // Spill Detector
    checkSpillEvent(tdsValue, previousTDS);
    
    // Display diagnosis
    if (farmerMode) {
      displayFarmerMode(tdsValue, phValue, diagnosis);
    } else {
      displayStandardMode(tdsValue, phValue, temperature, diagnosis);
    }
    
    // Store current TDS for next comparison
    previousTDS = tdsValue;
    
    Serial.println("----------------------------------------");
    bluetooth.println("----------------------------------------");
  }
  
  delay(100);
}

float readTemperature() {
  tempSensor.requestTemperatures();
  float temp = tempSensor.getTempCByIndex(0);
  
  // Validate temperature reading
  if (temp < -50 || temp > 100) {
    return 25.0; // Return default if invalid
  }
  return temp;
}

float readTDS(float temp) {
  // Collect samples for averaging
  tdsBuffer[tdsIndex] = analogRead(TDS_PIN);
  tdsIndex = (tdsIndex + 1) % SCOUNT;
  
  // Calculate average
  int sum = 0;
  for (int i = 0; i < SCOUNT; i++) {
    sum += tdsBuffer[i];
  }
  float averageVoltage = (sum / SCOUNT) * (VREF / 1024.0);
  
  // Convert voltage to TDS value (ppm) - NO temperature compensation
  float tds = (133.42 * averageVoltage * averageVoltage * averageVoltage 
               - 255.86 * averageVoltage * averageVoltage 
               + 857.39 * averageVoltage) * 0.5;
  
  return tds;
}

float readPH() {
  int samples = 10;
  int sum = 0;
  
  // Average multiple readings for stability
  for (int i = 0; i < samples; i++) {
    sum += analogRead(PH_PIN);
    delay(10);
  }
  
  float averageVoltage = (sum / samples) * (VREF / 1024.0);
  
  // Convert voltage to pH value
  // Standard pH sensor equation: pH = 7.0 - (voltage - midpoint_voltage) / slope
  float ph = 7.0 - ((averageVoltage - 2.5) / 0.18) + PH_OFFSET;
  
  // Constrain pH to valid range
  if (ph < 0) ph = 0;
  if (ph > 14) ph = 14;
  
  return ph;
}

String chemicalProxyAlgorithm(float tds, float ph) {
  String diagnosis = "";
  
  // Logic from Table 1: Chemical Proxy Algorithm
  
  // Condition 1: High Conductivity & Neutral pH → Sewage/Nitrates/Phosphates
  if (tds > TDS_SEWAGE_MIN && ph >= PH_NEUTRAL_MIN && ph <= PH_NEUTRAL_MAX) {
    diagnosis = "SEWAGE RUNOFF DETECTED - Nitrates/Phosphates Present";
  }
  // Condition 2: High Conductivity & Extreme pH → Heavy Metals/Industrial
  else if (tds > TDS_INDUSTRIAL_MIN && (ph < PH_ACID_MAX || ph > PH_ALKALINE_MIN)) {
    diagnosis = "INDUSTRIAL POLLUTION - Heavy Metals/Acids/Alkalines";
  }
  // Condition 3: Clean Water
  else if (tds < TDS_CLEAN_MAX && ph >= PH_SAFE_MIN && ph <= PH_SAFE_MAX) {
    diagnosis = "WATER SAFE - Good for drinking and use";
  }
  // Moderate contamination
  else if (tds >= TDS_CLEAN_MAX && tds <= TDS_SEWAGE_MIN) {
    diagnosis = "MODERATE CONTAMINATION - Caution advised";
  }
  // pH out of safe range but low TDS
  else if (tds < TDS_SEWAGE_MIN && (ph < PH_SAFE_MIN || ph > PH_SAFE_MAX)) {
    diagnosis = "pH IMBALANCE - Check acid/alkaline levels";
  }
  else {
    diagnosis = "ANOMALOUS READING - Further testing recommended";
  }
  
  return diagnosis;
}

void checkSpillEvent(float currentTDS, float prevTDS) {
  float tdsChange = abs(currentTDS - prevTDS);
  
  if (tdsChange > SPILL_THRESHOLD) {
    Serial.println("*** SUDDEN SPILL ALERT ***");
    Serial.print("TDS changed by: ");
    Serial.print(tdsChange);
    Serial.println(" ppm");
    
    bluetooth.println("*** SUDDEN SPILL ALERT ***");
    bluetooth.print("TDS change: ");
    bluetooth.print(tdsChange);
    bluetooth.println(" ppm");
  }
}

void displayReadings() {
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  Serial.print("TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm");
  
  Serial.print("pH: ");
  Serial.println(phValue);
}

void displayStandardMode(float tds, float ph, float temp, String diagnosis) {
  Serial.println("\n--- STANDARD MONITORING MODE ---");
  Serial.println(diagnosis);
  
  bluetooth.println("\n--- STANDARD MODE ---");
  bluetooth.print("Temp: ");
  bluetooth.print(temp);
  bluetooth.println("C");
  bluetooth.print("TDS: ");
  bluetooth.print(tds);
  bluetooth.println(" ppm");
  bluetooth.print("pH: ");
  bluetooth.println(ph);
  bluetooth.println(diagnosis);
}

void displayFarmerMode(float tds, float ph, String diagnosis) {
  Serial.println("\n--- FARMER MODE: Irrigation Suitability ---");
  
  bool safeForIrrigation = true;
  String recommendation = "";
  
  // Check TDS for irrigation
  if (tds < 300) {
    recommendation += "EXCELLENT for irrigation. Low salt content.\n";
  } else if (tds >= 300 && tds < 600) {
    recommendation += "GOOD for most crops. Monitor salt buildup.\n";
  } else if (tds >= 600 && tds < 1000) {
    recommendation += "MODERATE RISK. Suitable for salt-tolerant crops only.\n";
    safeForIrrigation = false;
  } else {
    recommendation += "HIGH RISK. NOT recommended for irrigation. Risk of soil toxicity.\n";
    safeForIrrigation = false;
  }
  
  // Check pH for irrigation
  if (ph >= 6.0 && ph <= 8.0) {
    recommendation += "pH is SUITABLE for most crops.";
  } else {
    recommendation += "pH OUT OF RANGE. May affect nutrient availability.";
    safeForIrrigation = false;
  }
  
  Serial.println(recommendation);
  Serial.print("Overall: ");
  Serial.println(safeForIrrigation ? "SAFE FOR IRRIGATION" : "USE WITH CAUTION");
  
  bluetooth.println("\n--- FARMER MODE ---");
  bluetooth.print("TDS: ");
  bluetooth.print(tds);
  bluetooth.println(" ppm");
  bluetooth.print("pH: ");
  bluetooth.println(ph);
  bluetooth.println(recommendation);
  bluetooth.println(safeForIrrigation ? "SAFE FOR IRRIGATION" : "USE WITH CAUTION");
}

void checkBluetoothCommands() {
  if (bluetooth.available()) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "FARMER") {
      farmerMode = true;
      Serial.println("Switched to FARMER MODE");
      bluetooth.println("Switched to FARMER MODE");
    } else if (command == "STANDARD") {
      farmerMode = false;
      Serial.println("Switched to STANDARD MODE");
      bluetooth.println("Switched to STANDARD MODE");
    } else if (command == "STATUS") {
      bluetooth.print("Current Mode: ");
      bluetooth.println(farmerMode ? "FARMER" : "STANDARD");
    } else {
      bluetooth.println("Unknown command. Use: STANDARD, FARMER, or STATUS");
    }
  }
  
  // Also check Serial monitor for commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "FARMER") {
      farmerMode = true;
      Serial.println("Switched to FARMER MODE");
    } else if (command == "STANDARD") {
      farmerMode = false;
      Serial.println("Switched to STANDARD MODE");
    }
  }
}