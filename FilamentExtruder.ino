#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// WiFi functionality can be added later by uncommenting:
// #include <WiFi.h>

// Thermistor constants (powered by VREF)
#define THERMISTOR_PIN A0
#define THERMISTOR_SERIES_RESISTOR 4660 // Measured 4.7k
#define THERMISTOR_NOMINAL 100000 // 100k unmeasured
#define TEMPERATURE_NOMINAL 25
#define B_COEFFICIENT 3950
#define VREF 3.27  // Measured VREF voltage (3.3V)
#define ADC_BIT_DEPTH 12
#define ADC_MAX_VALUE ((1 << ADC_BIT_DEPTH) - 1)

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS1 0x3C
#define SCREEN_ADDRESS2 0x3D

// Analog potentiometer pins
#define POT1_PIN A1
#define POT2_PIN A2
#define POT_BIT_DEPTH 12
#define POT_MAX_VALUE ((1 << POT_BIT_DEPTH) - 1)

#define MAX_TEMPERATURE 300

#define TORQUE 1000

#define MAX_SPEED 1000

// Heater control pins (GPIO18-21 for relay control)
#define HEATER_1_PIN 18
#define HEATER_2_PIN 19
#define HEATER_3_PIN 20
#define HEATER_4_PIN 21

// Temperature control settings
#define TARGET_TEMP_DEFAULT 220    // Default target temperature in Celsius
#define TEMP_TOLERANCE 2           // Temperature tolerance in degrees
#define HEATER_PULSE_INTERVAL 1000 // Pulse interval in milliseconds
#define MAX_HEATER_ON_TIME 5000    // Maximum continuous heater on time (safety)
#define SAFETY_MAX_TEMP 350        // Safety shutdown temperature

// Control variables
int targetSpeed = 0;
int targetTemperature = TARGET_TEMP_DEFAULT;
float currentTemperature = 0;
bool heatersEnabled = false;
unsigned long lastHeaterUpdate = 0;
unsigned long heaterOnTime = 0;
bool heaterState[4] = {false, false, false, false};

// PID-like control variables
float temperatureError = 0;
float lastTemperatureError = 0;
float heaterPower = 0; // 0.0 to 1.0
unsigned long lastTempUpdate = 0;

// Create display object
int displayAddress = SCREEN_ADDRESS1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables to store potentiometer values
uint16_t pot1Value = 0;
uint16_t pot2Value = 0;

// Function prototypes
void initializeI2C();
bool initializeDisplay();
void initializeHeaters();
uint16_t readAnalogPot(int pin);
float readTemperature();
void updateDisplay();
void scanI2CDevices();
void updateHeaterControl();
void setHeaterState(int heaterIndex, bool state);
void emergencyShutdown();
float calculateHeaterPower(float targetTemp, float currentTemp);
void updateTemperatureControl();

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection to be established (max 10 seconds)
  // This ensures we don't miss initial log messages
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 10000)) {
    delay(100);  // Small delay to prevent tight loop
  }
  
  Serial.println("Raspberry Pi Pico W I2C Project Starting...");
  
  analogReadResolution(ADC_BIT_DEPTH);
  delay(100); // Wait for ADC to settle
  (void)analogRead(THERMISTOR_PIN); // Throw away first reading
 
  // Initialize I2C with custom pins
  initializeI2C();
  
  // Scan for I2C devices
  scanI2CDevices();
  
  // Initialize display
  if (!initializeDisplay()) {
    Serial.println("Display initialization failed - trying alternate address");
    displayAddress = SCREEN_ADDRESS2;
    if (!initializeDisplay()) {
      Serial.println("Display initialization failed - continuing without display");
    }
  }
  
  // Initialize heater control pins
  initializeHeaters();
  
  // Test analog potentiometer reading
  Serial.println("Reading analog potentiometers...");
  pot1Value = readAnalogPot(POT1_PIN);
  pot2Value = readAnalogPot(POT2_PIN);
  Serial.print("Pot1 (A1): ");
  Serial.print(pot1Value);
  Serial.print(", Pot2 (A2): ");
  Serial.println(pot2Value);
  
  Serial.println("Setup complete!");
  Serial.println("Heater control: GPIO18-21 configured for relay control");
  Serial.print("Target temperature: ");
  Serial.print(targetTemperature);
  Serial.println("°C");
}

void loop() {
  // Read analog potentiometer values
  pot1Value = readAnalogPot(POT1_PIN);
  pot2Value = readAnalogPot(POT2_PIN);

  // Map potentiometer values to control ranges with calibration
  // Option: Add dead zones if pots don't reach full 0-4095 range
  targetSpeed = map(constrain(pot1Value, 50, POT_MAX_VALUE - 50), 50, POT_MAX_VALUE - 50, 0, MAX_SPEED);
  targetTemperature = map(constrain(pot2Value, 50, POT_MAX_VALUE - 50), 50, POT_MAX_VALUE - 50, 0, MAX_TEMPERATURE);

  currentTemperature = readTemperature();
  
  // Update temperature control system
  updateTemperatureControl();
  
  // Update heater control based on temperature
  updateHeaterControl();

  // Update display
  updateDisplay();
  
  // Print debug info every second
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.print("Pot1: ");
    Serial.print(pot1Value);
    Serial.print(" | Pot2: ");
    Serial.print(pot2Value);
    Serial.print(" | Speed: ");
    Serial.print(targetSpeed);
    Serial.print(" | Current Temp: ");
    Serial.print(currentTemperature, 1);
    Serial.print("°C | Target Temp: ");
    Serial.print(targetTemperature);
    Serial.print("°C | Heater Power: ");
    Serial.print(heaterPower * 100, 1);
    Serial.print("% | Heaters: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(heaterState[i] ? "ON " : "OFF ");
    }
    Serial.println();
    lastDebug = millis();
  }
}

void initializeI2C() {
  // Initialize I2C with custom pins (SDA=GPIO8, SCL=GPIO9)
  // In Arduino IDE, configure pins explicitly before begin
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();
  
  Serial.println("I2C initialized on SDA=GPIO8, SCL=GPIO9");
}

bool initializeDisplay() {
  // Test display I2C connection first
  Serial.print("Testing display at address 0x");
  Serial.print(displayAddress, HEX);
  Serial.print("... ");
  
  Wire.beginTransmission(displayAddress);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("Display responds to I2C");
  } else {
    Serial.print("Display I2C error: ");
    Serial.println(error);
    return false;
  }
  
  // Initialize SSD1306 display
  if(!display.begin(SSD1306_SWITCHCAPVCC, displayAddress)) {
    Serial.println("SSD1306 allocation failed");
    return false; // Return failure instead of hanging
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pico W ADC System");
  display.println("Initializing...");
  display.display();
  delay(2000);
  
  Serial.println("Display initialized successfully");
  return true;
}

uint16_t readAnalogPot(int pin) {
  // Read analog potentiometer directly
  uint16_t reading = analogRead(pin);
  
  // The ADC reading is already in the correct range (0 to POT_MAX_VALUE)
  // No additional processing needed
  return reading;
}

void updateDisplay() {
  display.clearDisplay();
  
  // Title
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Safety status
  if (currentTemperature > SAFETY_MAX_TEMP - 20) {
    display.println("! TOO HOT !");
  } else if (!heatersEnabled) {
    display.println("Standby");
  } else {
    display.println("Running");
  }
  
  // Draw line separator
  display.drawLine(0, 10, SCREEN_WIDTH-1, 10, SSD1306_WHITE);
  
  // Speed control
  display.setCursor(0, 14);
  display.setTextSize(1);
  display.print("Speed Set: ");
  display.print(targetSpeed);
  display.print(" RPM");
  
  // Temperature info
  display.setCursor(0, 24);
  display.print("Temp Targ: ");
  display.print(targetTemperature);
  display.print("C");
  
  display.setCursor(0, 34);
  display.print("Temp Curr: ");
  display.print(currentTemperature, 1);
  display.print("C");
  
  // Temperature progress bar
  int tempBarWidth = map(constrain(currentTemperature, 0, targetTemperature), 0, targetTemperature, 0, SCREEN_WIDTH-20);
  display.drawRect(10, 44, SCREEN_WIDTH-20, 4, SSD1306_WHITE);
  if (tempBarWidth > 0) {
    display.fillRect(10, 44, tempBarWidth, 4, SSD1306_WHITE);
  }
  
  // Heater status indicators
  display.setCursor(0, 52);
  display.print("Heat: ");
  for (int i = 0; i < 4; i++) {
    display.print(heaterState[i] ? "1" : "0");
  }
  
  // Power indicator
  display.print(" ");
  display.print((int)(heaterPower * 100));
  display.print("%");

  display.display();
}

void scanI2CDevices() {
  Serial.println("Scanning I2C devices...");
  uint8_t deviceCount = 0;
  
  for(uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Add known device identification
      if (address == SCREEN_ADDRESS1 || address == SCREEN_ADDRESS2) {
        Serial.print(" (SSD1306 OLED)");
      }
      Serial.println();
      deviceCount++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
    }
    
    // Add small delay to prevent bus issues
    delay(1);
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found - Check wiring and pull-up resistors!");
    Serial.println("Expected devices:");
    Serial.print("  - ");
    Serial.print(SCREEN_ADDRESS1, HEX);
    Serial.print("/");
    Serial.print(SCREEN_ADDRESS2, HEX);
    Serial.println(": SSD1306 Display");
    Serial.println("  - A1/A2: Analog potentiometers");  
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
}

float readTemperature() {
  int raw = analogRead(THERMISTOR_PIN);
  
  // Calculate resistance using measured VREF and series resistor values
  float voltage = raw * (VREF / (float)ADC_MAX_VALUE);
  float resistance = THERMISTOR_SERIES_RESISTOR * voltage / (VREF - voltage);
  
  // Steinhart-Hart equation
  float steinhart = resistance / THERMISTOR_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  
  return steinhart;
}

void initializeHeaters() {
  // Configure GPIO pins 18-21 as outputs for relay control
  pinMode(HEATER_1_PIN, OUTPUT);
  pinMode(HEATER_2_PIN, OUTPUT);
  pinMode(HEATER_3_PIN, OUTPUT);
  pinMode(HEATER_4_PIN, OUTPUT);
  
  // Configure GPIO28 as VREF output (limited current)
  pinMode(28, OUTPUT);
  digitalWrite(28, HIGH);  // Set GPIO28 to VREF
  
  // Initialize all heaters to OFF
  digitalWrite(HEATER_1_PIN, LOW);
  digitalWrite(HEATER_2_PIN, LOW);
  digitalWrite(HEATER_3_PIN, LOW);
  digitalWrite(HEATER_4_PIN, LOW);
  
  Serial.println("Heater control pins initialized (GPIO18-21)");
  Serial.println("All heaters initialized to OFF state");
  Serial.println("GPIO28 set to HIGH (VREF) - for I2C pull-up resistors");
  Serial.println("Use VREF (Pin 35) for thermistor power, VREF (Pin 36) for I2C devices");
}

void updateTemperatureControl() {
  // Calculate temperature error
  temperatureError = targetTemperature - currentTemperature;
  
  // Safety check - emergency shutdown if temperature too high
  if (currentTemperature > SAFETY_MAX_TEMP) {
    emergencyShutdown();
    return;
  }
  
  // Enable heaters only if target temperature is reasonable
  if (targetTemperature > 50 && targetTemperature < SAFETY_MAX_TEMP) {
    heatersEnabled = true;
  } else {
    heatersEnabled = false;
  }
  
  // Calculate required heater power using simple proportional control
  heaterPower = calculateHeaterPower(targetTemperature, currentTemperature);
  
  lastTempUpdate = millis();
}

float calculateHeaterPower(float targetTemp, float currentTemp) {
  float error = targetTemp - currentTemp;
  
  // Simple proportional control
  float power = error / 50.0; // Scale factor - adjust as needed
  
  // Clamp power to 0-1 range
  if (power < 0) power = 0;
  if (power > 1.0) power = 1.0;
  
  // Dead zone around target temperature
  if (fabs(error) < TEMP_TOLERANCE) {
    power *= 0.1; // Reduce power when close to target
  }
  
  return power;
}

void updateHeaterControl() {
  if (!heatersEnabled) {
    // Turn off all heaters if disabled
    for (int i = 0; i < 4; i++) {
      setHeaterState(i, false);
    }
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Pulse width modulation for temperature control
  // Use heaterPower to determine on/off ratio within HEATER_PULSE_INTERVAL
  unsigned long pulsePosition = currentTime % HEATER_PULSE_INTERVAL;
  unsigned long onTime = (unsigned long)(HEATER_PULSE_INTERVAL * heaterPower);
  
  bool shouldBeOn = pulsePosition < onTime;
  
  // Safety check - limit maximum continuous on time
  if (shouldBeOn) {
    if (heaterOnTime == 0) {
      heaterOnTime = currentTime; // Start timing
    } else if (currentTime - heaterOnTime > MAX_HEATER_ON_TIME) {
      shouldBeOn = false; // Force off after max time
      Serial.println("WARNING: Heater safety timeout - forcing OFF");
    }
  } else {
    heaterOnTime = 0; // Reset timer when off
  }
  
  // Apply the same control to all 4 heaters (can be modified for individual control)
  for (int i = 0; i < 4; i++) {
    setHeaterState(i, shouldBeOn);
  }
  
  lastHeaterUpdate = currentTime;
}

void setHeaterState(int heaterIndex, bool state) {
  if (heaterIndex < 0 || heaterIndex > 3) return;
  
  int pin;
  switch (heaterIndex) {
    case 0: pin = HEATER_1_PIN; break;
    case 1: pin = HEATER_2_PIN; break;
    case 2: pin = HEATER_3_PIN; break;
    case 3: pin = HEATER_4_PIN; break;
    default: return;
  }
  
  digitalWrite(pin, state ? HIGH : LOW);
  heaterState[heaterIndex] = state;
}

void emergencyShutdown() {
  Serial.println("EMERGENCY SHUTDOWN - Temperature too high!");
  
  // Turn off all heaters immediately
  heatersEnabled = false;
  for (int i = 0; i < 4; i++) {
    setHeaterState(i, false);
  }
  
  // Reset control variables
  heaterPower = 0;
  heaterOnTime = 0;
  
  // Visual indication on display would be good here
  Serial.print("Current temperature: ");
  Serial.print(currentTemperature);
  Serial.print("°C exceeds safety limit of ");
  Serial.print(SAFETY_MAX_TEMP);
  Serial.println("°C");
}


