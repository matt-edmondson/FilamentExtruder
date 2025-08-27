#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <math.h>

// ========================================
// ARDUINO MEGA 2560 R3 PIN CONFIGURATION
// ========================================

// Board configuration
#define VREF 5.0  // Arduino Mega 2560 uses 5V reference
#define ADC_BIT_DEPTH 10
#define ADC_MAX_VALUE ((1 << ADC_BIT_DEPTH) - 1)

// ========================================
// ANALOG INPUT PINS
// ========================================

// Temperature sensing
#define THERMISTOR_PIN A0           // Analog pin for thermistor reading
#define THERMISTOR_SERIES_RESISTOR 4660 // Measured 4.7k
#define THERMISTOR_NOMINAL 100000   // 100k unmeasured
#define TEMPERATURE_NOMINAL 25
#define B_COEFFICIENT 3950

// User control potentiometers
#define SPEED_POT_PIN A1            // Speed control potentiometer
#define TEMP_POT_PIN A2             // Temperature control potentiometer

// ========================================
// DIGITAL I/O PINS
// ========================================

// I2C Communication (Hardware I2C on Mega)
#define I2C_SDA_PIN 20              // Hardware SDA pin on Mega 2560
#define I2C_SCL_PIN 21              // Hardware SCL pin on Mega 2560

// SPI Communication (Hardware SPI on Mega)
#define SPI_MOSI_PIN 51             // Hardware MOSI/SDA pin on Mega 2560
#define SPI_SCK_PIN 52              // Hardware SCK/SCL pin on Mega 2560
#define SPI_MISO_PIN 50             // Hardware MISO pin on Mega 2560 (not used for display)

// SSD1351 Display Control Pins
#define DISPLAY_CS_PIN 53           // Chip Select for SSD1351 display
#define DISPLAY_DC_PIN 48           // Data/Command pin for SSD1351 display  
#define DISPLAY_RST_PIN 49          // Reset pin for SSD1351 display

// Heater control outputs (using PWM-capable pins for future flexibility)
#define HEATER_1_PIN 2              // Primary heater control (PWM capable)
#define HEATER_2_PIN 3              // Secondary heater control (PWM capable)
#define HEATER_3_PIN 4              // Tertiary heater control (PWM capable)
#define HEATER_4_PIN 5              // Quaternary heater control (PWM capable)
#define NUM_HEATERS 2               // Number of connected heaters
#define HEATER_OFF HIGH             // Relay OFF state (assuming active-low relays)
#define HEATER_ON LOW               // Relay ON state

// ========================================
// DISPLAY CONFIGURATION
// ========================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128           // SSD1351 is 128x128

// Display refresh settings
#define DISPLAY_UPDATE_INTERVAL_MS 300            // Display update rate: 50ms = 20Hz, 100ms = 10Hz, 16ms = 60Hz
                                                 // Adjust: 16ms = 60Hz (very smooth), 50ms = 20Hz (smooth), 100ms = 10Hz (efficient)
#define DISPLAY_FULL_REFRESH_INTERVAL_MS 300000  // Full screen refresh every 5 minutes (300000ms)
                                                 // Adjust: 60000 = 1min, 300000 = 5min, 600000 = 10min

// SSD1351 Color Definitions (16-bit RGB565)
#define SSD1351_BLACK 0x0000
#define SSD1351_BLUE 0x001F
#define SSD1351_RED 0xF800
#define SSD1351_GREEN 0x07E0
#define SSD1351_CYAN 0x07FF
#define SSD1351_MAGENTA 0xF81F
#define SSD1351_YELLOW 0xFFE0  
#define SSD1351_WHITE 0xFFFF

// Legacy I2C addresses (kept for potential fallback)
#define I2C_SCREEN_ADDRESS1 0x3C
#define I2C_SCREEN_ADDRESS2 0x3D

// ========================================
// MOTOR CONTROL SETTINGS
// ========================================
#define TORQUE 1000
#define MAX_SPEED_RPM 1000

// Temperature control settings
#define MIN_TARGET_TEMP 10         // Minimum target temperature to enable heaters (prevents accidental heating)
#define MAX_TARGET_TEMP 300
#define TEMP_TOLERANCE 2           // Temperature tolerance in degrees
#define HEATER_PULSE_INTERVAL 1000 // Pulse interval in milliseconds
#define MAX_HEATER_ON_TIME 30000   // Maximum continuous heater on time (30 seconds)
#define SAFETY_MAX_TEMP 350        // Safety shutdown temperature
#define MAX_DUTY_CYCLE 0.95        // Maximum heater duty cycle (95% - prevents 100% on time)

// Control variables
int targetSpeed = 0;
int targetTemperature = 0;
float currentTemperature = 0;
bool heatersEnabled = false;
unsigned long lastHeaterUpdate = 0;
unsigned long heaterOnTime = 0;
bool heaterState[4] = {false, false, false, false};  // Support up to 4, use NUM_HEATERS

// PID-like control variables
float temperatureError = 0;
float lastTemperatureError = 0;
float heaterPower = 0; // 0.0 to 1.0
unsigned long lastTempUpdate = 0;

// Create SSD1351 display object using SPI
Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

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
void resetEmergencyWarning();
void resetSafetyTimeout();
float calculateHeaterPower(float targetTemp, float currentTemp);
void updateTemperatureControl();

void setup() {
  Serial.begin(115200);
  
  // Validate configuration
  #if NUM_HEATERS < 1 || NUM_HEATERS > 4
    #error "NUM_HEATERS must be between 1 and 4"
  #endif
  
  // Wait for serial connection to be established (max 10 seconds)
  // This ensures we don't miss initial log messages
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 10000)) {
    delay(100);  // Small delay to prevent tight loop
  }
  
  Serial.println("Arduino Mega 2560 R3 Filament Extruder Controller Starting...");
  
  // Arduino Mega 2560 has fixed 10-bit ADC - no analogReadResolution() needed
  delay(100); // Wait for ADC to settle
  (void)analogRead(THERMISTOR_PIN); // Throw away first reading
 
  // Initialize SPI for display communication
  SPI.begin();
  Serial.println("SPI initialized for display communication");
  
  // Initialize display
  if (!initializeDisplay()) {
    Serial.println("SSD1351 display initialization failed - continuing without display");
  }
  
  // Initialize heater control pins
  initializeHeaters();
  
  // Test analog potentiometer reading
  Serial.println("Reading analog potentiometers...");
  pot1Value = readAnalogPot(SPEED_POT_PIN);
  pot2Value = readAnalogPot(TEMP_POT_PIN);
  Serial.print("Speed Pot (A1): ");
  Serial.print(pot1Value);
  Serial.print(", Temp Pot (A2): ");
  Serial.println(pot2Value);
  
  // Ensure clean start - reset any safety timeouts
  Serial.println("Clearing any previous safety timeout states...");
  
  Serial.println("Setup complete!");
  Serial.print("Heater control: Digital pins ");
  Serial.print(HEATER_1_PIN);
  Serial.print("-");
  Serial.print(HEATER_1_PIN + NUM_HEATERS - 1);
  Serial.println(" configured for relay control");
  Serial.print("Target temperature: ");
  Serial.print(targetTemperature);
  Serial.println("°C");
  Serial.println("DEBUG: Use Serial Monitor to diagnose heater issues");
}

void loop() {
  // Read analog potentiometer values
  pot1Value = readAnalogPot(SPEED_POT_PIN);
  pot2Value = readAnalogPot(TEMP_POT_PIN);

  // Map potentiometer values to control ranges with calibration
  // Option: Add dead zones if pots don't reach full 0-1023 range
  targetSpeed = map(constrain(pot1Value, 10, ADC_MAX_VALUE - 10), 10, ADC_MAX_VALUE - 10, 0, MAX_SPEED_RPM);
  targetTemperature = map(constrain(pot2Value, 10, ADC_MAX_VALUE - 10), 10, ADC_MAX_VALUE - 10, 0, MAX_TARGET_TEMP);

  currentTemperature = readTemperature();
  
  // Update temperature control system
  updateTemperatureControl();
  
  // Update heater control based on temperature
  updateHeaterControl();

  // Update display at configurable rate (reduces CPU load and SPI traffic)
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL_MS) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
  // Print debug info only when values change significantly or periodically when stable
  static unsigned long lastDebug = 0;
  static int lastDebugTargetSpeed = -1;
  static int lastDebugTargetTemp = -1; 
  static float lastDebugCurrentTemp = -999;
  static float lastDebugHeaterPower = -1;
  static bool lastDebugHeaterStates[4] = {false, false, false, false};
  
  // Check if any significant values have changed
  bool valuesChanged = false;
  if (abs(targetSpeed - lastDebugTargetSpeed) > 10) valuesChanged = true;           // Speed change > 10 RPM
  if (abs(targetTemperature - lastDebugTargetTemp) > 2) valuesChanged = true;       // Temp setting change > 2°C
  if (fabs(currentTemperature - lastDebugCurrentTemp) > 1.0) valuesChanged = true; // Temp reading change > 1°C
  if (fabs(heaterPower - lastDebugHeaterPower) > 0.05) valuesChanged = true;        // Power change > 5%
  
      // Check if heater states changed
    for (int i = 0; i < NUM_HEATERS; i++) {
      if (heaterState[i] != lastDebugHeaterStates[i]) {
        valuesChanged = true;
        break;
      }
    }
  
  // Print debug info when values change OR every 10 seconds when stable (reduce spam)
  if (valuesChanged || (millis() - lastDebug > 10000)) {
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
    Serial.print("% | Heaters (");
    Serial.print(NUM_HEATERS);
    Serial.print(" connected): ");
    for (int i = 0; i < NUM_HEATERS; i++) {
      Serial.print("H");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print(heaterState[i] ? "ON" : "OFF");
      if (i < NUM_HEATERS - 1) Serial.print(" ");
    }
    Serial.println();
    
    // Update last debug values
    lastDebugTargetSpeed = targetSpeed;
    lastDebugTargetTemp = targetTemperature;
    lastDebugCurrentTemp = currentTemperature;
    lastDebugHeaterPower = heaterPower;
    for (int i = 0; i < NUM_HEATERS; i++) {
      lastDebugHeaterStates[i] = heaterState[i];
    }
    
    lastDebug = millis();
  }
}

void initializeI2C() {
  // Arduino Mega 2560 uses hardware I2C pins (20=SDA, 21=SCL)
  // No need to set custom pins - these are the hardware I2C pins
  Wire.begin();
  
  Serial.print("I2C initialized on hardware pins (");
  Serial.print(I2C_SDA_PIN);
  Serial.print("=SDA, ");
  Serial.print(I2C_SCL_PIN);
  Serial.println("=SCL)");
}

bool initializeDisplay() {
  // Initialize SSD1351 color OLED display
  Serial.println("Initializing SSD1351 128x128 color display via SPI...");
  
  // Configure display control pins
  pinMode(DISPLAY_CS_PIN, OUTPUT);
  pinMode(DISPLAY_DC_PIN, OUTPUT);
  pinMode(DISPLAY_RST_PIN, OUTPUT);
  
  // Initialize the SSD1351 display
  display.begin();
  
  Serial.println("SSD1351 display initialized successfully");
  
  // Clear display and show startup screen
  display.fillScreen(SSD1351_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1351_WHITE);
  display.setCursor(5, 10);
  display.println("Arduino Mega 2560");
  display.setCursor(10, 25);
  display.setTextColor(SSD1351_CYAN);
  display.println("Filament Extruder");
  display.setCursor(20, 45);
  display.setTextColor(SSD1351_YELLOW);
  display.println("128x128 Color");
  display.setCursor(25, 60);
  display.setTextColor(SSD1351_GREEN);
  display.println("Initializing...");
  
  // Draw a colored border
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1351_WHITE);
  display.drawRect(1, 1, SCREEN_WIDTH-2, SCREEN_HEIGHT-2, SSD1351_BLUE);
  
  delay(2000);
  
  return true;
}

uint16_t readAnalogPot(int pin) {
  // Static variables to store filtered values for each pin
  static float filteredPot1 = 0;
  static float filteredPot2 = 0;
  static bool initialized = false;
  
  // Read raw analog value
  uint16_t rawReading = analogRead(pin);
  
  // Initialize filters on first call
  if (!initialized) {
    filteredPot1 = rawReading;
    filteredPot2 = rawReading; 
    initialized = true;
  }
  
  // Exponential moving average filter (adjustable smoothing) - optimized for UI responsiveness
  const float smoothingFactor = 0.4; // 0.05 = very smooth, 0.3 = more responsive, 0.4 = UI responsive
  
  float* currentFilter;
  if (pin == SPEED_POT_PIN) {
    currentFilter = &filteredPot1;
  } else if (pin == TEMP_POT_PIN) {
    currentFilter = &filteredPot2;
  } else {
    return rawReading; // Unknown pin, return raw reading
  }
  
  // Apply exponential smoothing: new_value = (alpha × raw) + ((1-alpha) × old_value)
  *currentFilter = (smoothingFactor * rawReading) + ((1.0 - smoothingFactor) * (*currentFilter));
  
  // Add deadband filter to eliminate tiny jitter around current position
  static uint16_t lastPot1Output = 0;
  static uint16_t lastPot2Output = 0;
  
  uint16_t* lastOutput;
  if (pin == SPEED_POT_PIN) {
    lastOutput = &lastPot1Output;
  } else {
    lastOutput = &lastPot2Output;
  }
  
  uint16_t smoothedReading = (uint16_t)(*currentFilter + 0.5); // Round to nearest integer
  
  // Deadband: only update output if change is significant (reduces jitter) - reduced for UI responsiveness  
  const uint16_t deadband = 1; // Ignore changes smaller than this (reduced from 3 for faster response)
  if (abs(smoothedReading - *lastOutput) > deadband) {
    *lastOutput = smoothedReading;
  }
  
  return *lastOutput;
}

void updateDisplay() {
  // Static variables to track display state and reduce unnecessary updates
  static bool displayInitialized = false;
  static int lastTargetSpeed = -1;
  static int lastTargetTemp = -1;
  static float lastCurrentTemp = -999.0;
  static float lastHeaterPower = -1.0;
  static bool lastHeatersEnabled = false;
  static bool lastHeaterStates[4] = {false, false, false, false};
  static unsigned long lastFullRefresh = 0;
  
  // Force full refresh at configurable interval or on first run (prevents long-term display corruption)
  unsigned long currentTime = millis();
  bool forceFullRefresh = !displayInitialized || (currentTime - lastFullRefresh > DISPLAY_FULL_REFRESH_INTERVAL_MS);
  
  // Check if any values have changed significantly (reduced thresholds for UI responsiveness)
  bool speedChanged = (abs(targetSpeed - lastTargetSpeed) > 2);        // Reduced from 5 to 2 RPM
  bool targetTempChanged = (abs(targetTemperature - lastTargetTemp) > 0.5);  // Reduced from 1 to 0.5°C
  bool currentTempChanged = (abs(currentTemperature - lastCurrentTemp) > 0.3); // Reduced from 0.5 to 0.3°C
  bool powerChanged = (abs(heaterPower - lastHeaterPower) > 0.01);     // Reduced from 0.02 to 0.01 (1% vs 2%)
  bool heatersEnabledChanged = (heatersEnabled != lastHeatersEnabled);
  
  bool heaterStateChanged = false;
  for (int i = 0; i < NUM_HEATERS; i++) {
    if (heaterState[i] != lastHeaterStates[i]) {
      heaterStateChanged = true;
      break;
    }
  }
  
  bool anyChanges = speedChanged || targetTempChanged || currentTempChanged || 
                   powerChanged || heatersEnabledChanged || heaterStateChanged;
  
  // Skip update if nothing changed and not forcing refresh
  if (!anyChanges && !forceFullRefresh) {
    return;
  }
  
  // Only clear screen on first initialization or major changes
  if (!displayInitialized || forceFullRefresh) {
    display.fillScreen(SSD1351_BLACK);
    
    // Draw static elements (border, labels, separator lines)
    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1351_BLUE);
    display.drawLine(5, 32, SCREEN_WIDTH-5, 32, SSD1351_CYAN);
    
    // Static title
    display.setTextSize(1);
    display.setCursor(5, 5);
    display.setTextColor(SSD1351_WHITE);
    display.print("Filament Extruder");
    
    // Static labels
    display.setCursor(5, 38);
    display.setTextColor(SSD1351_WHITE);
    display.print("Speed:");
    
    display.setCursor(5, 50);
    display.print("Target:");
    
    display.setCursor(5, 62);
    display.print("Current:");
    
    display.setCursor(5, 92);
    display.print("Heaters:");
    
    display.setCursor(5, 104);
    display.print("Power:");
    
    // Draw static bar outlines
    display.drawRect(10, 78, SCREEN_WIDTH-20, 8, SSD1351_WHITE); // Temp bar
    display.drawRect(5, 116, 62, 6, SSD1351_WHITE); // Power bar
    
    displayInitialized = true;
    lastFullRefresh = currentTime;
  }
  
  // Update dynamic content only in specific regions
  
  // Status area (18-30)
  if (heatersEnabledChanged || forceFullRefresh) {
    display.fillRect(5, 18, SCREEN_WIDTH-10, 12, SSD1351_BLACK); // Clear status area
    display.setCursor(5, 18);
    if (currentTemperature > SAFETY_MAX_TEMP - 20) {
      display.setTextColor(SSD1351_RED);
      display.print("! OVERHEATING !");
    } else if (!heatersEnabled) {
      display.setTextColor(SSD1351_YELLOW);
      display.print("STANDBY");
    } else {
      display.setTextColor(SSD1351_GREEN);
      display.print("RUNNING");
    }
  }
  
  // Speed value area
  if (speedChanged || forceFullRefresh) {
    display.fillRect(50, 38, 70, 8, SSD1351_BLACK); // Clear speed value area
    display.setCursor(50, 38);
    display.setTextColor(SSD1351_CYAN);
    display.print(targetSpeed);
    display.setTextColor(SSD1351_WHITE);
    display.print(" RPM");
  }
  
  // Target temperature area
  if (targetTempChanged || forceFullRefresh) {
    display.fillRect(55, 50, 65, 8, SSD1351_BLACK); // Clear target temp area
    display.setCursor(55, 50);
    display.setTextColor(SSD1351_YELLOW);
    display.print(targetTemperature);
    display.setTextColor(SSD1351_WHITE);
    display.print("C");
  }
  
  // Current temperature area
  if (currentTempChanged || forceFullRefresh) {
    display.fillRect(60, 62, 60, 8, SSD1351_BLACK); // Clear current temp area
    display.setCursor(60, 62);
    // Color code temperature based on proximity to target
    float tempDiff = abs(currentTemperature - targetTemperature);
    if (tempDiff < 5) {
      display.setTextColor(SSD1351_GREEN);  // Close to target
    } else if (tempDiff < 15) {
      display.setTextColor(SSD1351_YELLOW); // Moderately close
    } else {
      display.setTextColor(SSD1351_RED);    // Far from target
    }
    display.print(currentTemperature, 1);
    display.setTextColor(SSD1351_WHITE);
    display.print("C");
  }
  
  // Temperature progress bar (only redraw if temperature changed)
  if (currentTempChanged || targetTempChanged || forceFullRefresh) {
    int barY = 78;
    int barHeight = 8;
    int barWidth = SCREEN_WIDTH - 20;
    
    // Clear bar interior
    display.fillRect(11, barY+1, barWidth-2, barHeight-2, SSD1351_BLACK);
    
    int tempBarFill = 0;
    if (targetTemperature > 0) {
      tempBarFill = map(constrain(currentTemperature, 0, targetTemperature), 0, targetTemperature, 0, barWidth-2);
    }
    
    // Fill temperature bar with gradient effect
    if (tempBarFill > 0) {
      for (int x = 0; x < tempBarFill; x++) {
        uint16_t barColor;
        if (x < tempBarFill/3) {
          barColor = SSD1351_BLUE;
        } else if (x < (2*tempBarFill)/3) {
          barColor = SSD1351_YELLOW;
        } else {
          barColor = SSD1351_RED;
        }
        display.drawLine(11+x, barY+1, 11+x, barY+barHeight-2, barColor);
      }
    }
  }
  
  // Heater status area
  if (heaterStateChanged || forceFullRefresh) {
    display.fillRect(65, 92, 55, 8, SSD1351_BLACK); // Clear heater status area
    display.setCursor(65, 92);
    for (int i = 0; i < NUM_HEATERS; i++) {
      display.setTextColor(heaterState[i] ? SSD1351_RED : SSD1351_GREEN);
      display.print(heaterState[i] ? "ON " : "OFF");
      if (i < NUM_HEATERS - 1) display.print(" ");
    }
  }
  
  // Power indicator area
  if (powerChanged || forceFullRefresh) {
    display.fillRect(45, 104, 75, 8, SSD1351_BLACK); // Clear power value area
    display.setCursor(45, 104);
    display.setTextColor(SSD1351_MAGENTA);
    display.print((int)(heaterPower * 100));
    display.setTextColor(SSD1351_WHITE);
    display.print("%");
    
    // Power level bar
    display.fillRect(6, 117, 60, 4, SSD1351_BLACK); // Clear bar interior
    int powerBarWidth = map(heaterPower * 100, 0, 100, 0, 60);
    if (powerBarWidth > 0) {
      uint16_t powerColor = (heaterPower > 0.7) ? SSD1351_RED : 
                           (heaterPower > 0.4) ? SSD1351_YELLOW : SSD1351_GREEN;
      display.fillRect(6, 117, powerBarWidth, 4, powerColor);
    }
  }
  
  // Update last known values
  lastTargetSpeed = targetSpeed;
  lastTargetTemp = targetTemperature;
  lastCurrentTemp = currentTemperature;
  lastHeaterPower = heaterPower;
  lastHeatersEnabled = heatersEnabled;
  for (int i = 0; i < NUM_HEATERS; i++) {
    lastHeaterStates[i] = heaterState[i];
  }
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
      if (address == I2C_SCREEN_ADDRESS1 || address == I2C_SCREEN_ADDRESS2) {
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
    Serial.print(I2C_SCREEN_ADDRESS1, HEX);
    Serial.print("/");
    Serial.print(I2C_SCREEN_ADDRESS2, HEX);
    Serial.println(": SSD1306 Display");
    Serial.print("  - ");
    Serial.print(I2C_SDA_PIN);
    Serial.print("/");
    Serial.print(I2C_SCL_PIN);
    Serial.println(": I2C (SDA/SCL)");
    Serial.println("  - A1/A2: Analog potentiometers");  
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
}

float readTemperature() {
  // Static variables for temperature denoising and 1Hz sampling
  static float filteredADC = 0;
  static float filteredTemperature = 0;
  static bool tempInitialized = false;
  static unsigned long lastSampleTime = 0;
  static float lastTempOutput = 25.0; // Default room temperature
  
  // 1Hz sampling: Only read ADC once per second to prevent self-heating
  unsigned long currentTime = millis();
  const unsigned long sampleInterval = 1000; // 1000ms = 1Hz
  
  // Check if it's time for a new sample
  if (currentTime - lastSampleTime >= sampleInterval || !tempInitialized) {
    // Read raw ADC value
    int rawADC = analogRead(THERMISTOR_PIN);
    
    // Initialize filters on first call
    if (!tempInitialized) {
      filteredADC = rawADC;
      tempInitialized = true;
      lastSampleTime = currentTime;
    }
    
    // First-stage filter: Smooth ADC readings (faster response for electrical noise)
    const float adcSmoothingFactor = 0.3; // More responsive than pot filters
    filteredADC = (adcSmoothingFactor * rawADC) + ((1.0 - adcSmoothingFactor) * filteredADC);
    
    // Use filtered ADC value for calculations
    int smoothedRaw = (int)(filteredADC + 0.5); // Round to nearest integer
    
    // Calculate resistance using VREF and series resistor values
    float voltage = smoothedRaw * (VREF / (float)ADC_MAX_VALUE);
    float resistance = THERMISTOR_SERIES_RESISTOR * voltage / (VREF - voltage);
    
    // Steinhart-Hart equation
    float steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    
    // Second-stage filter: Smooth final temperature (slower, for thermal stability)
    const float tempSmoothingFactor = 0.1; // Very smooth for temperature stability
    filteredTemperature = (tempSmoothingFactor * steinhart) + ((1.0 - tempSmoothingFactor) * filteredTemperature);
    
    // Temperature deadband: only significant changes update the output
    const float tempDeadband = 0.5; // Ignore changes smaller than 0.5°C
    
    if (fabs(filteredTemperature - lastTempOutput) > tempDeadband) {
      lastTempOutput = filteredTemperature;
    }
    
    // Update sample time
    lastSampleTime = currentTime;
  }
  
  // Always return the last valid temperature (cached between samples)
  return lastTempOutput;
}

void initializeHeaters() {
  // Configure digital pins as outputs for relay control (only the ones you're using)
  if (NUM_HEATERS >= 1) pinMode(HEATER_1_PIN, OUTPUT);
  if (NUM_HEATERS >= 2) pinMode(HEATER_2_PIN, OUTPUT);
  if (NUM_HEATERS >= 3) pinMode(HEATER_3_PIN, OUTPUT);
  if (NUM_HEATERS >= 4) pinMode(HEATER_4_PIN, OUTPUT);
  
  // Initialize connected heaters to OFF
  if (NUM_HEATERS >= 1) digitalWrite(HEATER_1_PIN, HEATER_OFF);
  if (NUM_HEATERS >= 2) digitalWrite(HEATER_2_PIN, HEATER_OFF);
  if (NUM_HEATERS >= 3) digitalWrite(HEATER_3_PIN, HEATER_OFF);
  if (NUM_HEATERS >= 4) digitalWrite(HEATER_4_PIN, HEATER_OFF);
  
  Serial.print("Heater control pins initialized: ");
  Serial.print(NUM_HEATERS);
  Serial.print(" heater(s) connected on digital pins ");
  Serial.print(HEATER_1_PIN);
  if (NUM_HEATERS > 1) Serial.print("-");
  if (NUM_HEATERS > 1) Serial.print(HEATER_1_PIN + NUM_HEATERS - 1);
  Serial.println();
  Serial.println("All connected heaters initialized to OFF state");
  
  Serial.print("Heater pulse interval: ");
  Serial.print(HEATER_PULSE_INTERVAL);
  Serial.print("ms, Offsets for ");
  Serial.print(NUM_HEATERS);
  Serial.print(" heaters: ");
  for (int i = 0; i < NUM_HEATERS; i++) {
    Serial.print((HEATER_PULSE_INTERVAL / NUM_HEATERS) * i);
    Serial.print("ms");
    if (i < NUM_HEATERS - 1) Serial.print(", ");
  }
  Serial.println();
  Serial.print("Maximum duty cycle limited to: ");
  Serial.print(MAX_DUTY_CYCLE * 100, 1);
  Serial.println("% (prevents relay overheating)");
  Serial.println("Offset PWM: Natural sequential at low power, overlapping at high power");
  Serial.print("Temperature range: ");
  Serial.print(MIN_TARGET_TEMP);
  Serial.print("°C - ");
  Serial.print(SAFETY_MAX_TEMP);
  Serial.println("°C");
  Serial.println("Thermistor sampling: 1Hz (prevents self-heating)");
  Serial.println("Using Arduino Mega 2560 5V reference for thermistor and ADC");
}

void updateTemperatureControl() {
  // Calculate temperature error
  temperatureError = targetTemperature - currentTemperature;
  
  // Safety check - emergency shutdown if temperature too high
  if (currentTemperature > SAFETY_MAX_TEMP) {
    emergencyShutdown();
    return;
  } else {
    // Reset emergency warning when temperature is back to safe levels (with hysteresis)
    static bool wasInEmergency = false;
    if (wasInEmergency && currentTemperature < SAFETY_MAX_TEMP - 10) { // 10°C hysteresis
      resetEmergencyWarning();
      wasInEmergency = false;
      Serial.println("Temperature returned to safe levels - emergency reset");
    }
    if (currentTemperature > SAFETY_MAX_TEMP - 5) { // Set flag when approaching danger
      wasInEmergency = true;
    }
  }
  
  // Enable heaters only if target temperature is reasonable
  static bool lastHeatersEnabled = false;
  if (targetTemperature > MIN_TARGET_TEMP && targetTemperature < SAFETY_MAX_TEMP) {
    heatersEnabled = true;
  } else {
    heatersEnabled = false;
  }
  
  // Debug heaters enabled/disabled changes
  if (heatersEnabled != lastHeatersEnabled) {
    Serial.print("Heaters ");
    Serial.print(heatersEnabled ? "ENABLED" : "DISABLED");
    Serial.print(" - Target temp: ");
    Serial.print(targetTemperature);
    Serial.println("°C");
    lastHeatersEnabled = heatersEnabled;
  }
  
  // Calculate required heater power using simple proportional control
  heaterPower = calculateHeaterPower(targetTemperature, currentTemperature);
  
  lastTempUpdate = millis();
}

float calculateHeaterPower(float targetTemp, float currentTemp) {
  // Check for NaN inputs
  if (isnan(targetTemp) || isnan(currentTemp)) {
    Serial.println("ERROR: NaN temperature values detected!");
    return 0.0; // Safe fallback
  }
  
  float error = targetTemp - currentTemp;
  
  // Check for unreasonable values
  if (fabs(error) > 500.0) {
    Serial.print("ERROR: Unreasonable temperature error: ");
    Serial.println(error);
    return 0.0; // Safe fallback
  }
  
  // Simple proportional control
  float power = error / 50.0; // Scale factor - adjust as needed
  
  // Check for NaN result
  if (isnan(power)) {
    Serial.println("ERROR: Power calculation resulted in NaN!");
    return 0.0; // Safe fallback
  }
  
  // Clamp power to 0 to MAX_DUTY_CYCLE range (not full 0-1)
  if (power < 0) power = 0;
  if (power > MAX_DUTY_CYCLE) power = MAX_DUTY_CYCLE;
  
  // Dead zone around target temperature
  if (fabs(error) < TEMP_TOLERANCE) {
    power *= 0.1; // Reduce power when close to target
  }
  
  return power;
}

void updateHeaterControl() {
  // Safety timeout variables - moved to function scope so they can be reset
  static bool safetyWarningShown = false;
  static unsigned long anyHeaterOnTime = 0;
  static unsigned long safetyTimeoutStart = 0;
  
  // Debug: Print heater control status
  static unsigned long lastHeaterDebug = 0;
  if (millis() - lastHeaterDebug > 2000) { // Debug every 2 seconds
    Serial.print("HEATER DEBUG - Enabled: ");
    Serial.print(heatersEnabled ? "YES" : "NO");
    Serial.print(", Power: ");
    Serial.print(heaterPower * 100, 1);
    Serial.print("%, Target: ");
    Serial.print(targetTemperature);
    Serial.print("°C, Current: ");
    Serial.print(currentTemperature, 1);
    Serial.println("°C");
    lastHeaterDebug = millis();
  }
  
  // CRITICAL FIX: Track heater disabled state to clear safety timeouts
  static bool wasDisabled = false;
  
  if (!heatersEnabled) {
    // Turn off all connected heaters if disabled
    for (int i = 0; i < NUM_HEATERS; i++) {
      setHeaterState(i, false);
    }
    
    // Clear all safety timeout states when heaters first become disabled
    // This prevents stuck-off state when target temp is raised back up
    if (!wasDisabled) {
      // Just became disabled - clear all safety states
      anyHeaterOnTime = 0;
      safetyWarningShown = false;
      Serial.println("  -> Heaters DISABLED - clearing all safety timeout states");
      Serial.print("  -> MANUAL RESET: Turn temp pot below ");
      Serial.print(MIN_TARGET_TEMP);
      Serial.println("°C and back up to clear any stuck states");
      wasDisabled = true;
    }
    
    if (millis() - lastHeaterDebug < 100) {
      Serial.println("  -> Heaters DISABLED, forcing all OFF");
    }
    return;
  } else {
    // Reset disabled state flag when heaters are enabled again
    if (wasDisabled) {
      Serial.println("  -> Heaters RE-ENABLED - ready to heat");
      wasDisabled = false;
    }
  }
  
  unsigned long currentTime = millis();
  
  // Pulse width modulation with offset for each heater to reduce power supply stress
  // Calculate on time for PWM
  unsigned long onTime = (unsigned long)(HEATER_PULSE_INTERVAL * heaterPower);
  
  // Safety check - limit maximum continuous on time
  // (variables now declared at function start)
  bool anyHeaterOn = false;
  
  // Check safety timeout FIRST before applying heater control
  bool safetyTimeoutActive = false;
  if (anyHeaterOnTime > 0) {
    unsigned long elapsed;
    if (currentTime >= anyHeaterOnTime) {
      elapsed = currentTime - anyHeaterOnTime;
    } else {
      elapsed = (0xFFFFFFFF - anyHeaterOnTime) + currentTime + 1;
    }
    
    if (elapsed > MAX_HEATER_ON_TIME) {
      safetyTimeoutActive = true;
      if (!safetyWarningShown) {
        Serial.println("WARNING: Heater safety timeout - forcing OFF for 10 seconds");
        safetyWarningShown = true;
        safetyTimeoutStart = currentTime;
      }
    }
  }
  
  // Apply individual offset control to each connected heater (but override if safety timeout)
  for (int i = 0; i < NUM_HEATERS; i++) {
    bool heaterShouldBeOn = false;
    
    if (!safetyTimeoutActive) {
      // Offset PWM: Natural sequential at low power, overlapping at high power
      unsigned long heaterOffset = (HEATER_PULSE_INTERVAL / NUM_HEATERS) * i;
      unsigned long offsetTime = (currentTime + heaterOffset) % HEATER_PULSE_INTERVAL;
      heaterShouldBeOn = offsetTime < onTime;
    }
    // If safety timeout active, heaterShouldBeOn stays false
    
    // Track if any heater is actually on
    if (heaterShouldBeOn) {
      anyHeaterOn = true;
    }
    
    setHeaterState(i, heaterShouldBeOn);
  }
  
  // Safety timing based on any heater being on
  // (variables now declared at function start)
  
  // Debug safety timeout status and check for NaN/overflow issues
  if (millis() - lastHeaterDebug < 100) {
    Serial.print("  -> Safety: Warning=");
    Serial.print(safetyWarningShown ? "YES" : "NO");
    
    // Check for timer overflow/NaN issues
    if (anyHeaterOnTime > 0) {
      unsigned long timeDiff = currentTime - anyHeaterOnTime;
      Serial.print(", OnTime=");
      if (timeDiff > currentTime) {
        Serial.print("OVERFLOW!");
        anyHeaterOnTime = currentTime; // Fix overflow
      } else {
        Serial.print(timeDiff / 1000);
        Serial.print("s/");
        Serial.print(MAX_HEATER_ON_TIME / 1000);
        Serial.print("s");
      }
    }
    
    Serial.print(", AnyOn=");
    Serial.print(anyHeaterOn ? "YES" : "NO");
    
    // Check for NaN in heater power
    if (isnan(heaterPower)) {
      Serial.print(", POWER=NaN!");
      heaterPower = 0.0; // Fix NaN
    }
    
    Serial.println();
  }
  
  // Handle safety timeout recovery and timing
  if (anyHeaterOn && !safetyTimeoutActive) {
    if (anyHeaterOnTime == 0) {
      anyHeaterOnTime = currentTime; // Start timing
      safetyWarningShown = false; // Reset warning flag for new heating cycle
    }
    // Timer checking is now done above before heater control
  } else {
    // No heaters on OR safety timeout active
    if (safetyWarningShown) {
      // In safety timeout recovery period
      unsigned long cooldownElapsed;
      if (currentTime >= safetyTimeoutStart) {
        cooldownElapsed = currentTime - safetyTimeoutStart;
      } else {
        cooldownElapsed = (0xFFFFFFFF - safetyTimeoutStart) + currentTime + 1;
      }
      
      if (cooldownElapsed > 10000) {
        // Reset after 10 second cooldown
        anyHeaterOnTime = 0;
        safetyWarningShown = false;
        Serial.println("Safety timeout cleared - heaters can restart");
      }
    } else {
      anyHeaterOnTime = 0; // Reset timer when heaters off normally
    }
  }
  
  lastHeaterUpdate = currentTime;
}

void setHeaterState(int heaterIndex, bool state) {
  if (heaterIndex < 0 || heaterIndex >= NUM_HEATERS) return;
  
  int pin;
  switch (heaterIndex) {
    case 0: pin = HEATER_1_PIN; break;
    case 1: pin = HEATER_2_PIN; break;
    case 2: pin = HEATER_3_PIN; break;
    case 3: pin = HEATER_4_PIN; break;
    default: return;
  }
  
  digitalWrite(pin, state ? HEATER_ON : HEATER_OFF);
  heaterState[heaterIndex] = state;
}

// Global emergency warning flag
static bool emergencyWarningShown = false;

void resetEmergencyWarning() {
  emergencyWarningShown = false;
}

void resetSafetyTimeout() {
  // Reset all safety timeout variables to clear stuck state
  extern bool safetyWarningShown;
  extern unsigned long anyHeaterOnTime;
  static bool safetyTimeoutReset = false;
  
  Serial.println("MANUAL SAFETY TIMEOUT RESET");
  safetyTimeoutReset = false;
  // Note: The actual variables are static in updateHeaterControl()
  // This function mainly provides a way to force reset via serial commands
}

void emergencyShutdown() {
  if (!emergencyWarningShown) {
    Serial.println("EMERGENCY SHUTDOWN - Temperature too high!");
    Serial.print("Current temperature: ");
    Serial.print(currentTemperature);
    Serial.print("°C exceeds safety limit of ");
    Serial.print(SAFETY_MAX_TEMP);
    Serial.println("°C");
    emergencyWarningShown = true; // Only show emergency message once
  }
  
  // Turn off all heaters immediately
  heatersEnabled = false;
  for (int i = 0; i < NUM_HEATERS; i++) {
    setHeaterState(i, false);
  }
  
  // Reset control variables
  heaterPower = 0;
  heaterOnTime = 0;
}
