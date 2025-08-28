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

// Temperature sensing (dual thermistors)
#define THERMISTOR_1_PIN A0         // Primary thermistor (existing)
#define THERMISTOR_2_PIN A1         // Secondary thermistor (new)
#define THERMISTOR_1_SERIES_RESISTOR 4660 // Measured 4.7k (each thermistor needs its own)
#define THERMISTOR_2_SERIES_RESISTOR 4610 // Measured 4.7k (each thermistor needs its own)
#define THERMISTOR_NOMINAL 100000   // 100k unmeasured
#define TEMPERATURE_NOMINAL 25
#define B_COEFFICIENT 3950

// User control potentiometers
#define SPEED_POT_PIN A2            // Speed control potentiometer
#define TEMP_POT_PIN A3             // Temperature control potentiometer

// Analog input filtering settings (minimum delta approach - no smoothing)
#define SPEED_POT_MIN_DELTA 0             // Speed pot: minimum 8 ADC count change to update output
                                          // Adjust: 4 = very sensitive, 8 = balanced, 15 = stable
                                          
#define TEMP_POT_MIN_DELTA 0             // Temp pot: minimum 20 ADC count change to update output  
                                          // Adjust: 5 = sensitive, 10 = balanced, 20 = very stable
                                          // Increased due to electrical noise from PWM heaters

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

// Heater control outputs (PWM pins for MOSFET control)
#define HEATER_1_PIN 2              // Primary heater control (Timer 3B - 30Hz PWM)
#define HEATER_2_PIN 3              // Secondary heater control (Timer 3C - 30Hz PWM)
#define HEATER_3_PIN 4              // Tertiary heater control (Timer 0B - default PWM)
#define HEATER_4_PIN 5              // Quaternary heater control (Timer 3A - 30Hz PWM)
#define NUM_HEATERS 2               // Number of connected heaters

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
// STEPPER MOTOR CONTROL (NEMA 17 + L298N)
// ========================================
#define STEPPER_TORQUE_PERCENT 50  // Stepper holding torque (50-100%) - configurable
#define MAX_SPEED_RPM 300          // Maximum stepper speed (NEMA 17 typical max)
#define MIN_SPEED_RPM 1            // Minimum reliable speed
#define STEPS_PER_REVOLUTION 200   // NEMA 17: 1.8° per step = 200 steps/rev
#define MICROSTEPS 1               // Full steps (no microstepping)

// L298N controlling NEMA 17 Stepper (4-wire bipolar)
#define STEPPER_ENA_PIN 6          // Enable A (coil A enable)
#define STEPPER_IN1_PIN 7          // Coil A+ 
#define STEPPER_IN2_PIN 8          // Coil A-
#define STEPPER_IN3_PIN 9          // Coil B+
#define STEPPER_IN4_PIN 10         // Coil B-
#define STEPPER_ENB_PIN 11         // Enable B (coil B enable)

// Stepper control settings
#define STEPPER_FORWARD true
#define STEPPER_REVERSE false
#define STEPPER_HOLD_MODE true     // Keep coils energized when stopped (holding torque)

// Temperature control settings
#define MIN_TARGET_TEMP 10         // Minimum target temperature to enable heaters (prevents accidental heating)
#define MAX_TARGET_TEMP 300
#define TEMP_TOLERANCE 2           // Temperature tolerance in degrees
#define HEATER_PWM_FREQUENCY_HZ 30 // PWM frequency for MOSFET control (configurable)
#define HEATER_PWM_PERIOD_MS (1000 / HEATER_PWM_FREQUENCY_HZ) // Calculate period from frequency
#define MAX_HEATER_ON_TIME 30000   // Maximum continuous heater on time (30 seconds)
#define SAFETY_MAX_TEMP 350        // Safety shutdown temperature
#define MAX_DUTY_CYCLE 0.95        // Maximum heater duty cycle (95% - prevents 100% on time)

// PID Control Constants (Tuned for thermal systems to reduce overshoot)
#define PID_KP 0.4                 // Proportional gain - reduced from 0.8 to be less aggressive
#define PID_KI 0.01                // Integral gain - slightly reduced to prevent windup
#define PID_KD 0.3                 // Derivative gain - increased from 0.1 to dampen overshoot
#define PID_SAMPLE_TIME_MS 1000    // PID calculation interval (1 second)
#define PID_OUTPUT_MIN 0.0         // Minimum PID output (0% duty cycle)
#define PID_OUTPUT_MAX MAX_DUTY_CYCLE // Maximum PID output (95% duty cycle)
#define PID_INTEGRAL_MAX 100.0     // Maximum integral term to prevent windup

// Stepper control variables
int targetSpeed = 0;                // Target stepper speed (0-MAX_SPEED_RPM)
int currentStepperSpeed = 0;        // Current stepper speed
bool stepperDirection = STEPPER_FORWARD; // Stepper direction
bool stepperEnabled = false;        // Stepper enable state
unsigned long stepDelayMicros = 0;  // Microseconds between steps
int currentStep = 0;                // Current step position (0-3 for full-step)
unsigned long lastStepTime = 0;     // Timestamp of last step
long totalSteps = 0;                // Total steps taken (for position tracking)

int targetTemperature = 0;
float currentTemperature1 = 0;      // Primary thermistor (A0)
float currentTemperature2 = 0;      // Secondary thermistor (A1)  
float currentTemperature = 0;       // Average or primary temp (for compatibility)
float temperatureDifference = 0;    // Temp1 - Temp2 differential
bool heatersEnabled = false;
// Independent heater control variables
int heaterTargetTemp[4] = {0, 0, 0, 0};           // Individual target temps for each heater
float heaterDutyCycle[4] = {0.0, 0.0, 0.0, 0.0}; // PWM duty cycle (0.0-1.0) for each heater
bool heaterState[4] = {false, false, false, false}; // On/off state for each heater

// PID control variables for each heater
float heaterError[4] = {0, 0, 0, 0};             // Current error (setpoint - input)
float heaterErrorPrevious[4] = {0, 0, 0, 0};     // Previous error for derivative calculation
float heaterIntegral[4] = {0, 0, 0, 0};          // Integral term accumulator
float heaterDerivative[4] = {0, 0, 0, 0};        // Derivative term
float heaterPIDOutput[4] = {0, 0, 0, 0};         // Final PID output (duty cycle)
unsigned long heaterLastPIDTime[4] = {0, 0, 0, 0}; // Last PID calculation time

// Legacy compatibility variables
float temperatureError = 0;         // Primary heater error (for compatibility)
float lastTemperatureError = 0;     // Legacy
float heaterPower = 0;             // Average power across all heaters (for display)

// Create SSD1351 display object using SPI
Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

// Variables to store potentiometer values
uint16_t pot1Value = 0;
uint16_t pot2Value = 0;

// Function prototypes
void initializeI2C();
bool initializeDisplay();
void initializeHeaters();
void initializeStepper();
void configurePWMFrequency();
uint16_t readAnalogPot(int pin);
float readTemperature(int thermistorPin);
void updateTemperatureReadings();
void updateDisplay();
void scanI2CDevices();
void setHeaterState(int heaterIndex, bool state);
void setHeaterPWM(int heaterIndex, float dutyCycle);
void updateIndependentHeaterControl();
float getHeaterTemperature(int heaterIndex);
void setStepperSpeed(int speedRPM, bool direction);
void stepStepper();
void stopStepper();
void updateStepperControl();
void setStepperCoils(int step);
void emergencyShutdown();
void resetEmergencyWarning();
float calculatePID(int heaterIndex, float setpoint, float input);
void resetPID(int heaterIndex);

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
  
  Serial.println("Filament Extruder Starting...");
  
  // Initialize hardware
  delay(100);
  (void)analogRead(THERMISTOR_1_PIN);
  (void)analogRead(THERMISTOR_2_PIN);
  SPI.begin();
  
  if (!initializeDisplay()) {
    Serial.println("Display init failed");
  }
  
  // Configure PWM frequency FIRST (needed for setHeaterPWM to work)
  configurePWMFrequency();
  
  // Initialize heater control pins
  initializeHeaters();
  
  // Initialize stepper motor control
  initializeStepper();
  
  pot1Value = readAnalogPot(SPEED_POT_PIN);
  pot2Value = readAnalogPot(TEMP_POT_PIN);
  
  // Initialize PID controllers
  for (int i = 0; i < NUM_HEATERS; i++) {
    resetPID(i);
  }
  
  Serial.print("Ready - ");
  Serial.print(NUM_HEATERS);
  Serial.print(" heaters @ ");
  Serial.print(HEATER_PWM_FREQUENCY_HZ);
  Serial.print("Hz, NEMA17 stepper @ ");
  Serial.print(STEPPER_TORQUE_PERCENT);
  Serial.print("%, PID(");
  Serial.print(PID_KP);
  Serial.print(",");
  Serial.print(PID_KI);
  Serial.print(",");
  Serial.print(PID_KD);
  Serial.println(")");
}

void loop() {
  // Read analog potentiometer values
  pot1Value = readAnalogPot(SPEED_POT_PIN);
  pot2Value = readAnalogPot(TEMP_POT_PIN);

  // Map potentiometer values to control ranges with calibration
  // Option: Add dead zones if pots don't reach full 0-1023 range
  targetSpeed = map(constrain(pot1Value, 10, ADC_MAX_VALUE - 10), 10, ADC_MAX_VALUE - 10, 0, MAX_SPEED_RPM);
  targetTemperature = map(constrain(pot2Value, 10, ADC_MAX_VALUE - 10), 10, ADC_MAX_VALUE - 10, 0, MAX_TARGET_TEMP);

  // Update both temperature readings
  updateTemperatureReadings();
  
  // Apply global target temperature to individual heaters
  for (int i = 0; i < NUM_HEATERS; i++) {
    heaterTargetTemp[i] = targetTemperature;
  }
  
  // Update stepper control based on speed potentiometer
  updateStepperControl();
  
  // Update independent heater control based on their respective thermistors
  updateIndependentHeaterControl();

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
  static float lastDebugDutyCycles[4] = {-1, -1, -1, -1};
  
  // Check if any significant values have changed
  bool valuesChanged = false;
  if (abs(targetSpeed - lastDebugTargetSpeed) > 10) valuesChanged = true;           // Speed change > 10 RPM
  if (abs(targetTemperature - lastDebugTargetTemp) > 2) valuesChanged = true;       // Temp setting change > 2°C
  if (fabs(currentTemperature - lastDebugCurrentTemp) > 1.0) valuesChanged = true; // Temp reading change > 1°C
  if (fabs(heaterPower - lastDebugHeaterPower) > 0.05) valuesChanged = true;        // Power change > 5%
  
      // Check if heater states or duty cycles changed
    for (int i = 0; i < NUM_HEATERS; i++) {
      if (heaterState[i] != lastDebugHeaterStates[i]) {
        valuesChanged = true;
        break;
      }
      if (fabs(heaterDutyCycle[i] - lastDebugDutyCycles[i]) > 0.05) { // 5% duty cycle change
        valuesChanged = true;
        break;
      }
    }
  
  // Minimal debug - only major changes or every 60 seconds
  if (valuesChanged || (millis() - lastDebug > 60000)) {
    Serial.print("Stepper:");
    Serial.print(currentStepperSpeed);
    Serial.print("RPM T1:");
    Serial.print(currentTemperature1, 0);
    Serial.print("°C T2:");
    Serial.print(currentTemperature2, 0);
    Serial.print("°C -> ");
    Serial.print(targetTemperature);
    Serial.println("°C");
    
    // Update last debug values
    lastDebugTargetSpeed = targetSpeed;
    lastDebugTargetTemp = targetTemperature;
    lastDebugCurrentTemp = currentTemperature;
    lastDebugHeaterPower = heaterPower;
    for (int i = 0; i < NUM_HEATERS; i++) {
      lastDebugHeaterStates[i] = heaterState[i];
      lastDebugDutyCycles[i] = heaterDutyCycle[i];
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
  // Ring buffer size for averaging
  const int RING_BUFFER_SIZE = 8;
  
  // Static ring buffers for each pot
  static uint16_t speedPotBuffer[RING_BUFFER_SIZE] = {0};
  static uint16_t tempPotBuffer[RING_BUFFER_SIZE] = {0};
  static int speedPotIndex = 0;
  static int tempPotIndex = 0;
  static bool speedPotFilled = false;
  static bool tempPotFilled = false;
  
  // Static variables for output values
  static uint16_t lastPot1Output = 0;
  static uint16_t lastPot2Output = 0;
  static bool initialized = false;
  
  // Single ADC reading (no blocking delays)
  uint16_t rawReading = analogRead(pin);
  
  // Get per-input parameters
  uint16_t* ringBuffer;
  int* ringIndex;
  bool* ringFilled;
  uint16_t* lastOutput;
  uint16_t minDelta;
  
  if (pin == SPEED_POT_PIN) {
    ringBuffer = speedPotBuffer;
    ringIndex = &speedPotIndex;
    ringFilled = &speedPotFilled;
    lastOutput = &lastPot1Output;
    minDelta = SPEED_POT_MIN_DELTA;
  } else if (pin == TEMP_POT_PIN) {
    ringBuffer = tempPotBuffer;
    ringIndex = &tempPotIndex;
    ringFilled = &tempPotFilled;
    lastOutput = &lastPot2Output;
    minDelta = TEMP_POT_MIN_DELTA;
  } else {
    return rawReading; // Unknown pin, return raw reading
  }
  
  // Add new reading to ring buffer
  ringBuffer[*ringIndex] = rawReading;
  (*ringIndex)++;
  if (*ringIndex >= RING_BUFFER_SIZE) {
    *ringIndex = 0;
    *ringFilled = true;
  }
  
  // Calculate average from ring buffer
  uint32_t sum = 0;
  int samples = *ringFilled ? RING_BUFFER_SIZE : (*ringIndex);
  for (int i = 0; i < samples; i++) {
    sum += ringBuffer[i];
  }
  uint16_t averagedReading = sum / samples;
  
  // Initialize on first call
  if (!initialized) {
    lastPot1Output = averagedReading;
    lastPot2Output = averagedReading; 
    initialized = true;
    return averagedReading;
  }
  
  // Only update output if change is greater than minimum delta
  if (abs((int)averagedReading - (int)(*lastOutput)) >= minDelta) {
    *lastOutput = averagedReading;
  }
  
  // Always return the last reported value (creates hysteresis)
  return *lastOutput;
}

void updateDisplay() {
  // Static variables to track display state and reduce unnecessary updates
  static bool displayInitialized = false;
  static int lastTargetSpeed = -1;
  static int lastTargetTemp = -1;
  static float lastCurrentTemp1 = -999.0;
  static float lastCurrentTemp2 = -999.0;
  static float lastTempDiff = -999.0;
  static float lastHeaterPower = -1.0;
  static bool lastHeatersEnabled = false;
  static bool lastHeaterStates[4] = {false, false, false, false};
  static unsigned long lastFullRefresh = 0;
  
  // Force full refresh at configurable interval or on first run (prevents long-term display corruption)
  unsigned long currentTime = millis();
  bool forceFullRefresh = !displayInitialized || (currentTime - lastFullRefresh > DISPLAY_FULL_REFRESH_INTERVAL_MS);
  
  // Check if any values have changed significantly (increased thresholds to reduce noise-based updates)
  bool speedChanged = (abs(targetSpeed - lastTargetSpeed) > 5);        // Increased to 5 RPM to ignore noise
  bool targetTempChanged = (abs(targetTemperature - lastTargetTemp) > 2);  // Increased to 2°C to ignore noise
  bool currentTempChanged = (abs(currentTemperature1 - lastCurrentTemp1) > 0.5) || 
                           (abs(currentTemperature2 - lastCurrentTemp2) > 0.5) ||
                           (abs(temperatureDifference - lastTempDiff) > 0.3); // Any temp sensor or diff changed
  bool powerChanged = (abs(heaterPower - lastHeaterPower) > 0.03);     // Increased to 3% to ignore small power changes
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
    display.print("T1:");
    
    display.setCursor(68, 62);
    display.print("T2:");
    
    display.setCursor(5, 74);
    display.print("Diff:");
    
    display.setCursor(5, 92);
    display.print("Heaters:");
    
    display.setCursor(5, 104);
    display.print("Power:");
    
    // Draw static bar outlines (adjusted positions)
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
        if (stepperEnabled) {
      display.setTextColor(SSD1351_GREEN);
    } else {
      display.setTextColor(SSD1351_CYAN);
    }
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
  
  // Temperature 1 area (primary thermistor)
  if (currentTempChanged || forceFullRefresh) {
    display.fillRect(25, 62, 40, 8, SSD1351_BLACK); // Clear T1 area
    display.setCursor(25, 62);
    // Color code temperature based on proximity to target
    float temp1Diff = abs(currentTemperature1 - targetTemperature);
    if (temp1Diff < 5) {
      display.setTextColor(SSD1351_GREEN);  // Close to target
    } else if (temp1Diff < 15) {
      display.setTextColor(SSD1351_YELLOW); // Moderately close
    } else {
      display.setTextColor(SSD1351_RED);    // Far from target
    }
    display.print(currentTemperature1, 1);
    display.setTextColor(SSD1351_WHITE);
    display.print("C");
  }
  
  // Temperature 2 area (secondary thermistor)  
  if (currentTempChanged || forceFullRefresh) {
    display.fillRect(88, 62, 35, 8, SSD1351_BLACK); // Clear T2 area
    display.setCursor(88, 62);
    display.setTextColor(SSD1351_CYAN);
    display.print(currentTemperature2, 1);
    display.setTextColor(SSD1351_WHITE);
    display.print("C");
  }
  
  // Temperature difference area
  if (currentTempChanged || forceFullRefresh) {
    display.fillRect(35, 74, 80, 8, SSD1351_BLACK); // Clear diff area
    display.setCursor(35, 74);
    // Color code difference - green if small, yellow/red if large
    float absDiff = abs(temperatureDifference);
    if (absDiff < 2) {
      display.setTextColor(SSD1351_GREEN);      // Small difference
    } else if (absDiff < 5) {
      display.setTextColor(SSD1351_YELLOW);     // Medium difference
    } else {
      display.setTextColor(SSD1351_RED);        // Large difference
    }
    display.print(temperatureDifference, 1);
    display.setTextColor(SSD1351_WHITE);
    display.print("C");
  }
  
  // Temperature progress bar (only redraw if temperature changed) - moved down due to new layout
  if (currentTempChanged || targetTempChanged || forceFullRefresh) {
    int barY = 84;  // Moved down to accommodate T2 and Diff lines
    int barHeight = 6; // Slightly smaller
    int barWidth = SCREEN_WIDTH - 20;
    
    // Clear bar interior
    display.fillRect(11, barY+1, barWidth-2, barHeight-2, SSD1351_BLACK);
    
    int tempBarFill = 0;
    if (targetTemperature > 0) {
      // Use primary temperature (T1) for the bar
      tempBarFill = map(constrain(currentTemperature1, 0, targetTemperature), 0, targetTemperature, 0, barWidth-2);
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
    
    // Draw static bar outline
    display.drawRect(10, barY, barWidth, barHeight, SSD1351_WHITE); // Temp bar
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
  
  // Individual heater duty cycle bars (visual representation)
  if (powerChanged || heaterStateChanged || forceFullRefresh) {
    int barY = 105; // Position below power text
    int barWidth = 25; // Width per heater bar
    int barHeight = 3; // Small bars
    int barSpacing = 30; // Space between bars
    
    for (int i = 0; i < NUM_HEATERS; i++) {
      int barX = 75 + (i * barSpacing);
      
      // Clear individual bar area
      display.fillRect(barX, barY, barWidth, barHeight, SSD1351_BLACK);
      
      // Draw bar outline
      display.drawRect(barX, barY, barWidth, barHeight, SSD1351_WHITE);
      
      // Fill bar based on duty cycle
      int fillWidth = map(heaterDutyCycle[i] * 100, 0, 100, 0, barWidth - 2);
      if (fillWidth > 0) {
        uint16_t barColor;
        if (heaterDutyCycle[i] < 0.33) {
          barColor = SSD1351_GREEN;
        } else if (heaterDutyCycle[i] < 0.67) {
          barColor = SSD1351_YELLOW;
        } else {
          barColor = SSD1351_RED;
        }
        display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, barColor);
      }
    }
  }
  
  // Update last known values
  lastTargetSpeed = targetSpeed;
  lastTargetTemp = targetTemperature;
  lastCurrentTemp1 = currentTemperature1;
  lastCurrentTemp2 = currentTemperature2;
  lastTempDiff = temperatureDifference;
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

float readTemperature(int thermistorPin) {
  // Static variables for temperature denoising and 1Hz sampling - separate for each thermistor
  static float filteredADC1 = 0, filteredADC2 = 0;
  static float filteredTemperature1 = 0, filteredTemperature2 = 0;
  static bool tempInitialized1 = false, tempInitialized2 = false;
  static unsigned long lastSampleTime1 = 0, lastSampleTime2 = 0;
  static float lastTempOutput1 = 25.0, lastTempOutput2 = 25.0; // Default room temperature
  
  // Select variables based on thermistor pin
  float* filteredADC = (thermistorPin == THERMISTOR_1_PIN) ? &filteredADC1 : &filteredADC2;
  float* filteredTemperature = (thermistorPin == THERMISTOR_1_PIN) ? &filteredTemperature1 : &filteredTemperature2;
  bool* tempInitialized = (thermistorPin == THERMISTOR_1_PIN) ? &tempInitialized1 : &tempInitialized2;
  unsigned long* lastSampleTime = (thermistorPin == THERMISTOR_1_PIN) ? &lastSampleTime1 : &lastSampleTime2;
  float* lastTempOutput = (thermistorPin == THERMISTOR_1_PIN) ? &lastTempOutput1 : &lastTempOutput2;
  
  // 1Hz sampling: Only read ADC once per second to prevent self-heating
  unsigned long currentTime = millis();
  const unsigned long sampleInterval = 1000; // 1000ms = 1Hz
  
  // Check if it's time for a new sample
  if (currentTime - *lastSampleTime >= sampleInterval || !*tempInitialized) {
    // Read raw ADC value
    int rawADC = analogRead(thermistorPin);
    
    // Initialize filters on first call with actual temperature reading
    if (!*tempInitialized) {
      *filteredADC = rawADC;
      float seriesResistor = (thermistorPin == THERMISTOR_1_PIN) ? THERMISTOR_1_SERIES_RESISTOR : THERMISTOR_2_SERIES_RESISTOR;

      // Calculate actual temperature from first reading to avoid 0°C startup
      float voltage = rawADC * (VREF / (float)ADC_MAX_VALUE);
      float resistance = seriesResistor * voltage / (VREF - voltage);
      float steinhart = resistance / THERMISTOR_NOMINAL;
      steinhart = log(steinhart);
      steinhart /= B_COEFFICIENT;
      steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
      steinhart = 1.0 / steinhart;
      steinhart -= 273.15;
      
      // Initialize temperature filter with actual reading instead of 0
      *filteredTemperature = steinhart;
      *lastTempOutput = steinhart;
      
      *tempInitialized = true;
      *lastSampleTime = currentTime;
    }
    
    // First-stage filter: Smooth ADC readings (faster response for electrical noise)
    const float adcSmoothingFactor = 0.3; // More responsive than pot filters
    *filteredADC = (adcSmoothingFactor * rawADC) + ((1.0 - adcSmoothingFactor) * (*filteredADC));
    
    // Use filtered ADC value for calculations
    int smoothedRaw = (int)(*filteredADC + 0.5); // Round to nearest integer
    
    float seriesResistor = (thermistorPin == THERMISTOR_1_PIN) ? THERMISTOR_1_SERIES_RESISTOR : THERMISTOR_2_SERIES_RESISTOR;

    // Calculate resistance using VREF and series resistor values
    float voltage = smoothedRaw * (VREF / (float)ADC_MAX_VALUE);
    float resistance = seriesResistor * voltage / (VREF - voltage);
    
    // Steinhart-Hart equation
    float steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    
    // Second-stage filter: Smooth final temperature (slower, for thermal stability)
    const float tempSmoothingFactor = 0.1; // Very smooth for temperature stability
    *filteredTemperature = (tempSmoothingFactor * steinhart) + ((1.0 - tempSmoothingFactor) * (*filteredTemperature));
    
    // Temperature deadband: only significant changes update the output
    const float tempDeadband = 0.5; // Ignore changes smaller than 0.5°C
    
    if (fabs(*filteredTemperature - *lastTempOutput) > tempDeadband) {
      *lastTempOutput = *filteredTemperature;
    }
    
    // Update sample time
    *lastSampleTime = currentTime;
  }
  
  // Always return the last valid temperature (cached between samples)
  return *lastTempOutput;
}

void updateTemperatureReadings() {
  // Read both thermistors
  currentTemperature1 = readTemperature(THERMISTOR_1_PIN);
  currentTemperature2 = readTemperature(THERMISTOR_2_PIN);
  
  // Calculate temperature difference (positive = temp1 higher than temp2)
  temperatureDifference = currentTemperature1 - currentTemperature2;
  
  // Set primary temperature for existing control system (using temp1 as primary)
  currentTemperature = currentTemperature1;
  
  // Alternative: use average temperature instead
  // currentTemperature = (currentTemperature1 + currentTemperature2) / 2.0;
}

void initializeHeaters() {
  // Configure digital pins as outputs for relay control (only the ones you're using)
  if (NUM_HEATERS >= 1) pinMode(HEATER_1_PIN, OUTPUT);
  if (NUM_HEATERS >= 2) pinMode(HEATER_2_PIN, OUTPUT);
  if (NUM_HEATERS >= 3) pinMode(HEATER_3_PIN, OUTPUT);
  if (NUM_HEATERS >= 4) pinMode(HEATER_4_PIN, OUTPUT);
  
  // Initialize connected heaters to 0% PWM
  for (int i = 0; i < NUM_HEATERS; i++) {
    setHeaterPWM(i, 0.0);
  }
  

}


// PID Controller - calculates output based on setpoint, current input, and PID parameters
float calculatePID(int heaterIndex, float setpoint, float input) {
  // Validate inputs
  if (heaterIndex < 0 || heaterIndex >= NUM_HEATERS) return 0.0;
  if (isnan(setpoint) || isnan(input)) {
    Serial.print("ERROR: NaN values in PID heater ");
    Serial.println(heaterIndex);
    return 0.0;
  }
  
  // Check for unreasonable values
  if (fabs(setpoint - input) > 500.0) {
    Serial.print("ERROR: Unreasonable temperature error on heater ");
    Serial.print(heaterIndex);
    Serial.print(": ");
    Serial.println(setpoint - input);
    return 0.0;
  }
  
  unsigned long currentTime = millis();
  
  // Initialize on first run - allow immediate calculation if large error
  if (heaterLastPIDTime[heaterIndex] == 0) {
    heaterLastPIDTime[heaterIndex] = currentTime - PID_SAMPLE_TIME_MS;
    heaterErrorPrevious[heaterIndex] = 0;
    heaterIntegral[heaterIndex] = 0;
    
    // If there's a significant temperature error, calculate immediately
    float initialError = setpoint - input;
    if (fabs(initialError) > 10.0) { // >10°C error - start heating immediately
      float initialOutput = PID_KP * initialError;
      if (initialOutput > PID_OUTPUT_MAX) initialOutput = PID_OUTPUT_MAX;
      if (initialOutput < PID_OUTPUT_MIN) initialOutput = PID_OUTPUT_MIN;
      heaterPIDOutput[heaterIndex] = initialOutput;
      heaterError[heaterIndex] = initialError;
      return initialOutput;
    }
    return 0.0;
  }
  
  // Only calculate at specified sample time
  unsigned long timeChange = currentTime - heaterLastPIDTime[heaterIndex];
  if (timeChange < PID_SAMPLE_TIME_MS) {
    return heaterPIDOutput[heaterIndex]; // Return last calculated value
  }
  
  // Calculate error
  float error = setpoint - input;
  heaterError[heaterIndex] = error;
  
  // Calculate integral term with windup protection
  heaterIntegral[heaterIndex] += error * (timeChange / 1000.0); // Convert ms to seconds
  if (heaterIntegral[heaterIndex] > PID_INTEGRAL_MAX) {
    heaterIntegral[heaterIndex] = PID_INTEGRAL_MAX;
  } else if (heaterIntegral[heaterIndex] < -PID_INTEGRAL_MAX) {
    heaterIntegral[heaterIndex] = -PID_INTEGRAL_MAX;
  }
  
  // Calculate derivative term
  heaterDerivative[heaterIndex] = (error - heaterErrorPrevious[heaterIndex]) / (timeChange / 1000.0);
  
  // Calculate PID output
  float output = (PID_KP * error) + 
                 (PID_KI * heaterIntegral[heaterIndex]) + 
                 (PID_KD * heaterDerivative[heaterIndex]);
  
  // Anti-overshoot: Reduce max output when close to target
  float maxAllowedOutput = PID_OUTPUT_MAX;
  if (fabs(error) < 20.0) {  // Within 20°C of target
    // Scale max output based on error: 20°C = 100%, 5°C = 25%
    float scaleFactor = fabs(error) / 20.0;
    if (scaleFactor < 0.25) scaleFactor = 0.25; // Minimum 25% power
    maxAllowedOutput = PID_OUTPUT_MAX * scaleFactor;
  }
  
  // Clamp output to scaled range
  if (output > maxAllowedOutput) output = maxAllowedOutput;
  if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;
  
  // Store values for next iteration
  heaterErrorPrevious[heaterIndex] = error;
  heaterLastPIDTime[heaterIndex] = currentTime;
  heaterPIDOutput[heaterIndex] = output;
  
  return output;
}

// Reset PID controller for a specific heater
void resetPID(int heaterIndex) {
  if (heaterIndex < 0 || heaterIndex >= NUM_HEATERS) return;
  
  heaterError[heaterIndex] = 0;
  heaterErrorPrevious[heaterIndex] = 0;
  heaterIntegral[heaterIndex] = 0;
  heaterDerivative[heaterIndex] = 0;
  heaterPIDOutput[heaterIndex] = 0;
  heaterLastPIDTime[heaterIndex] = 0;
}



// Independent heater control - each heater controlled by its own thermistor
void updateIndependentHeaterControl() {
  unsigned long currentTime = millis();
  
  // Safety check - emergency shutdown if any temperature too high
  for (int i = 0; i < NUM_HEATERS; i++) {
    float heaterTemp = getHeaterTemperature(i);
    if (heaterTemp > SAFETY_MAX_TEMP) {
      emergencyShutdown();
      return;
    }
  }
  
  // Enable heaters if any target temperature is reasonable
  bool shouldEnable = false;
  for (int i = 0; i < NUM_HEATERS; i++) {
    if (heaterTargetTemp[i] > MIN_TARGET_TEMP && heaterTargetTemp[i] < SAFETY_MAX_TEMP) {
      shouldEnable = true;
      break;
    }
  }
  
  // Debug heater enable state changes
  if (shouldEnable != heatersEnabled) {
    Serial.print("Heaters ");
    Serial.println(shouldEnable ? "ENABLED" : "DISABLED");
  }
  heatersEnabled = shouldEnable;
  
  // Calculate average heater power for compatibility/display
  float totalPower = 0.0;
  int activeHeaters = 0;
  
  // Process each heater independently
  for (int i = 0; i < NUM_HEATERS; i++) {
    float currentTemp = getHeaterTemperature(i);
    int targetTemp = heaterTargetTemp[i];
    
    // Temperature error is now handled inside the PID controller
    
    // Enable heater only if target temperature is reasonable
    bool heaterEnabled = (targetTemp > MIN_TARGET_TEMP && targetTemp < SAFETY_MAX_TEMP && heatersEnabled);
    
    if (heaterEnabled) {
      // Calculate required heater power using PID control
      float pidOutput = calculatePID(i, targetTemp, currentTemp);
      
      // Apply PWM duty cycle
      setHeaterPWM(i, pidOutput);
      
      totalPower += pidOutput;
      activeHeaters++;
      
      // Debug: Show if heater should be working
      static unsigned long lastHeaterDebug = 0;
      if (pidOutput > 0.05 && (currentTime - lastHeaterDebug > 5000)) {
        Serial.print("Heater ");
        Serial.print(i);
        Serial.print(" ON: ");
        Serial.print(currentTemp, 1);
        Serial.print("°C -> ");
        Serial.print(targetTemp);
    Serial.print("°C @ ");
        Serial.print((int)(pidOutput * 100));
    Serial.println("%");
        lastHeaterDebug = currentTime;
      }
    } else {
      // Heater disabled - turn off and reset PID
      setHeaterPWM(i, 0.0);
      resetPID(i);
    }
  }
  
  // Update legacy heaterPower variable for display compatibility
  heaterPower = (activeHeaters > 0) ? (totalPower / activeHeaters) : 0.0;
  
    // Minimal debug output - only when significantly changed
  static unsigned long lastDebugTime = 0;
  static float lastDebugDuty[4] = {-1, -1, -1, -1};
  bool significantChange = false;
  
  // Only debug if duty cycle changed by >10% or every 30 seconds
    for (int i = 0; i < NUM_HEATERS; i++) {
    if (fabs(heaterDutyCycle[i] - lastDebugDuty[i]) > 0.1) {
      significantChange = true;
      lastDebugDuty[i] = heaterDutyCycle[i];
    }
  }
  
  if (significantChange || (currentTime - lastDebugTime > 30000)) {
    Serial.print("T:");
    for (int i = 0; i < NUM_HEATERS; i++) {
      Serial.print(getHeaterTemperature(i), 0);
      Serial.print("°C@");
      Serial.print((int)(heaterDutyCycle[i] * 100));
      Serial.print("% ");
    }
    Serial.println();
    lastDebugTime = currentTime;
  }
}

// Set heater PWM duty cycle (0.0 to 1.0)
void setHeaterPWM(int heaterIndex, float dutyCycle) {
  if (heaterIndex < 0 || heaterIndex >= NUM_HEATERS) return;
  
  // Clamp duty cycle to safe range
  if (dutyCycle < 0.0) dutyCycle = 0.0;
  if (dutyCycle > MAX_DUTY_CYCLE) dutyCycle = MAX_DUTY_CYCLE;
  
  int pin;
  switch (heaterIndex) {
    case 0: pin = HEATER_1_PIN; break;  // Pin 2
    case 1: pin = HEATER_2_PIN; break;  // Pin 3
    case 2: pin = HEATER_3_PIN; break;  // Pin 4
    case 3: pin = HEATER_4_PIN; break;  // Pin 5
    default: return;
  }
  
  // Store values for tracking
  heaterDutyCycle[heaterIndex] = dutyCycle;
  heaterState[heaterIndex] = (dutyCycle > 0.01); // Consider >1% as "on"
  
  // Apply PWM using appropriate method based on pin/timer
  if (pin == 2) {
    // Pin 2 -> Timer 3, Channel B (OCR3B)
    OCR3B = (uint16_t)(dutyCycle * ICR3);
  }
  else if (pin == 3) {
    // Pin 3 -> Timer 3, Channel C (OCR3C)  
    OCR3C = (uint16_t)(dutyCycle * ICR3);
  }
  else if (pin == 4) {
    // Pin 4 -> Timer 0 (avoid modifying timer, use standard analogWrite)
    analogWrite(pin, (int)(dutyCycle * 255.0));
  }
  else if (pin == 5) {
    // Pin 5 -> Timer 3, Channel A (OCR3A)
    OCR3A = (uint16_t)(dutyCycle * ICR3);
  }
  else {
    // Fallback for any other pins
    analogWrite(pin, (int)(dutyCycle * 255.0));
  }
}

// Get temperature reading for specific heater's thermistor
float getHeaterTemperature(int heaterIndex) {
  switch (heaterIndex) {
    case 0: return currentTemperature1; // Heater 1 -> Thermistor 1
    case 1: return currentTemperature2; // Heater 2 -> Thermistor 2
    case 2: return currentTemperature1; // Heater 3 -> Thermistor 1 (fallback)
    case 3: return currentTemperature2; // Heater 4 -> Thermistor 2 (fallback)
    default: return currentTemperature1;
  }
}

// Legacy function for compatibility - now uses PWM internally
void setHeaterState(int heaterIndex, bool state) {
  // Convert boolean state to PWM duty cycle
  float dutyCycle = state ? heaterDutyCycle[heaterIndex] : 0.0;
  setHeaterPWM(heaterIndex, dutyCycle);
}

// Global emergency warning flag
static bool emergencyWarningShown = false;

void resetEmergencyWarning() {
  emergencyWarningShown = false;
}

// Initialize NEMA 17 stepper with L298N driver
void initializeStepper() {
  // Configure stepper control pins as outputs
  pinMode(STEPPER_ENA_PIN, OUTPUT);
  pinMode(STEPPER_IN1_PIN, OUTPUT);
  pinMode(STEPPER_IN2_PIN, OUTPUT);
  pinMode(STEPPER_IN3_PIN, OUTPUT);
  pinMode(STEPPER_IN4_PIN, OUTPUT);
  pinMode(STEPPER_ENB_PIN, OUTPUT);
  
  // Initialize stepper to stopped state
  stopStepper();
  
  Serial.print("NEMA 17 stepper initialized on pins ");
  Serial.print(STEPPER_ENA_PIN);
  Serial.print("-");
  Serial.print(STEPPER_ENB_PIN);
  Serial.print(", ");
  Serial.print(STEPS_PER_REVOLUTION);
  Serial.print(" steps/rev, Torque: ");
  Serial.print(STEPPER_TORQUE_PERCENT);
  Serial.println("%");
}

// Set stepper speed and direction
void setStepperSpeed(int speedRPM, bool direction) {
  // Clamp speed to valid range
  if (speedRPM < 0) speedRPM = 0;
  if (speedRPM > MAX_SPEED_RPM) speedRPM = MAX_SPEED_RPM;
  
  currentStepperSpeed = speedRPM;
  stepperDirection = direction;
  stepperEnabled = (speedRPM > 0);
  
  if (stepperEnabled && speedRPM >= MIN_SPEED_RPM) {
    // Calculate step delay from RPM
    // RPM -> steps per second -> microseconds per step
    long stepsPerSecond = (long)speedRPM * STEPS_PER_REVOLUTION / 60;
    if (stepsPerSecond > 0) {
      stepDelayMicros = 1000000UL / stepsPerSecond;
    } else {
      stepDelayMicros = 1000000UL; // 1 second default for very slow speeds
    }
    
    // Enable coil drivers with torque limiting
    int torqueValue = (255 * STEPPER_TORQUE_PERCENT) / 100;
    analogWrite(STEPPER_ENA_PIN, torqueValue);
    analogWrite(STEPPER_ENB_PIN, torqueValue);
  } else {
    stepperEnabled = false;
    stepDelayMicros = 0;
  }
}

// Full-step sequence for bipolar stepper (4 steps per cycle)
void setStepperCoils(int step) {
  switch (step % 4) {
    case 0: // Step 1: A+, B-
      digitalWrite(STEPPER_IN1_PIN, HIGH);
      digitalWrite(STEPPER_IN2_PIN, LOW);
      digitalWrite(STEPPER_IN3_PIN, LOW);
      digitalWrite(STEPPER_IN4_PIN, HIGH);
      break;
    case 1: // Step 2: A+, B+
      digitalWrite(STEPPER_IN1_PIN, HIGH);
      digitalWrite(STEPPER_IN2_PIN, LOW);
      digitalWrite(STEPPER_IN3_PIN, HIGH);
      digitalWrite(STEPPER_IN4_PIN, LOW);
      break;
    case 2: // Step 3: A-, B+
      digitalWrite(STEPPER_IN1_PIN, LOW);
      digitalWrite(STEPPER_IN2_PIN, HIGH);
      digitalWrite(STEPPER_IN3_PIN, HIGH);
      digitalWrite(STEPPER_IN4_PIN, LOW);
      break;
    case 3: // Step 4: A-, B-
      digitalWrite(STEPPER_IN1_PIN, LOW);
      digitalWrite(STEPPER_IN2_PIN, HIGH);
      digitalWrite(STEPPER_IN3_PIN, LOW);
      digitalWrite(STEPPER_IN4_PIN, HIGH);
      break;
  }
}

// Execute one step if timing is right
void stepStepper() {
  if (!stepperEnabled || stepDelayMicros == 0) return;
  
  unsigned long currentTime = micros();
  if (currentTime - lastStepTime >= stepDelayMicros) {
    // Advance or retreat step based on direction
    if (stepperDirection == STEPPER_FORWARD) {
      currentStep++;
      totalSteps++;
    } else {
      currentStep--;
      totalSteps--;
    }
    
    // Keep currentStep in 0-3 range
    currentStep = ((currentStep % 4) + 4) % 4;
    
    // Set coil states for this step
    setStepperCoils(currentStep);
    
    lastStepTime = currentTime;
  }
}

// Stop stepper motor
void stopStepper() {
  stepperEnabled = false;
  currentStepperSpeed = 0;
  stepDelayMicros = 0;
  
  if (STEPPER_HOLD_MODE) {
    // Hold position with reduced torque (50% of configured torque)
    int holdTorque = (255 * STEPPER_TORQUE_PERCENT) / 200; // Half torque for holding
    analogWrite(STEPPER_ENA_PIN, holdTorque);
    analogWrite(STEPPER_ENB_PIN, holdTorque);
    // Keep current coil state for holding
  } else {
    // Disable all coils (no holding torque)
    analogWrite(STEPPER_ENA_PIN, 0);
    analogWrite(STEPPER_ENB_PIN, 0);
    digitalWrite(STEPPER_IN1_PIN, LOW);
    digitalWrite(STEPPER_IN2_PIN, LOW);
    digitalWrite(STEPPER_IN3_PIN, LOW);
    digitalWrite(STEPPER_IN4_PIN, LOW);
  }
}

// Update stepper control based on speed potentiometer
void updateStepperControl() {
  // Set stepper speed based on target speed from potentiometer  
  setStepperSpeed(targetSpeed, STEPPER_FORWARD);
  
  // Execute step if it's time
  stepStepper();
  
  // Debug output for stepper changes
  static int lastDebugSpeed = -1;
  static unsigned long lastStepperDebug = 0;
  
  bool speedChanged = (abs(currentStepperSpeed - lastDebugSpeed) > 10); // 10 RPM threshold
  
  if (speedChanged || (millis() - lastStepperDebug > 15000)) { // Every 15 seconds
    if (currentStepperSpeed > 0) {
      Serial.print("Stepper: ");
      Serial.print(currentStepperSpeed);
      Serial.print(" RPM (");
      Serial.print(stepDelayMicros);
      Serial.print("μs/step, ");
      Serial.print(totalSteps);
      Serial.print(" total steps, ");
      Serial.print(STEPPER_TORQUE_PERCENT);
      Serial.println("% torque)");
    } else {
      Serial.println("Stepper: STOPPED");
    }
    
    lastDebugSpeed = currentStepperSpeed;
    lastStepperDebug = millis();
  }
}

// Configure PWM frequency for MOSFET control
void configurePWMFrequency() {
  const uint16_t prescaler = 64;
  const uint16_t top_value = (F_CPU / (prescaler * HEATER_PWM_FREQUENCY_HZ)) - 1;
  
  // Set Timer 3 to Fast PWM mode with ICR3 as TOP
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31) | _BV(CS30);
  
  ICR3 = top_value;
  
  // Initialize all PWM channels to 0
  OCR3A = 0;  // Pin 5
  OCR3B = 0;  // Pin 2  
  OCR3C = 0;  // Pin 3
  
  if (NUM_HEATERS > 2) {
    Serial.println("Note: Pin 4 uses default PWM frequency");
  }
}

void emergencyShutdown() {
  if (!emergencyWarningShown) {
    Serial.println("EMERGENCY SHUTDOWN - Temperature too high!");
    Serial.print("Current temperatures: T1:");
    Serial.print(currentTemperature1);
    Serial.print("°C T2:");
    Serial.print(currentTemperature2);
    Serial.print("°C (Primary:");
    Serial.print(currentTemperature);
    Serial.print("°C) exceeds safety limit of ");
    Serial.print(SAFETY_MAX_TEMP);
    Serial.println("°C");
    emergencyWarningShown = true; // Only show emergency message once
  }
  
  // Turn off all heaters immediately
  heatersEnabled = false;
  for (int i = 0; i < NUM_HEATERS; i++) {
    setHeaterState(i, false);
  }
  
  // Stop stepper immediately
  stopStepper();
  
  // Reset control variables
  heaterPower = 0;
}
