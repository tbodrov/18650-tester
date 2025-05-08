#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <FreeRTOS_SAMD21.h> // Include FreeRTOS for SAMD21
SemaphoreHandle_t dataMutex;

// OLED display width and height
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C; // Define the I2C address for the SSD1306 display

constexpr uint8_t THERMISTOR_PIN = A0; // Analog pin for the thermistor
constexpr uint8_t BAT_PIN_1 = A1; // Analog pin connected to battery voltage
constexpr uint8_t BAT_PIN_2 = A2; // Analog pin connected to the second battery voltage
constexpr uint8_t VREF_PIN = A4; // Analog pin connected to LM385

constexpr uint8_t BUTTON1_PIN = 6; // Button to cycle through the menu
constexpr uint8_t BUTTON2_PIN = 7; // Additional button
constexpr uint8_t BUTTON3_PIN = 8; // Additional button
constexpr uint8_t CHARGING_PIN = 2; // Pin to enable on the second page
constexpr uint8_t PWM1_PIN = 3; // Define digital pin D3 as PWM1
constexpr uint8_t PWM2_PIN = 4; // Define digital pin D4 as PWM2
constexpr uint8_t LED_RUNNING_PIN = 0; // Define digital pin D0 as LED_RUNNING
constexpr uint8_t LED_FINISHED_PIN = 1; // Define digital pin D1 as LED_FINISHED
constexpr uint8_t BUZZER_PIN = 5; // Define digital pin D5 as BUZZER

#define DEBUG_MODE  0 // Set to 1 to enable debug mode, 0 to disable

#if DEBUG_MODE 
#define TEMP_LIMIT 27.0
#define DISCHARGE_VOLTAGE_LIMIT_ENABLED 0
#define SAMPLE_INTERVAL 3UL   // 30 seconds in debug mode
#else
#define TEMP_LIMIT 45.0
#define DISCHARGE_VOLTAGE_LIMIT_ENABLED 1
#define SAMPLE_INTERVAL 300UL  // 5 minutes in normal mode
#endif

// Place the function here:
bool DischargeVoltageLimitEnabled() {
    return DISCHARGE_VOLTAGE_LIMIT_ENABLED;
}

#if DEBUG_MODE
bool debugSkipToDischarge = false;
void checkDebugSkipToDischarge(bool &chargingDone) {
    static bool lastButton3State = LOW;
    bool button3State = digitalRead(BUTTON3_PIN);
    if (button3State == HIGH && !lastButton3State) {
        debugSkipToDischarge = !debugSkipToDischarge;
        if (debugSkipToDischarge) {
            chargingDone = true;
        }
    }
    lastButton3State = button3State;
}
#endif

const int chargePins[2] = {A0, A1};  // Pins for charging batteries
const int dischargePins[2] = {A1, A1};

float BAT_Voltage1 = 0;
float BAT_Voltage2 = 0;

// Thermistor-related constants
const float BETA = 3977; // Beta coefficient for the thermistor
const float THERMISTOR_NOMINAL = 10000; // Nominal resistance at 25°C
const float NOMINAL_TEMP_KELVIN = 25 + 273.15; // Nominal temperature in Kelvin
const float SERIES_RESISTOR = 10000; // Fixed resistor value in the voltage divider

const float VREF = 2.5;          // LM385-2.5's fixed output voltage
const int NUM_SAMPLES = 100;     // Number of ADC samples for averaging

// Resistor values for the voltage divider
const float R1 = 200000.0;  // 200k ohms
const float R2 = 100000.0;  // 100k ohms

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables
float currentPWMValue = 0.0; // Rename pwmValue to currentPWMValue
float measuredVCC = 0.0;
int currentPage = 0; // Variable to track the current page
int selectedCurrent = 0; // Initial current in mA
int selectedCurrentPWM2 = 0; // Initial current for PWM2 in mA
bool selectingPWM2 = false;    // Flag to indicate if the user is selecting current for PWM2

// Add global variables to store the results of the last charge and discharge phases
float lastChargeTime = 0.0;
float lastDischargeTime = 0.0;
float lastDischargeCapacity1 = 0.0;
float lastDischargeCapacity2 = 0.0;

// Add variables to store the latest discharge voltages
float lastDischargeVoltage1 = 0.0;
float lastDischargeVoltage2 = 0.0;

// Add arrays to store voltage and time data for charge/discharge


// Store up to 120 points (2 hours, 1 per minute)
#define PHASE_LOG_POINTS 120
float chargeLogV1[PHASE_LOG_POINTS];
float chargeLogV2[PHASE_LOG_POINTS];
unsigned long chargeLogTime[PHASE_LOG_POINTS];
int chargeLogCount = 0;

// Ensure dischargeLogCount is global and not shadowed elsewhere
float dischargeLogV1[PHASE_LOG_POINTS];
float dischargeLogV2[PHASE_LOG_POINTS];
unsigned long dischargeLogTime[PHASE_LOG_POINTS];
int dischargeLogCount = 0;

// Function prototypes
float measureBatteryVoltage1();
float measureBatteryVoltage2();
float readTemperature();
float readVCC();

// Function prototypes for menu pages
void displayMainStatusPage();
void displayChargingPage();
void displayDischargingPage();
void displayCombinedProcessPage();
void displayResultsPage();
void displayCombinedPlotPage();
void displayInternalResistancePage(); // Prototype for IR test page
void displayMenuPage(int page); // Prototype for displayMenuPage

void setup() {
    Serial.begin(115200);

    // Initialize pins
    pinMode(THERMISTOR_PIN, INPUT);
    pinMode(BAT_PIN_1, INPUT);
    pinMode(BAT_PIN_2, INPUT);
    pinMode(VREF_PIN, INPUT);

    // Initialize the OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();

    analogReference(AR_DEFAULT);

    // Initialize digital buttons as pull-down
    pinMode(BUTTON1_PIN, INPUT);
    pinMode(BUTTON2_PIN, INPUT);
    pinMode(BUTTON3_PIN, INPUT);

    // Initialize the charging pin
    pinMode(CHARGING_PIN, OUTPUT);
    digitalWrite(CHARGING_PIN, LOW); // Ensure the pin is initially disabled

    // Initialize PWM1 pin
    pinMode(PWM1_PIN, OUTPUT);
    analogWrite(PWM1_PIN, 0); // Set initial PWM value to 0

    // Initialize PWM2 pin
    pinMode(PWM2_PIN, OUTPUT);
    analogWrite(PWM2_PIN, 0); // Set initial PWM value to 0

    // Initialize LEDs
    pinMode(LED_RUNNING_PIN, OUTPUT);
    pinMode(LED_FINISHED_PIN, OUTPUT);
    digitalWrite(LED_RUNNING_PIN, LOW); // Ensure LEDs are initially off
    digitalWrite(LED_FINISHED_PIN, LOW);

    // Initialize buzzer pin
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    dataMutex = xSemaphoreCreateMutex();
}

void loop() {
    static unsigned long lastDebounceTime = 0;
    static unsigned long lastUpdateTime = 0;
    static int lastPage = -1;
    static bool button1LastState = LOW;
    static unsigned long button3PressStart = 0;
    unsigned long now = millis();
    

    // Turn off LED_FINISHED when the page is switched
    if (currentPage != lastPage) {
        digitalWrite(LED_FINISHED_PIN, LOW);
        lastPage = currentPage;
    }

    // Debounce logic for BUTTON1_PIN using millis()
    bool button1CurrentState = digitalRead(BUTTON1_PIN);
    if (button1CurrentState == HIGH && button1LastState == LOW && (now - lastDebounceTime > 200)) {
        currentPage = (currentPage + 1) % 7;
        lastDebounceTime = now;
    }
    button1LastState = button1CurrentState;

    // Check if button 3 is pressed for 5 seconds to toggle between PWM1 and PWM2 current selection
    if (digitalRead(BUTTON3_PIN) == HIGH) {
        if (button3PressStart == 0) {
            button3PressStart = now;
        } else if (now - button3PressStart >= 5000) {
            selectingPWM2 = !selectingPWM2;
            button3PressStart = 0;
            lastDebounceTime = now;
        }
    } else {
        button3PressStart = 0;
    }

    // Allow user to cycle through current values for the selected PWM pin (debounced)
    static bool button3LastState = LOW;
    bool button3CurrentState = digitalRead(BUTTON3_PIN);
    if (button3CurrentState == HIGH && button3LastState == LOW && button3PressStart == 0 && (now - lastDebounceTime > 200)) {
        if (selectingPWM2) {
            selectedCurrentPWM2 += 100;
            if (selectedCurrentPWM2 > 1000) selectedCurrentPWM2 = 0; // Wrap to 0 instead of 100
        } else {
            selectedCurrent += 100;
            if (selectedCurrent > 1000) selectedCurrent = 0; // Wrap to 0 instead of 100
        }
        lastDebounceTime = now;
    }
    button3LastState = button3CurrentState;

    // Non-blocking periodic update
    
    if (now - lastUpdateTime > 1000) {
        displayMenuPage(currentPage);
        lastUpdateTime = now;
    }
}

void displayMenuPage(int page) {
    switch (page) {
        case 0:
            displayMainStatusPage();
            break;
        case 1:
            displayChargingPage();
            break;
        case 2:
            displayDischargingPage();
            break;
        case 3:
            displayCombinedProcessPage();
            break;
        case 4:
            displayInternalResistancePage(); // New IR test page
            break;
        case 5:
            displayResultsPage();
            break;
        case 6:
            displayCombinedPlotPage(); // Combined charge/discharge plot
            break;
        default:
            break;
    }
}

void printBatPin1AnalogValue(const char* label = "BAT_PIN_1") {
    int analogValue = analogRead(BAT_PIN_1);
    Serial.print(label);
    Serial.print(": ");
    Serial.println(analogValue);
}

// --- Move each case's code into its own function below ---
float measureBatteryVoltage1() {
    float Vcc = readVCC();  // Measure the actual Vcc
    float batterySum = 0;  // Sum of all the battery readings
    static int i = 0;
    static unsigned long lastReadTime = 0;
    while (i < 100) {
        if (millis() - lastReadTime >= 2) {
            batterySum += analogRead(BAT_PIN_1);  // Read raw analog value from the battery pin 100 times
            lastReadTime = millis();
            i++;
        }
    }
    float averageBatteryReading = batterySum / 100.0;  // Calculate the average battery reading
    float voltageDividerRatio = (R1 + R2) / R2;
    float batteryVoltage1 = (averageBatteryReading * Vcc / 1024.0) * voltageDividerRatio;      // Convert ADC value to battery voltage
    i = 0; // Reset for next call
    return batteryVoltage1;
}

float measureBatteryVoltage2() {
    float Vcc = readVCC();  // Measure the actual Vcc
    float batterySum = 0;  // Sum of all the battery readings
    static int i = 0;
    static unsigned long lastReadTime = 0;
    while (i < 100) {
        if (millis() - lastReadTime >= 2) {
            batterySum += analogRead(BAT_PIN_2);  // Read raw analog value from the battery pin 100 times
            lastReadTime = millis();
            i++;
        }
    }
    float averageBatteryReading = batterySum / 100.0;  // Calculate the average battery reading
    float voltageDividerRatio = (R1 + R2) / R2;
    float batteryVoltage2 = (averageBatteryReading * Vcc / 1024.0) * voltageDividerRatio;      // Convert ADC value to battery voltage
    i = 0; // Reset for next call
    return batteryVoltage2;
}

void displayMainStatusPage() {
    BAT_Voltage1 = measureBatteryVoltage1();
    BAT_Voltage2 = measureBatteryVoltage2();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.print(F("Temp: "));
    display.print(readTemperature(), 2);
    display.println(F(" C"));

    display.println(F(""));

    if (BAT_Voltage1 < 1.0 ) {
        display.print(F("V1oc: N/B\n"));
    } else {
        display.print(F("V1oc: "));
    display.print(BAT_Voltage1, 2);
    display.println(F(" V"));
    }
    
    display.println(F(""));

    if (BAT_Voltage2 < 1.0 ) {
        display.print(F("V2oc: N/B\n"));
    } else {
        display.print(F("V2oc: "));
    display.print(BAT_Voltage2, 2);
    display.println(F(" V"));
    };

    display.display();
}



//========================================= CHARGING ===============================================


void displayChargingPage() {
    display.clearDisplay();
    display.setTextSize(1);
    static bool button1LastState = LOW;
    display.setCursor(0, 0);
    display.println(F("Charge test"));
    display.println(F(""));
    display.println(F("Press Button 2"));
    display.println(F("to Start"));
    display.display();

    // Wait for BUTTON2_PIN to be pressed
    if (digitalRead(BUTTON2_PIN) == LOW) {
        return;
    }

    static unsigned long chargingStartTime = 0;
    static unsigned long elapsedTime = 0;
    static unsigned long lastLogSampleTime = 0;
    chargeLogCount = 0;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Charging..."));
    display.display();
    digitalWrite(LED_RUNNING_PIN, HIGH); // Turn on LED_RUNNING
    digitalWrite(CHARGING_PIN, HIGH);   // Enable CHARGING_PIN
    chargingStartTime = millis();
    lastLogSampleTime = chargingStartTime;

    unsigned long buttonPressStart = 0; // Track button press duration


    while (true) {
        // Check temperature
        if (readTemperature() > TEMP_LIMIT) {
            display.clearDisplay();
            display.setTextSize(1);
            int16_t x1, y1;
            uint16_t w, h;
            display.getTextBounds("Temp Exceeded 45C!", 0, 0, &x1, &y1, &w, &h);
            int16_t x = (SCREEN_WIDTH - w) / 2;
            int16_t y = (SCREEN_HEIGHT - h) / 2;
            display.setCursor(x, y);
            display.print(F("Temp Exceeded 45C!")); 
            analogWrite(PWM1_PIN, 0);// Disable PWM1_PIN
            analogWrite(PWM2_PIN, 0);// Disable PWM2_PIN
            digitalWrite(LED_RUNNING_PIN, LOW);// Turn off LED_RUNNING
            digitalWrite(LED_FINISHED_PIN, HIGH);// Turn on LED_FINISHED
            display.display();
            digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
            delay(2000);
            digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer
            delay(3000);
            break;
        }

        // Always read live voltages
        BAT_Voltage1 = measureBatteryVoltage1();
        BAT_Voltage2 = measureBatteryVoltage2();

        // Log data every minute for up to 2 hours
        unsigned long now = millis();
        if ((chargeLogCount == 0) || (now - lastLogSampleTime >= SAMPLE_INTERVAL * 1000UL)) {
            if (chargeLogCount < PHASE_LOG_POINTS) {
                chargeLogV1[chargeLogCount] = BAT_Voltage1;
                chargeLogV2[chargeLogCount] = BAT_Voltage2;
                chargeLogTime[chargeLogCount] = (now - chargingStartTime) / 1000;
                chargeLogCount++;
                lastLogSampleTime = now;
            }
        }

        // Calculate elapsed time
        elapsedTime = (millis() - chargingStartTime) / 1000;

        // Display charging information
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Charging..."));
        display.println(F(""));
        
        display.print(F("V1: "));
        if (BAT_Voltage1 < 1.0) {
            display.println(F("N/B"));
        } else {
            display.print(BAT_Voltage1, 2);
            display.println(F(" V"));
        }
        
        display.print(F("V2: "));
        if (BAT_Voltage2 < 1.0) {
            display.println(F("N/B"));
        } else {
            display.print(BAT_Voltage2, 2);
            display.println(F(" V"));
        }
        
        display.println(F(""));
        display.print(F("T: "));
        display.print(elapsedTime);
        display.println(F(" s"));
        display.println(F(""));
        display.print(F("Temp: "));
        display.print(readTemperature(), 2);
        display.println(F(" C"));
        
        display.display();

        // Stop the process if both battery voltages exceed 4.2V
        if (BAT_Voltage1 >= 4.2 && BAT_Voltage2 >= 4.2) {
            display.println(F("Charging Complete"));
            digitalWrite(CHARGING_PIN, LOW); // Disable CHARGING_PIN
            display.display();
            break;
        }

        // Check if BUTTON2_PIN is pressed for 5 seconds to interrupt the process
        if (digitalRead(BUTTON2_PIN) == HIGH) {
            if (buttonPressStart == 0) {
                buttonPressStart = millis(); // Start tracking button press time
            } else if (millis() - buttonPressStart >= 1000) { // Button held for 5 seconds
                display.clearDisplay();
                // Center "Test Interrupted" on the display
                display.setTextSize(1);
                int16_t x1, y1;
                uint16_t w, h;
                display.getTextBounds("Test Interrupted", 0, 0, &x1, &y1, &w, &h);
                int16_t x = (SCREEN_WIDTH - w) / 2;
                int16_t y = (SCREEN_HEIGHT - h) / 2;
                display.setCursor(x, y);
                display.println(F("Test Interrupted"));
                digitalWrite(CHARGING_PIN, LOW); // Disable CHARGING_PIN
                digitalWrite(LED_RUNNING_PIN, LOW); // Turn off LED_RUNNING
                display.display();
                // Ignore BUTTON2_PIN for 3 seconds after interruption
                unsigned long ignoreStart = millis();
                while (millis() - ignoreStart < 3000) {
                    // Wait and ignore BUTTON2_PIN input
                    delay(10);
                }
                break;
            }
        } else {
            buttonPressStart = 0; // Reset button press timer if button is released
        }

        // Allow page cycling during charging
        bool button1CurrentState = digitalRead(BUTTON1_PIN);
        if (button1CurrentState == HIGH && button1LastState == LOW) {
            // User wants to cycle page, break out of charging loop
            button1LastState = button1CurrentState;
            break;
        }
        button1LastState = button1CurrentState;

        display.display();
        delay(1000); // Update every second
    }

    digitalWrite(LED_RUNNING_PIN, LOW); // Turn off LED_RUNNING after process
    digitalWrite(LED_FINISHED_PIN, HIGH); // Turn on LED_FINISHED

    // --- Serial output of all collected charge data ---
    Serial.println(F("Charge Data Log:"));
    for (int i = 0; i < chargeLogCount; i++) {
        Serial.print("t=");
        Serial.print(chargeLogTime[i]);
        Serial.print("s V1=");
        Serial.print(chargeLogV1[i], 3);
        Serial.print("V V2=");
        Serial.print(chargeLogV2[i], 3);
        Serial.println("V");
    }
    Serial.print("Charge Log Count: ");
    Serial.println(chargeLogCount);
}



//========================================= DISCHARGING ============================================

void displayDischargingPage() {
    static bool button1LastState = LOW;
    display.println(F("Press Button 3 to Cycle Options"));
    display.println(F("Press Button 2 to Activate"));

    enum SelectionMode { SELECT_PWM1, SELECT_PWM2, START_PROCESS };
    static SelectionMode currentMode = SELECT_PWM1; // Initial mode
    bool button3LastState = LOW; // Track the last state of BUTTON3_PIN
    bool button2LastState = LOW; // Track the last state of BUTTON2_PIN

    static unsigned long lastLogSampleTime = 0;
    unsigned long dischargeStartTime = millis();
    

    while (true) {
        // Handle button 3 to cycle through selection modes
        bool button3CurrentState = digitalRead(BUTTON3_PIN);
        if (button3CurrentState == HIGH && button3LastState == LOW) {
            currentMode = static_cast<SelectionMode>((currentMode + 1) % 3); // Cycle through modes
        }
        button3LastState = button3CurrentState;

        // Display current mode and values
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Discharge test"));
        switch (currentMode) {
            case SELECT_PWM1:
                display.println(F("Mode: PWM1"));
                display.print(F("PWM1 Current: "));
                display.print(selectedCurrent);
                display.println(F(" mA"));
                break;
            case SELECT_PWM2:
                display.println(F("Mode: PWM2"));
                display.print(F("PWM2 Current: "));
                display.print(selectedCurrentPWM2);
                display.println(F(" mA"));
                break;
            case START_PROCESS:
                display.println(F("Mode: Start Process"));
                display.println(F("Press Button 2"));
                display.println(F("to Start"));
                break;
        }
        display.display();

        // Handle button 2 to activate the selected mode
        bool button2CurrentState = digitalRead(BUTTON2_PIN);
        if (button2CurrentState == HIGH && button2LastState == LOW) {
            if (currentMode == SELECT_PWM1) {
                selectedCurrent += 100;
                if (selectedCurrent > 1000) selectedCurrent = 0; // Wrap to 0 instead of 100
            }
            // When cycling current for PWM2
            else if (currentMode == SELECT_PWM2) {
                selectedCurrentPWM2 += 100;
                if (selectedCurrentPWM2 > 1000) selectedCurrentPWM2 = 0; // Wrap to 0 instead of 100
            } else if (currentMode == START_PROCESS) {
                // Start the discharging process
                dischargeLogCount = 0;
                display.clearDisplay();
                display.println(F("Discharging..."));
                digitalWrite(LED_RUNNING_PIN, HIGH); // Turn on LED_RUNNING

                int pwmValue1 = map(selectedCurrent, 0, 1000, 0, 255); // Map current to PWM range
                int pwmValue2 = map(selectedCurrentPWM2, 0, 1000, 0, 255);
                analogWrite(PWM1_PIN, pwmValue1);
                analogWrite(PWM2_PIN, pwmValue2);

                dischargeLogCount = 0;
                unsigned long dischargingStartTime = millis();
                lastLogSampleTime = dischargingStartTime;

                unsigned long elapsedTime = 0;
                unsigned long buttonPressStart = 0; // Declare buttonPressStart here

                while (true) {
                    unsigned long now = millis();
                    // Check temperature
                    if (readTemperature() > TEMP_LIMIT) {
                        display.clearDisplay();
                        display.setTextSize(1);
                        int16_t x1, y1;
                        uint16_t w, h;
                        display.getTextBounds("Temp Exceeded 45C!", 0, 0, &x1, &y1, &w, &h);
                        int16_t x = (SCREEN_WIDTH - w) / 2;
                        int16_t y = (SCREEN_HEIGHT - h) / 2;
                        display.setCursor(x, y);
                        display.print(F("Temp Exceeded 45C!"));
                        analogWrite(PWM1_PIN, 0);// Disable PWM1_PIN
                        analogWrite(PWM2_PIN, 0);// Disable PWM2_PIN
                        digitalWrite(LED_RUNNING_PIN, LOW);// Turn off LED_RUNNING
                        digitalWrite(LED_FINISHED_PIN, HIGH);// Turn on LED_FINISHED
                        display.display();
                        digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
                        delay(2000);
                        digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer
                        delay(3000);
                        break;
                    }

                    // Calculate elapsed time
                    elapsedTime = (millis() - dischargingStartTime) / 1000;

                    // Always read live voltages
                    float BAT_Voltage1 = measureBatteryVoltage1();
                    float BAT_Voltage2 = measureBatteryVoltage2();
                    lastDischargeVoltage1 = BAT_Voltage1;
                    lastDischargeVoltage2 = BAT_Voltage2;

                    lastDischargeTime = elapsedTime;

                    // Calculate capacity (mAh)
                    float current1 = (selectedCurrent / 1000.0); // Convert mA to A
                    float current2 = (selectedCurrentPWM2 / 1000.0); // Convert mA to A
                    float capacity1 = current1 * (elapsedTime / 3600.0); // Capacity in Ah
                    float capacity2 = current2 * (elapsedTime / 3600.0); // Capacity in Ah

                    // Store capacities for display in case 4
                    lastDischargeCapacity1 = capacity1 * 1000; // Convert to mAh
                    lastDischargeCapacity2 = capacity2 * 1000; // Convert to mAh
                    
                    // Protect discharge logging arrays with mutex
                    if ((dischargeLogCount == 0) || (now - lastLogSampleTime >= SAMPLE_INTERVAL * 1000UL)) {
                        if (dischargeLogCount < PHASE_LOG_POINTS) {
                            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                                dischargeLogV1[dischargeLogCount] = BAT_Voltage1;
                                dischargeLogV2[dischargeLogCount] = BAT_Voltage2;
                                dischargeLogTime[dischargeLogCount] = (now - dischargingStartTime) / 1000;
                                dischargeLogCount++;
                                xSemaphoreGive(dataMutex);
                            }
                            lastLogSampleTime = now;
                        }
                    }

                    // Display discharging information
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.println(F("Discharging..."));

                    // V1
                    display.print(F("V1: "));
                    if (BAT_Voltage1 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(BAT_Voltage1, 2);
                        display.println(F(" V"));
                    }

                    // V2
                    display.print(F("V2: "));
                    if (BAT_Voltage2 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(BAT_Voltage2, 2);
                        display.println(F(" V"));
                    }

                    // Time
                    display.print(F("T: "));
                    display.print(elapsedTime);
                    display.println(F(" s"));

                    // Cap1
                    display.print(F("Cap1: "));
                    if (BAT_Voltage1 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(capacity1 * 1000, 2);
                        display.println(F("mAh"));
                    }

                    // Cap2
                    display.print(F("Cap2: "));
                    if (BAT_Voltage2 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(capacity2 * 1000, 2);
                        display.println(F("mAh"));
                    }

                    // Temp
                    display.print(F("Temp: "));
                    display.print(readTemperature(), 2);
                    display.println(F(" C"));
                    display.display();


                    // Turn off PWM1 if BAT_Voltage1 <= 2.5V
                    if (DischargeVoltageLimitEnabled() && (now - dischargeStartTime > 10000)) {
                        if (BAT_Voltage1 <= 2.5) {
                            analogWrite(PWM1_PIN, 0);
                        }
                        if (BAT_Voltage2 <= 2.5) {
                            analogWrite(PWM2_PIN, 0);
                        }
                        // Finish only when BOTH PWM outputs are 0
                        if (analogRead(PWM1_PIN) == 0 && analogRead(PWM2_PIN) == 0) {
                            display.clearDisplay();
                            display.setCursor(0, 0);
                            display.setTextSize(1);
                            display.println(F("Discharge Complete"));
                            display.display();
                            digitalWrite(LED_RUNNING_PIN, LOW);
                            digitalWrite(LED_FINISHED_PIN, HIGH);
                            delay(2000);
                        
                        // --- Serial output of all collected discharge data ---
                        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                            Serial.println(F("Discharge Data Log:"));
                            for (int i = 0; i < dischargeLogCount; i++) {
                                Serial.print("t=");
                                Serial.print(dischargeLogTime[i]);
                                Serial.print("s V1=");
                                Serial.print(dischargeLogV1[i], 3);
                                Serial.print("V V2=");
                                Serial.print(dischargeLogV2[i], 3);
                                Serial.println("V");
                                Serial.print("Discharge Log Count: ");
                                Serial.println(dischargeLogCount);
                                    }
                                    xSemaphoreGive(dataMutex);
                                }
                        
                                break;
                            }
                    } 
                    // Check if BUTTON2_PIN is pressed for 5 seconds to interrupt the process
                    if (digitalRead(BUTTON2_PIN) == HIGH) {
                        if (buttonPressStart == 0) {
                            buttonPressStart = millis(); // Start tracking button press time
                        } else if (millis() - buttonPressStart >= 1000) { // Button held for 1 seconds
                            display.clearDisplay();
                            // Center "Test Interrupted" on the display
                            display.setTextSize(1);
                            int16_t x1, y1;
                            uint16_t w, h;
                            display.getTextBounds("Test Interrupted", 0, 0, &x1, &y1, &w, &h);
                            int16_t x = (SCREEN_WIDTH - w) / 2;
                            int16_t y = (SCREEN_HEIGHT - h) / 2;
                            display.setCursor(x, y);
                            display.println(F("Test Interrupted"));
                            analogWrite(PWM1_PIN, 0); // Disable PWM1_PIN
                            analogWrite(PWM2_PIN, 0); // Disable PWM2_PIN
                            digitalWrite(LED_RUNNING_PIN, LOW); // Turn off LED_RUNNING
                            display.display();
                            // --- Serial output of all collected discharge data ---
                            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                                Serial.println(F("Discharge Data Log:"));
                                for (int i = 0; i < dischargeLogCount; i++) {
                                    Serial.print("t=");
                                    Serial.print(dischargeLogTime[i]);
                                    Serial.print("s V1=");
                                    Serial.print(dischargeLogV1[i], 3);
                                    Serial.print("V V2=");
                                    Serial.print(dischargeLogV2[i], 3);
                                    Serial.println("V");
                                    Serial.print("Discharge Log Count: ");
                                    Serial.println(dischargeLogCount);
                                }
                                xSemaphoreGive(dataMutex);
                            }
                            // --- end serial output ---
                            // Ignore BUTTON2_PIN for 3 seconds after interruption
                            unsigned long ignoreStart = millis();
                            while (millis() - ignoreStart < 3000) {
                                // Wait and ignore BUTTON2_PIN input
                                delay(10);
                            }
                            break;
                        }
                    } else {
                        buttonPressStart = 0; // Reset button press timer if button is released
                    }

                    // Allow page cycling during discharging
                    bool button1CurrentState = digitalRead(BUTTON1_PIN);
                    if (button1CurrentState == HIGH && button1LastState == LOW) {
                        // User wants to cycle page, break out of discharging loop
                        button1LastState = button1CurrentState;
                        // Turn off PWM1 and PWM2 when leaving the process
                        analogWrite(PWM1_PIN, 0);
                        analogWrite(PWM2_PIN, 0);
                        // --- Serial output of all collected discharge data ---
                        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                            Serial.println(F("Discharge Data Log:"));
                            for (int i = 0; i < dischargeLogCount; i++) {
                                Serial.print("t=");
                                Serial.print(dischargeLogTime[i]);
                                Serial.print("s V1=");
                                Serial.print(dischargeLogV1[i], 3);
                                Serial.print("V V2=");
                                Serial.print(dischargeLogV2[i], 3);
                                Serial.println("V");
                                Serial.print("Discharge Log Count: ");
                                Serial.println(dischargeLogCount);
                            }
                            xSemaphoreGive(dataMutex);
                        }
                        // --- end serial output ---
                        break;
                    }
                    button1LastState = button1CurrentState;

                    delay(1000); // Update every second
                }

                // Ensure PWM1 and PWM2 are off after the discharging loop
                analogWrite(PWM1_PIN, 0);
                analogWrite(PWM2_PIN, 0);
                digitalWrite(LED_RUNNING_PIN, LOW); // Turn off LED_RUNNING after process
                digitalWrite(LED_FINISHED_PIN, HIGH); // Turn on LED_FINISHED
                break; // Exit the discharging loop
            }
        }
        button2LastState = button2CurrentState;

        // Exit the main loop if BUTTON1_PIN is pressed to switch pages
        if (digitalRead(BUTTON1_PIN) == HIGH) {
            break;
        }

        delay(100); // Small delay for button debounce
    }
}


//========================================= COMBINED PROCESS ============================================
void displayCombinedProcessPage() {
    static bool button1LastState = LOW;
    // --- Current selection UI (identical to case 2) ---
    enum SelectionMode { SELECT_PWM1, SELECT_PWM2, START_PROCESS };
    static SelectionMode currentMode = SELECT_PWM1;
    bool button3LastState = LOW;
    bool button2LastState = LOW;
    unsigned long dischargeStartTime = millis();

    while (true) {
        // Handle button 3 to cycle through selection modes
        bool button3CurrentState = digitalRead(BUTTON3_PIN);
        if (button3CurrentState == HIGH && button3LastState == LOW) {
            currentMode = static_cast<SelectionMode>((currentMode + 1) % 3);
        }
        button3LastState = button3CurrentState;

        // Display current mode and values
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Combined Test"));
        switch (currentMode) {
            case SELECT_PWM1:
                display.println(F("Mode: PWM1"));
                display.print(F("PWM1 Current: "));
                display.print(selectedCurrent);
                display.println(F(" mA"));
                break;
            case SELECT_PWM2:
                display.println(F("Mode: PWM2"));
                display.print(F("PWM2 Current: "));
                display.print(selectedCurrentPWM2);
                display.println(F(" mA"));
                break;
            case START_PROCESS:
                display.println(F("Mode: Start Process"));
                display.println(F("Press Button 2 to Start"));
                break;
        }
        display.display();

        // Handle button 2 to activate the selected mode
        bool button2CurrentState = digitalRead(BUTTON2_PIN);
        if (button2CurrentState == HIGH && button2LastState == LOW) {
            if (currentMode == SELECT_PWM1) {
                selectedCurrent += 100;
                if (selectedCurrent > 1000) selectedCurrent = 0; // Wrap to 0 instead of 100
            } else if (currentMode == SELECT_PWM2) {
                selectedCurrentPWM2 += 100;
                if (selectedCurrentPWM2 > 1000) selectedCurrentPWM2 = 0; // Wrap to 0 instead of 100
            } else if (currentMode == START_PROCESS) {
                break; // Exit selection loop and start process
            }
        }
        button2LastState = button2CurrentState;

        // Allow page cycling
        if (digitalRead(BUTTON1_PIN) == HIGH) {
            return;
        }
        delay(100);
    }

    // --- Charging phase (identical to case 1) ---
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Charging..."));
    display.display();
    digitalWrite(LED_RUNNING_PIN, HIGH);
    digitalWrite(CHARGING_PIN, HIGH);
    unsigned long chargingStartTime = millis();
    unsigned long elapsedTime = 0;
    unsigned long lastLogSampleTime = chargingStartTime;
    unsigned long buttonPressStart = 0;

   

    chargeLogCount = 0;

    bool chargingDone = false;
    while (!chargingDone) {
        if (readTemperature() > TEMP_LIMIT) {
            display.clearDisplay();
            display.setTextSize(1);
            int16_t x1, y1;
            uint16_t w, h;
            display.getTextBounds("Temp Exceeded 45C!", 0, 0, &x1, &y1, &w, &h);
            int16_t x = (SCREEN_WIDTH - w) / 2;
            int16_t y = (SCREEN_HEIGHT - h) / 2;
            display.setCursor(x, y);
            display.print(F("Temp Exceeded 45C!"));
            analogWrite(PWM1_PIN, 0);
            analogWrite(PWM2_PIN, 0);
            digitalWrite(LED_RUNNING_PIN, LOW);
            digitalWrite(LED_FINISHED_PIN, HIGH);
            display.display();
            digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
            delay(2000);
            digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer
            delay(3000);
            return;
        }

        float BAT_Voltage1 = measureBatteryVoltage1();
        float BAT_Voltage2 = measureBatteryVoltage2();

        // Log data every SAMPLE_INTERVAL seconds
        unsigned long now = millis();
        if ((chargeLogCount == 0) || (now - lastLogSampleTime >= SAMPLE_INTERVAL * 1000UL)) {
            if (chargeLogCount < PHASE_LOG_POINTS) {
                chargeLogV1[chargeLogCount] = BAT_Voltage1;
                chargeLogV2[chargeLogCount] = BAT_Voltage2;
                chargeLogTime[chargeLogCount] = (now - chargingStartTime) / 1000;
                chargeLogCount++;
                lastLogSampleTime = now;
            }
        }

        elapsedTime = (millis() - chargingStartTime) / 1000;

       

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Charging..."));
        display.println(F(""));
        
        display.print(F("V1: "));
        if (BAT_Voltage1 < 1.0) {
            display.println(F("N/B"));
        } else {
            display.print(BAT_Voltage1, 2);
            display.println(F(" V"));
        }
        
        display.print(F("V2: "));
        if (BAT_Voltage2 < 1.0) {
            display.println(F("N/B"));
        } else {
            display.print(BAT_Voltage2, 2);
            display.println(F(" V"));
        }
        
        display.println(F(""));
        display.print(F("T: "));
        display.print(elapsedTime);
        display.println(F(" s"));
        display.println(F(""));
        display.print(F("Temp: "));
        display.print(readTemperature(), 2);
        display.println(F(" C"));
        
        display.display();
        if (BAT_Voltage1 >= 4.2 && BAT_Voltage2 >= 4.2) {
            display.println(F("Charging Complete"));
            digitalWrite(CHARGING_PIN, LOW);
            chargingDone = true;
            delay(1000);
            break;
        }

        // --- DEBUG: allow skipping to discharge phase with BUTTON3 ---
        #if DEBUG_MODE
        checkDebugSkipToDischarge(chargingDone);
        if (chargingDone) {
            digitalWrite(CHARGING_PIN, LOW);
            break;
        }
        #endif

        if (digitalRead(BUTTON2_PIN) == HIGH) {
            if (buttonPressStart == 0) {
                buttonPressStart = millis();
            } else if (millis() - buttonPressStart >= 1000) {
                display.clearDisplay();
                display.setTextSize(1);
                int16_t x1, y1;
                uint16_t w, h;
                display.getTextBounds("Test Interrupted", 0, 0, &x1, &y1, &w, &h);
                int16_t x = (SCREEN_WIDTH - w) / 2;
                int16_t y = (SCREEN_HEIGHT - h) / 2;
                display.setCursor(x, y);
                display.println(F("Test Interrupted"));
                digitalWrite(CHARGING_PIN, LOW);
                digitalWrite(LED_RUNNING_PIN, LOW);
                display.display();
                // Ignore BUTTON2_PIN for 3 seconds after interruption
                unsigned long ignoreStart = millis();
                while (millis() - ignoreStart < 3000) {
                    delay(10);
                }
                return;
            }
        } else {
            buttonPressStart = 0;
        }

        bool button1CurrentState = digitalRead(BUTTON1_PIN);
        if (button1CurrentState == HIGH && button1LastState == LOW) {
            button1LastState = button1CurrentState;
            digitalWrite(CHARGING_PIN, LOW);
            return;
        }
        button1LastState = button1CurrentState;

        delay(1000);
    }

    // --- Serial output of all collected charge data ---
    Serial.println(F("Charge Data Log:"));
    for (int i = 0; i < chargeLogCount; i++) {
        Serial.print("t=");
        Serial.print(chargeLogTime[i]);
        Serial.print("s V1=");
        Serial.print(chargeLogV1[i], 3);
        Serial.print("V V2=");
        Serial.print(chargeLogV2[i], 3);
        Serial.println("V");
    }
    Serial.print("Charge Log Count: ");
    Serial.println(chargeLogCount);


    // --- Add 5 minute resting delay between charge and discharge ---
display.clearDisplay();
display.setTextSize(2);
display.setCursor(10, 25);
display.print(F("Resting..."));
display.display();

unsigned long restStart = millis();
const unsigned long restDuration = 5UL * 60UL * 1000UL; // 5 minutes in ms
while (millis() - restStart < restDuration) {
    // Optionally, show countdown
    unsigned long secondsLeft = (restDuration - (millis() - restStart)) / 1000;
    display.setTextSize(1);
    display.setCursor(10, 50);
    display.print(F("Left: "));
    display.print(secondsLeft);
    display.print(F(" s   "));
    display.display();
    delay(500);
}



    // --- Discharging phase (identical to case 2) ---
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Discharging..."));
    display.display();
    int pwmValue1 = map(selectedCurrent, 100, 1000, 25, 255);
    int pwmValue2 = map(selectedCurrentPWM2, 100, 1000, 25, 255);
    analogWrite(PWM1_PIN, pwmValue1);
    analogWrite(PWM2_PIN, pwmValue2);
    unsigned long dischargingStartTime = millis();
    unsigned long dischargeElapsedTime = 0;
    unsigned long dischargeButtonPressStart = 0;

    static unsigned long lastDischargeLogSampleTime = 0;
    dischargeLogCount = 0;
    lastDischargeLogSampleTime = dischargingStartTime;

    while (true) {

        unsigned long now = millis();
        if (readTemperature() > TEMP_LIMIT) {
            display.clearDisplay();
            display.setTextSize(1);
            int16_t x1, y1;
            uint16_t w, h;
            display.getTextBounds("Temp Exceeded 45C!", 0, 0, &x1, &y1, &w, &h);
            int16_t x = (SCREEN_WIDTH - w) / 2;
            int16_t y = (SCREEN_HEIGHT - h) / 2;
            display.setCursor(x, y);
            display.print(F("Temp Exceeded 45C!"));
            analogWrite(PWM1_PIN, 0);
            analogWrite(PWM2_PIN, 0);
            digitalWrite(LED_RUNNING_PIN, LOW);
            digitalWrite(LED_FINISHED_PIN, HIGH);
            display.display();
            digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
            delay(2000);
            digitalWrite(BUZZER_PIN, LOW);  // Deactivate buzzer
            delay(3000);
            break;
        }

        dischargeElapsedTime = (millis() - dischargingStartTime) / 1000;

        float BAT_Voltage1 = measureBatteryVoltage1();
        float BAT_Voltage2 = measureBatteryVoltage2();
        lastDischargeVoltage1 = BAT_Voltage1;
        lastDischargeVoltage2 = BAT_Voltage2;

        // Log data every SAMPLE_INTERVAL seconds

        if ((dischargeLogCount == 0) || (now - lastDischargeLogSampleTime >= SAMPLE_INTERVAL * 1000UL)) {
            if (dischargeLogCount < PHASE_LOG_POINTS) {
                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                    dischargeLogV1[dischargeLogCount] = BAT_Voltage1;
                    dischargeLogV2[dischargeLogCount] = BAT_Voltage2;
                    dischargeLogTime[dischargeLogCount] = (now - dischargingStartTime) / 1000;
                    dischargeLogCount++;
                    xSemaphoreGive(dataMutex);
                }
                lastDischargeLogSampleTime = now;
            }
        }

        float current1 = (selectedCurrent / 1000.0);
        float current2 = (selectedCurrentPWM2 / 1000.0);
        float capacity1 = current1 * (dischargeElapsedTime / 3600.0);
        float capacity2 = current2 * (dischargeElapsedTime / 3600.0);

        lastDischargeTime = dischargeElapsedTime;
        lastDischargeCapacity1 = capacity1 * 1000;
        lastDischargeCapacity2 = capacity2 * 1000;

        display.clearDisplay();
                    display.setCursor(0, 0);
                    display.println(F("Discharging..."));

                    // V1
                    display.print(F("V1: "));
                    if (BAT_Voltage1 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(BAT_Voltage1, 2);
                        display.println(F(" V"));
                    }

                    // V2
                    display.print(F("V2: "));
                    if (BAT_Voltage2 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(BAT_Voltage2, 2);
                        display.println(F(" V"));
                    }

                    // Time
                    display.print(F("T: "));
                    display.print(elapsedTime);
                    display.println(F(" s"));

                    // Cap1
                    display.print(F("Cap1: "));
                    if (BAT_Voltage1 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(capacity1 * 1000, 2);
                        display.println(F("mAh"));
                    }

                    // Cap2
                    display.print(F("Cap2: "));
                    if (BAT_Voltage2 < 1.0) {
                        display.println(F("N/B"));
                    } else {
                        display.print(capacity2 * 1000, 2);
                        display.println(F("mAh"));
                    }

                    // Temp
                    display.print(F("Temp: "));
                    display.print(readTemperature(), 2);
                    display.println(F(" C"));
                    display.display();

        // Turn off PWM1 if BAT_Voltage1 <= 2.5V
        if (DischargeVoltageLimitEnabled() && (now - dischargeStartTime > 10000)) {
            if (BAT_Voltage1 <= 2.5) {
                analogWrite(PWM1_PIN, 0);
            }
            if (BAT_Voltage2 <= 2.5) {
                analogWrite(PWM2_PIN, 0);
            }
            // Finish only when BOTH PWM outputs are 0
            if (analogRead(PWM1_PIN) == 0 && analogRead(PWM2_PIN) == 0) {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.setTextSize(1);
                display.println(F("Discharge Complete"));
                display.display();
                digitalWrite(LED_RUNNING_PIN, LOW);
                digitalWrite(LED_FINISHED_PIN, HIGH);
                delay(2000);
            // --- Serial output of all collected discharge data ---
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                Serial.println(F("Discharge Data Log:"));
                for (int i = 0; i < dischargeLogCount; i++) {
                    Serial.print("t=");
                    Serial.print(dischargeLogTime[i]);
                    Serial.print("s V1=");
                    Serial.print(dischargeLogV1[i], 3);
                    Serial.print("V V2=");
                    Serial.print(dischargeLogV2[i], 3);
                    Serial.println("V");
                    Serial.print("Discharge Log Count: ");
                    Serial.println(dischargeLogCount);
                        }
                        xSemaphoreGive(dataMutex);
                    }
            
                    break;
                }
        } 

        if (digitalRead(BUTTON2_PIN) == HIGH) {
            if (dischargeButtonPressStart == 0) {
                dischargeButtonPressStart = millis();
            } else if (millis() - dischargeButtonPressStart >= 1000) {
                display.clearDisplay();
                display.setTextSize(1);
                int16_t x1, y1;
                uint16_t w, h;
                display.getTextBounds("Test Interrupted", 0, 0, &x1, &y1, &w, &h);
                int16_t x = (SCREEN_WIDTH - w) / 2;
                int16_t y = (SCREEN_HEIGHT - h) / 2;
                display.setCursor(x, y);
                display.println(F("Test Interrupted"));
                analogWrite(PWM1_PIN, 0);
                analogWrite(PWM2_PIN, 0);
                digitalWrite(LED_RUNNING_PIN, LOW);
                display.display();
                unsigned long ignoreStart = millis();
                while (millis() - ignoreStart < 3000) {
                    delay(10);
                }
                break;
            }
        } else {
            dischargeButtonPressStart = 0;
        }

        bool button1CurrentState = digitalRead(BUTTON1_PIN);
        if (button1CurrentState == HIGH && button1LastState == LOW) {
            button1LastState = button1CurrentState;
            analogWrite(PWM1_PIN, 0);
            analogWrite(PWM2_PIN, 0);
            break;
        }
        button1LastState = button1CurrentState;

        delay(1000);
    }

    // --- Serial output of all collected discharge data ---
    Serial.println(F("Discharge Data Log:"));
    for (int i = 0; i < dischargeLogCount; i++) {
        Serial.print("t=");
        Serial.print(dischargeLogTime[i]);
        Serial.print("s V1=");
        Serial.print(dischargeLogV1[i], 3);
        Serial.print("V V2=");
        Serial.print(dischargeLogV2[i], 3);
        Serial.println("V");
    }
    Serial.print("Discharge Log Count: ");
    Serial.println(dischargeLogCount);

    analogWrite(PWM1_PIN, 0);
    analogWrite(PWM2_PIN, 0);
    digitalWrite(LED_RUNNING_PIN, LOW);
    digitalWrite(LED_FINISHED_PIN, HIGH);

    // Store the latest charge/discharge data for results/plot
    lastChargeTime = elapsedTime;
    lastDischargeTime = dischargeElapsedTime;
    lastDischargeCapacity1 = (selectedCurrent / 1000.0) * (dischargeElapsedTime / 3600.0) * 1000.0;
    lastDischargeCapacity2 = (selectedCurrentPWM2 / 1000.0) * (dischargeElapsedTime / 3600.0) * 1000.0;

    
}


//========================================= INTERNAL RESISTANCE PAGE ============================================
void displayInternalResistancePage() {
    // Always use max current for IR test
    const int irSelectedCurrent = 1000;
    const int irSelectedCurrentPWM2 = 1000;

    // Wait for user to press Button 2 to start the test
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("IR Test"));
    display.println(F("Press Button 2"));
    display.println(F("to Start"));
    display.display();

    // Wait for BUTTON2_PIN to be pressed
    while (digitalRead(BUTTON2_PIN) == LOW) {
        if (digitalRead(BUTTON1_PIN) == HIGH) return; // Allow exit
        delay(50);
    }
    // Debounce: wait for release
    while (digitalRead(BUTTON2_PIN) == HIGH) {
        if (digitalRead(BUTTON1_PIN) == HIGH) return;
        delay(10);
    }

    // --- Internal Resistance Measurement for PWM1 ---
    float voltageNoLoad1 = 0, voltageLoad1 = 0, ir1 = 0;
    int pwmValue1 = map(irSelectedCurrent, 100, 1000, 25, 255);
    analogWrite(PWM1_PIN, 0);
    delay(1000);
    voltageNoLoad1 = measureBatteryVoltage1();
    analogWrite(PWM1_PIN, pwmValue1);
    delay(1000);
    voltageLoad1 = measureBatteryVoltage1();
    float currentDrawn1 = irSelectedCurrent / 1000.0;
    if (currentDrawn1 > 0) {
        ir1 = (voltageNoLoad1 - voltageLoad1) / currentDrawn1;
    } else {
        ir1 = 0;
    }
    analogWrite(PWM1_PIN, 0);

    // --- Internal Resistance Measurement for PWM2 ---
    float voltageNoLoad2 = 0, voltageLoad2 = 0, ir2 = 0;
    int pwmValue2 = map(irSelectedCurrentPWM2, 100, 1000, 25, 255);
    analogWrite(PWM2_PIN, 0);
    delay(1000);
    voltageNoLoad2 = measureBatteryVoltage2();
    analogWrite(PWM2_PIN, pwmValue2);
    delay(1000);
    voltageLoad2 = measureBatteryVoltage2();
    float currentDrawn2 = irSelectedCurrentPWM2 / 1000.0;
    if (currentDrawn2 > 0) {
        ir2 = (voltageNoLoad2 - voltageLoad2) / currentDrawn2;
    } else {
        ir2 = 0;
    }
    analogWrite(PWM2_PIN, 0);

    // --- Display Results ---
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("IR Test Results:"));

    // BAT1
    if (voltageNoLoad1 < 1.0 || voltageLoad1 < 1.0) {
        display.print(F("BAT1: N/B\n"));
    } else {
        display.print(F("BAT1: "));
        display.print(ir1, 3);
        display.println(F(" Ohm"));
        display.print(F("Vnl:"));
        display.print(voltageNoLoad1, 3);
        display.print(F("V Vl:"));
        display.print(voltageLoad1, 3);
        display.println(F("V"));
    }
    display.println(F(""));
    // BAT2
    if (voltageNoLoad2 < 1.0 || voltageLoad2 < 1.0) {
        display.print(F("BAT2: N/B\n"));
    } else {
        display.print(F("BAT2: "));
        display.print(ir2, 3);
        display.println(F(" Ohm"));
        display.print(F("Vnl:"));
        display.print(voltageNoLoad2, 3);
        display.print(F("V Vl:"));
        display.print(voltageLoad2, 3);
        display.println(F("V"));
    }

    display.display();
    delay(20000); // Wait for user to read
}


//========================================= RESULTS PAGE ============================================
void displayResultsPage() {
    static bool button1LastState = LOW;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    // Display last charge data
    display.println(F("L Charge Results:"));
    if (chargeLogCount > 0) {
        display.print(F("T: "));
        display.print(chargeLogTime[chargeLogCount - 1]);
        display.println(F("s"));
        display.print(F("V1:"));
        display.print(chargeLogV1[chargeLogCount - 1], 2);
        display.print(F("V V2:"));
        display.print(chargeLogV2[chargeLogCount - 1], 2);
        display.println(F("V"));
    } else {
        display.println(F("No Charge Data Available"));
    }

    // Display last discharge data (latest voltages)
    display.println();
    display.println(F("L Discharge Results:"));
    if (lastDischargeTime > 0) {
        display.print(F("T: "));
        display.print(lastDischargeTime, 2);
        display.println(F("s"));
        display.print(F("C1:"));
        display.print(lastDischargeCapacity1, 2);
        display.print(F("mAh C2:"));
        display.print(lastDischargeCapacity2, 2);
        display.println(F("mAh"));
        display.print(F("V1:"));
        display.print(lastDischargeVoltage1, 2);
        display.print(F("V"));
        display.print(F(" V2:"));
        display.print(lastDischargeVoltage2, 2);
        display.println(F("V"));
    } else {
        display.println(F("No Discharge Data Available"));
    }

    display.println();

    display.display();

    // --- Add page cycling logic for case 4 ---
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        bool button1CurrentState = digitalRead(BUTTON1_PIN);
        if (button1CurrentState == HIGH && !button1LastState) {
            button1LastState = button1CurrentState;
            break;
        }
        button1LastState = button1CurrentState;
        delay(10);
    }
}

void displayCombinedPlotPage() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Draw axes
    int x0 = 10, y0 = 54, x1 = 118, y1 = 10;
    display.drawLine(x0, y0, x0, y1, SSD1306_WHITE); // Y-axis
    display.drawLine(x0, y0, x1, y0, SSD1306_WHITE); // X-axis

    // --- Plot charge phase data (V1 and V2) ---
    int nCharge = chargeLogCount;
    float tmaxC = 1, vmaxC = 0, vminC = 0;
    if (nCharge > 1) {
        tmaxC = chargeLogTime[nCharge-1];
        vmaxC = -1000, vminC = 1000;
        for (int i = 0; i < nCharge; i++) {
            if (chargeLogV1[i] > vmaxC) vmaxC = chargeLogV1[i];
            if (chargeLogV2[i] > vmaxC) vmaxC = chargeLogV2[i];
            if (chargeLogV1[i] < vminC) vminC = chargeLogV1[i];
            if (chargeLogV2[i] < vminC) vminC = chargeLogV2[i];
        }
        if (vmaxC == vminC) vmaxC = vminC + 0.1;
    }

    // --- Plot discharge phase data (V1 and V2) ---
    int nDischarge = 0;
    float v1D[PHASE_LOG_POINTS], v2D[PHASE_LOG_POINTS];
    unsigned long tD[PHASE_LOG_POINTS];
    float tmaxD = 1, vmaxD = 0, vminD = 0;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        nDischarge = dischargeLogCount;
        for (int i = 0; i < nDischarge; i++) {
            v1D[i] = dischargeLogV1[i];
            v2D[i] = dischargeLogV2[i];
            tD[i] = dischargeLogTime[i];
        }
        xSemaphoreGive(dataMutex);
    }
    if (nDischarge > 1) {
        tmaxD = tD[nDischarge-1];
        vmaxD = -1000, vminD = 1000;
        for (int i = 0; i < nDischarge; i++) {
            if (v1D[i] > vmaxD) vmaxD = v1D[i];
            if (v2D[i] > vmaxD) vmaxD = v2D[i];
            if (v1D[i] < vminD) vminD = v1D[i];
            if (v2D[i] < vminD) vminD = v2D[i];
        }
        if (vmaxD == vminD) vmaxD = vminD + 0.1;
    }

    // --- Determine global min/max for scaling ---
    float tmax = max(tmaxC, tmaxD);
    if (lastChargeTime > tmax) tmax = lastChargeTime;
    if (lastDischargeTime > tmax) tmax = lastDischargeTime;
    float vmax = max(vmaxC, vmaxD);
    float vmin = min(vminC, vminD);
    if (vmax - vmin < 0.1) { vmax += 0.2; vmin -= 0.2; }
    if (tmax < 1) tmax = 1;

    // --- Serial Plotter Output for all data ---
    Serial.println(F("time_charge,V1_charge,V2_charge,time_discharge,V1_discharge,V2_discharge"));
    int maxPoints = max(nCharge, nDischarge);
    for (int i = 0; i < maxPoints; i++) {
        // Print charge data if available
        if (i < nCharge) {
            Serial.print(chargeLogTime[i]);
            Serial.print(",");
            Serial.print(chargeLogV1[i], 4);
            Serial.print(",");
            Serial.print(chargeLogV2[i], 4);
        } else {
            Serial.print(",,");
        }
        Serial.print(",");
        // Print discharge data if available
        if (i < nDischarge) {
            Serial.print(tD[i]);
            Serial.print(",");
            Serial.print(v1D[i], 4);
            Serial.print(",");
            Serial.print(v2D[i], 4);
        } else {
            Serial.print(",,");
        }
        Serial.println();
    }

    // --- Plot charge V1 (solid line) ---
    if (nCharge > 1) {
        for (int i = 0; i < nCharge-1; i++) {
            int xA = map(chargeLogTime[i], 0, tmax, x0, x1);
            int yA = map(chargeLogV1[i], vmin, vmax, y0, y1);
            int xB = map(chargeLogTime[i+1], 0, tmax, x0, x1);
            int yB = map(chargeLogV1[i+1], vmin, vmax, y0, y1);
            display.drawLine(xA, yA, xB, yB, SSD1306_WHITE);
        }
    }
    // --- Plot charge V2 (dotted line) ---
    if (nCharge > 1) {
        for (int i = 0; i < nCharge-1; i++) {
            int xA = map(chargeLogTime[i], 0, tmax, x0, x1);
            int yA = map(chargeLogV2[i], vmin, vmax, y0, y1);
            int xB = map(chargeLogTime[i+1], 0, tmax, x0, x1);
            int yB = map(chargeLogV2[i+1], vmin, vmax, y0, y1);
            for (float t = 0; t < 1.0; t += 0.05) {
                int x = xA + (xB - xA) * t;
                int y = yA + (yB - yA) * t;
                display.drawPixel(x, y, SSD1306_WHITE);
            }
        }
    }
    // --- Plot discharge V1 (solid line) ---
    if (nDischarge > 1) {
        for (int i = 0; i < nDischarge-1; i++) {
            int xA = map(tD[i], 0, tmax, x0, x1);
            int yA = map(v1D[i], vmin, vmax, y0, y1);
            int xB = map(tD[i+1], 0, tmax, x0, x1);
            int yB = map(v1D[i+1], vmin, vmax, y0, y1);
            display.drawLine(xA, yA, xB, yB, SSD1306_WHITE);
        }
    }
    // --- Plot discharge V2 (dotted line) ---
    if (nDischarge > 1) {
        for (int i = 0; i < nDischarge-1; i++) {
            int xA = map(tD[i], 0, tmax, x0, x1);
            int yA = map(v2D[i], vmin, vmax, y0, y1);
            int xB = map(tD[i+1], 0, tmax, x0, x1);
            int yB = map(v2D[i+1], vmin, vmax, y0, y1);
            for (float tt = 0; tt < 1.0; tt += 0.2) {
                int x = xA + (xB - xA) * tt;
                int y = yA + (yB - yA) * tt;
                display.drawPixel(x, y, SSD1306_WHITE);
            }
        }
    }

    // --- Add labels ---
    display.setCursor(0, 0);
    display.print(F("Charge/Discharge Plot"));
    display.setCursor(10, 56);
    display.print(F("Time (s)"));
    display.setCursor(70, 56);
    display.print(tmax);
    display.print("s");

    display.display();

    // Allow page cycling or auto-exit after 5 seconds
    unsigned long startTime = millis();
    static bool button1LastState = LOW;
    while (millis() - startTime < 5000) {
        bool button1CurrentState = digitalRead(BUTTON1_PIN);
        if (button1CurrentState == HIGH && !button1LastState) {
            button1LastState = button1CurrentState;
            break;
        }
        button1LastState = button1CurrentState;
        delay(10);
    }
}

float readTemperature() {
    int analogValue = analogRead(THERMISTOR_PIN);
    if (analogValue <= 0 || analogValue >= 1023) {
        // Return an error value or a safe default
        return -1000.0;
    }
    float resistance = SERIES_RESISTOR / ((1023.0 / analogValue) - 1);
    if (resistance <= 0) {
        return -1000.0;
    }
    float celsius = 1 / (log(resistance / THERMISTOR_NOMINAL) / BETA + 1.0 / NOMINAL_TEMP_KELVIN) - 273.15;
    return celsius;
}

float readVCC() {
    long adcSum = 0;

    // Take multiple samples for noise reduction
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int adcReading = analogRead(VREF_PIN);  // 10-bit reading (0-1023)
        adcSum += adcReading;
        delay(1);  // Short delay between reads
    }

    float adcAverage = adcSum / (float)NUM_SAMPLES;
    if (adcAverage <= 0) {
        // Return an error value or a safe default
        return 0.0;
    }

    // Calculate VCC: VCC = (VREF * 1024) / ADC_Reading
    float vcc = (VREF * 1024.0) / adcAverage;
    return vcc;
} 