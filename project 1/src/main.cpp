#include <Arduino.h>


//Constants
const float BRAKE_THRESHOLD = 0.1f;          // 10% depression for brakes to be considered "depressed"
const uint16_t BRAKE_THRESH = 102.3;
const float APPS_FAULT_THRESHOLD = 0.1f;     // 10% difference between APPS readings for fault
const uint32_t STARTIN_HOLD_TIME_MS = 2000;  // 2 seconds in milliseconds
const uint32_t BUZZIN_HOLD_TIME_MS = 2000;   // 2 seconds in milliseconds
const uint32_t FAULT_HOLD_TIME_MS = 100;     // 100ms fault hold time
const bool FLIP_MOTOR_DIRECTION = false;     // Set to true to flip motor direction
const float APPS_5V_SCALE = 3.3f / 5.0f;     // Scale 5V APPS to 3.3V range
const uint16_t PEDAL_MIN = 0;
const uint16_t PEDAL_MAX = 1023;
const int16_t TORQUE_MIN = -32768;
const int16_t TORQUE_MAX = 32767;


//Definitions
typedef enum {
    STATE_INIT,
    STATE_STARTIN,
    STATE_BUZZIN,
    STATE_DRIVE
} VCUState;

// Global
static VCUState current_state = STATE_INIT;
static uint32_t state_entry_time = 0;
static uint32_t fault_start_time = 0;
static bool fault_detected = false;


// Pin Definitions
#define APPS_5V PIN_PC0      // PC0 analog
#define APPS_3V3 PIN_PC1     // PC1 analog
#define BRAKE PIN_PC3        // PC3 analog
#define START_BUTTON PIN_PC4 // PC4 digital input

#define BRAKE_LIGHT PIN_PD2  // PD2 digital output
#define BUZZER PIN_PD4       // PD4 digital output
#define DRIVE_LED PIN_PD3    // PD3 digital output

// put function definitions here:
bool check_apps_fault(float apps1, float apps2);
int16_t calculate_torque(float pedal_percent);
void set_motor_output(int16_t torque);
bool check_apps_fault(float apps1, float apps2) {
    float difference = abs(apps1 - apps2);
    return (difference > APPS_FAULT_THRESHOLD);
}

int16_t calculate_torque(float pedal_percent) {
    // Scale pedal percentage to torque range
    float normalized_pedal = constrain(pedal_percent, 0.0f, 1.0f);
    int16_t torque = static_cast<int16_t>(normalized_pedal * (TORQUE_MAX - TORQUE_MIN) + TORQUE_MIN);
    return torque;
}

void set_motor_output(int16_t torque) {
    //debugging
    Serial.print("Setting torque: ");
    Serial.println(torque);
}



void setup() {
  pinMode(APPS_5V, INPUT);
  pinMode(APPS_3V3, INPUT);
  pinMode(BRAKE, INPUT);
  pinMode(START_BUTTON, INPUT);

  pinMode(BRAKE_LIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(DRIVE_LED, OUTPUT);


}

void loop() {
    uint32_t current_time = millis();
    uint32_t time_in_state = current_time - state_entry_time;
    bool brakesPressed = analogRead(BRAKE) > BRAKE_THRESH;
    digitalWrite(BRAKE_LIGHT, brakesPressed ? HIGH : LOW);

    switch (current_state) {
        case STATE_INIT:
            if (digitalRead(START_BUTTON) && brakesPressed) {
                current_state = STATE_STARTIN;
                state_entry_time = current_time;
            }
            break;

        case STATE_STARTIN:
            if (!brakesPressed || !digitalRead(START_BUTTON)) {
                current_state = STATE_INIT;
                state_entry_time = current_time;
            } else if (time_in_state >= STARTIN_HOLD_TIME_MS) {
                digitalWrite(BUZZER, HIGH);
                current_state = STATE_BUZZIN;
                state_entry_time = current_time;
            }
            break;

        case STATE_BUZZIN:
            if (time_in_state >= BUZZIN_HOLD_TIME_MS) {
                current_state = STATE_DRIVE;
                state_entry_time = current_time;
            }
            break;

        case STATE_DRIVE:
            float apps_3v3 = analogRead(APPS_3V3) * APPS_5V_SCALE;
            float apps_5v = analogRead(APPS_5V) * APPS_5V_SCALE;
            float pedal_percent = (apps_3v3 + apps_5v) / 2.0f;

            if (check_apps_fault(apps_3v3, apps_5v)) {
                if (!fault_detected) {
                    fault_detected = true;
                    fault_start_time = current_time;
                } else if (current_time - fault_start_time >= FAULT_HOLD_TIME_MS) {
                    Serial.println("APPS Fault! Transitioning to INIT for safety.");
                    current_state =STATE_INIT;
                    state_entry_time = current_time;
                }
            } else {
                fault_detected = false;
                int16_t torque = calculate_torque(pedal_percent);
                set_motor_output(torque);
                digitalWrite(DRIVE_LED, HIGH);
            }
            break;
    }
}