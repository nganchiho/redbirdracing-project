#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// Constants
const uint16_t BRAKE_THRESH = 102;               // 10% of 1023 for brake depression
const uint16_t APPS_FAULT_THRESH = 102;          // 10% of 1023 for APPS fault
const uint32_t STARTIN_HOLD_TIME_MS = 2000;      // 2 sec hold time
const uint32_t BUZZIN_HOLD_TIME_MS = 2000;       // 2 sec hold time
const uint32_t FAULT_HOLD_TIME_MS = 100;         // 100ms fault hold time
const bool FLIP_MOTOR_DIRECTION = false;         // Set to true to flip motor direction
const uint16_t PEDAL_MAX = 1023;                 // MAX analog reading
const int16_t TORQUE_MAX = 32767;                // Maximum torque

// Pin Definitions
#define APPS_5V PIN_PC0      // PC0 analog
#define APPS_3V3 PIN_PC1     // PC1 analog
#define BRAKE PIN_PC3        // PC3 analog
#define START_BUTTON PIN_PC4 // PC4 digital input
#define BRAKE_LIGHT PIN_PD2  // PD2 digital output
#define BUZZER PIN_PD4       // PD4 digital output
#define DRIVE_LED PIN_PD3    // PD3 digital output

// CAN Chip Select Pins
#define CAN_MOTOR_CS PIN_PB2  // PB2 for motor CAN
#define CAN_BMS_CS PIN_PB1    // PB1 for BMS CAN
#define CAN_DEBUG_CS PIN_PB0  // PB0 for debug CAN

// CAN IDs
#define CAN_ID_MOTOR_TORQUE 0x201
#define CAN_ID_DEBUG_STATE 0x300
#define CAN_ID_DEBUG_PEDALS 0x301
#define CAN_ID_DEBUG_FAULT 0x302
#define CAN_ID_BMS_STATUS 0x186040F3  // For reading BMS status

// State definitions
enum VCUState {
    INIT,
    STARTIN,
    BUZZIN,
    DRIVE
};

// String representations of states for debug messages
const char* stateStrings[] = {
    "INIT",
    "STARTIN",
    "BUZZIN",
    "DRIVE"
};

// CAN objects for each bus
MCP2515 canMotor(CAN_MOTOR_CS);
MCP2515 canBMS(CAN_BMS_CS);
MCP2515 canDebug(CAN_DEBUG_CS);

// Global variables
VCUState current_state = INIT;
uint32_t state_entry_time = 0;
uint32_t fault_start_time = 0; // Fault start time
bool fault_detected = false;
bool fault_message_sent = false; // ADDED: Track if fault message was sent
bool bms_ok = false;
uint32_t last_debug_send = 0;
const uint32_t DEBUG_SEND_INTERVAL = 100;  // Send debug messages every 100ms

// Function declarations
bool check_apps_fault(uint16_t apps1, uint16_t apps2);
int16_t calculate_torque(uint16_t pedal_value);
void set_motor_output(int16_t torque); // ADDED BACK: Function declaration
bool initCAN();
void sendMotorTorque(int16_t torque);
void sendDebugMessages(uint16_t apps_3v3_raw, uint16_t apps_5v_raw, bool brake_pressed);
void sendFaultMessage(uint16_t apps_3v3_scaled, uint16_t apps_5v_raw);
void checkBMSMessage();
void update_outputs_for_state(VCUState state);

// APPS faults
bool check_apps_fault(uint16_t apps1, uint16_t apps2) {
    uint16_t difference = abs(apps1 - apps2);
    return (difference > APPS_FAULT_THRESH);
}

// Torque calculation
int16_t calculate_torque(uint16_t pedal_value) {
    int32_t torque = (int32_t)pedal_value * TORQUE_MAX / PEDAL_MAX;
    if (FLIP_MOTOR_DIRECTION) {
        torque = -torque;
    }
    return (int16_t)torque;
}

// Set motor output (wrapper for torque command)
void set_motor_output(int16_t torque) {
    sendMotorTorque(torque); 


// CAN controllers
bool initCAN() {
    SPI.begin();
    if (canMotor.reset() != MCP2515::ERROR_OK) return false;
    if (canMotor.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) return false;
    if (canMotor.setNormalMode() != MCP2515::ERROR_OK) return false;

    if (canBMS.reset() != MCP2515::ERROR_OK) return false;
    if (canBMS.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) return false;
    if (canBMS.setNormalMode() != MCP2515::ERROR_OK) return false;

    if (canDebug.reset() != MCP2515::ERROR_OK) return false;
    if (canDebug.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) return false;
    if (canDebug.setNormalMode() != MCP2515::ERROR_OK) return false;

    Serial.println("CAN controllers initialized");
    return true;
}

// Send torque command to motor
void sendMotorTorque(int16_t torque) {
    struct can_frame torqueMsg;
    torqueMsg.can_id = CAN_ID_MOTOR_TORQUE;
    torqueMsg.can_dlc = 8;
    memset(torqueMsg.data, 0, 8);
    torqueMsg.data[0] = 0x90;  // Command
    torqueMsg.data[1] = torque & 0xFF;           // Low byte
    torqueMsg.data[2] = (torque >> 8) & 0xFF;    // High byte

    if (canMotor.sendMessage(&torqueMsg) == MCP2515::ERROR_OK) {
        Serial.print("Sent torque command: ");
        Serial.println(torque);
    } else {
        Serial.println("Error sending torque command");
    }
}

// Debug messages
void sendDebugMessages(uint16_t apps_3v3_raw, uint16_t apps_5v_raw, bool brake_pressed) {
    uint32_t current_time = millis();
    if (current_time - last_debug_send < DEBUG_SEND_INTERVAL) {
        return;
    }
    last_debug_send = current_time;

    // Vehicle state Debug
    struct can_frame stateMsg;
    stateMsg.can_id = CAN_ID_DEBUG_STATE;
    stateMsg.can_dlc = 4; 
    stateMsg.data[0] = current_state;              // State enum value
    stateMsg.data[1] = brake_pressed ? 1 : 0;     // Brake status
    stateMsg.data[2] = bms_ok ? 1 : 0;            // BMS status
    stateMsg.data[3] = fault_detected ? 1 : 0;    // Fault status

    canDebug.sendMessage(&stateMsg);

    // Pedal readings debug message
    struct can_frame pedalMsg;
    pedalMsg.can_id = CAN_ID_DEBUG_PEDALS;
    pedalMsg.can_dlc = 6;
    pedalMsg.data[0] = (apps_3v3_raw >> 8) & 0xFF;
    pedalMsg.data[1] = apps_3v3_raw & 0xFF;
    pedalMsg.data[2] = (apps_5v_raw >> 8) & 0xFF;
    pedalMsg.data[3] = apps_5v_raw & 0xFF;
    uint16_t brake_reading = analogRead(BRAKE);
    pedalMsg.data[4] = (brake_reading >> 8) & 0xFF;
    pedalMsg.data[5] = brake_reading & 0xFF;

    canDebug.sendMessage(&pedalMsg);
}

// Send fault message when APPS fault occurs
void sendFaultMessage(uint16_t apps_3v3_scaled, uint16_t apps_5v_raw) {
    struct can_frame faultMsg;
    faultMsg.can_id = CAN_ID_DEBUG_FAULT;
    faultMsg.can_dlc = 6;
    faultMsg.data[0] = (apps_3v3_scaled >> 8) & 0xFF;
    faultMsg.data[1] = apps_3v3_scaled & 0xFF;
    faultMsg.data[2] = (apps_5v_raw >> 8) & 0xFF;
    faultMsg.data[3] = apps_5v_raw & 0xFF;
    uint16_t difference = abs(apps_3v3_scaled - apps_5v_raw);
    faultMsg.data[4] = (difference >> 8) & 0xFF;
    faultMsg.data[5] = difference & 0xFF;

    canDebug.sendMessage(&faultMsg);
    Serial.print("APPS Fault! Difference: ");
    Serial.println(difference);
}

// Check for BMS messages
void checkBMSMessage() {
    struct can_frame bmsMsg;
    if (canBMS.readMessage(&bmsMsg) == MCP2515::ERROR_OK) {
        if (bmsMsg.can_id == CAN_ID_BMS_STATUS) {
            bms_ok = (bmsMsg.data[6] == 0x50);
            if (bms_ok) {
                Serial.println("BMS OK signal received");
            }
        }
    }
}

// Update outputs when state changes
void update_outputs_for_state(VCUState state) {
    switch(state) {
        case INIT:
            digitalWrite(BUZZER, LOW);
            digitalWrite(DRIVE_LED, LOW);
            break;
        case STARTIN:
            // No special outputs for STARTIN
            break;
        case BUZZIN:
            digitalWrite(BUZZER, HIGH);
            break;
        case DRIVE:
            digitalWrite(BUZZER, LOW);
            digitalWrite(DRIVE_LED, HIGH);
            break;
    }
}

void setup() {
    // Configure inputs
    pinMode(APPS_5V, INPUT);
    pinMode(APPS_3V3, INPUT);
    pinMode(BRAKE, INPUT);
    pinMode(START_BUTTON, INPUT_PULLUP); 

    // Configure outputs
    pinMode(BRAKE_LIGHT, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(DRIVE_LED, OUTPUT);

    // Configure CAN chip select pins
    pinMode(CAN_MOTOR_CS, OUTPUT);
    pinMode(CAN_BMS_CS, OUTPUT);
    pinMode(CAN_DEBUG_CS, OUTPUT);

    // Initialize outputs to LOW
    digitalWrite(BRAKE_LIGHT, LOW);
    digitalWrite(BUZZER, LOW);
    digitalWrite(DRIVE_LED, LOW);

    // Start serial for debugging
    Serial.begin(9600);
    Serial.println("VCU Starting...");
    
    // Initialize CAN controllers
    if (!initCAN()) {
        Serial.println("CAN initialization failed!");
        while(1); // Halt if CAN fails
    }
    
    // Set initial state outputs
    update_outputs_for_state(INIT);
}

void loop() {
    uint32_t current_time = millis();
    bool brakesPressed = analogRead(BRAKE) > BRAKE_THRESH;
    digitalWrite(BRAKE_LIGHT, brakesPressed ? HIGH : LOW);
    
    checkBMSMessage();

    switch (current_state) {
        case INIT:
            // Transition to STARTIN if button pressed AND brakes depressed
            if (digitalRead(START_BUTTON) == LOW && brakesPressed) {
                current_state = STARTIN;
                state_entry_time = current_time;
                update_outputs_for_state(STARTIN);
                Serial.println("Transitioned to STARTIN");
            }
            break;

        case STARTIN:
            // Return to INIT if brakes released OR button not held
            if (!brakesPressed || digitalRead(START_BUTTON) != LOW) {
                current_state = INIT;
                state_entry_time = current_time;
                update_outputs_for_state(INIT);
                Serial.println("Transitioned back to INIT");
            } 
            // Transition to BUZZIN after holding for required time AND BMS is OK
            else if (current_time - state_entry_time >= STARTIN_HOLD_TIME_MS && bms_ok) {
                current_state = BUZZIN;
                state_entry_time = current_time;
                update_outputs_for_state(BUZZIN);
                Serial.println("Transitioned to BUZZIN");
            }
            break;

        case BUZZIN:
            // Transition to DRIVE after buzzer time
            if (current_time - state_entry_time >= BUZZIN_HOLD_TIME_MS) {
                current_state = DRIVE;
                state_entry_time = current_time;
                update_outputs_for_state(DRIVE);
                Serial.println("Transitioned to DRIVE");
            }
            break;

        case DRIVE:
            uint16_t apps_3v3_raw = analogRead(APPS_3V3);
            uint16_t apps_5v_raw = analogRead(APPS_5V);
            uint16_t apps_3v3_scaled = (uint32_t)apps_3v3_raw * 50 / 33;

            // Send debug messages
            sendDebugMessages(apps_3v3_raw, apps_5v_raw, brakesPressed);
            
            // Check for APPS fault
            if (check_apps_fault(apps_3v3_scaled, apps_5v_raw)) {
                if (!fault_detected) {
                    fault_detected = true;
                    fault_message_sent = false; // Reset flag
                    fault_start_time = current_time; // Record the fault start time
                }
                
                // Send fault message only once when first detected
                if (!fault_message_sent) {
                    sendFaultMessage(apps_3v3_scaled, apps_5v_raw);
                    fault_message_sent = true;
                }
                
                // Check if fault persisted for required time
                if (current_time - fault_start_time >= FAULT_HOLD_TIME_MS) {
                    // Fault persisted for required time - transition to INIT
                    Serial.println("APPS Fault detected for over 100ms. Transitioning to INIT for safety.");
                    set_motor_output(0); // Stop motor output
                    current_state = INIT;
                    state_entry_time = current_time;
                    update_outputs_for_state(INIT);
                    fault_detected = false; // Reset fault detection
                    fault_message_sent = false; // Reset flag
                }
            } else {
                // No fault - calculate and set torque
                if (fault_detected) {
                    fault_detected = false; // Reset fault detection
                    fault_message_sent = false; // Reset flag
                    Serial.println("APPS fault cleared");
                }
                int16_t torque = calculate_torque(apps_5v_raw);
                sendMotorTorque(torque);
            }
            break;
    }
    
    // prevent cpu overload
    delay(10);
}