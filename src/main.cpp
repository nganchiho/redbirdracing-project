#include <Arduino.h>
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

// CAN objects for each bus
MCP2515 canMotor(CAN_MOTOR_CS);
MCP2515 canBMS(CAN_BMS_CS);
MCP2515 canDebug(CAN_DEBUG_CS);

// Global variables
VCUState current_state = INIT;
uint32_t state_entry_time = 0;
uint32_t fault_start_time = 0;
bool fault_detected = false;
bool fault_message_sent = false;
bool bms_ok = false;
uint32_t last_debug_send = 0;
const uint32_t DEBUG_SEND_INTERVAL = 100;

// APPS faults
bool check_apps_fault(uint16_t apps1, uint16_t apps2) {
    int16_t difference = abs((int16_t)apps1 - (int16_t)apps2);
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

// CAN controllers initialization
bool initCAN() {
    SPI.begin();

    // Helper lambda to init a single controller
    auto initOne = [](MCP2515& can) -> bool {
        if (can.reset() != MCP2515::ERROR_OK) return false;
        if (can.setBitrate(CAN_500KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) return false;
        if (can.setNormalMode() != MCP2515::ERROR_OK) return false;
        return true;
    };

    if (!initOne(canMotor)) return false;
    if (!initOne(canBMS)) return false;
    if (!initOne(canDebug)) return false;

    Serial.println("CAN controllers initialized");
    return true;
}

// Send torque command to motor
void sendMotorTorque(int16_t torque) {
    struct can_frame torqueMsg;
    torqueMsg.can_id = CAN_ID_MOTOR_TORQUE;
    torqueMsg.can_dlc = 3;
    torqueMsg.data[0] = 0x90;              // Command byte
    torqueMsg.data[1] = torque & 0xFF;     // Low byte
    torqueMsg.data[2] = (torque >> 8) & 0xFF;  // High byte

    if (canMotor.sendMessage(&torqueMsg) == MCP2515::ERROR_OK) {
    } else {
        Serial.println("Error sending torque command");
    }
}

// Set motor output (wrapper)
void set_motor_output(int16_t torque) {
    sendMotorTorque(torque);
}

// Debug messages
void sendDebugMessages(uint16_t apps_3v3_raw, uint16_t apps_5v_raw, uint16_t brake_raw, bool brake_pressed) {
    uint32_t current_time = millis();
    if (current_time - last_debug_send < DEBUG_SEND_INTERVAL) {
        return;
    }
    last_debug_send = current_time;

    // Send State
    uint8_t statusByte = (current_state & 0x03) |  // State
                         ((brake_pressed ? 1 : 0) << 2) |
                         ((bms_ok ? 1 : 0) << 3) |
                         ((fault_detected ? 1 : 0) << 4);

    struct can_frame stateMsg;
    stateMsg.can_id = CAN_ID_DEBUG_STATE;
    stateMsg.can_dlc = 2;
    stateMsg.data[0] = statusByte;
    stateMsg.data[1] = 0;
    canDebug.sendMessage(&stateMsg);

    // 2. Send Pedals
    struct can_frame pedalMsg;
    pedalMsg.can_id = CAN_ID_DEBUG_PEDALS;
    pedalMsg.can_dlc = 6;
    
    // APPS 3V3
    pedalMsg.data[0] = apps_3v3_raw & 0xFF;
    pedalMsg.data[1] = (apps_3v3_raw >> 8) & 0xFF;
    
    // APPS 5V
    pedalMsg.data[2] = apps_5v_raw & 0xFF;
    pedalMsg.data[3] = (apps_5v_raw >> 8) & 0xFF;
    
    // Brake
    pedalMsg.data[4] = brake_raw & 0xFF;
    pedalMsg.data[5] = (brake_raw >> 8) & 0xFF;

    canDebug.sendMessage(&pedalMsg);
}

// Fault message
void sendFaultMessage(uint16_t apps_3v3_scaled, uint16_t apps_5v_raw) {
    struct can_frame faultMsg;
    faultMsg.can_id = CAN_ID_DEBUG_FAULT;
    faultMsg.can_dlc = 6;
    
    // Switched to Little Endian (LSB First) 
    faultMsg.data[0] = apps_3v3_scaled & 0xFF;
    faultMsg.data[1] = (apps_3v3_scaled >> 8) & 0xFF;
    
    faultMsg.data[2] = apps_5v_raw & 0xFF;
    faultMsg.data[3] = (apps_5v_raw >> 8) & 0xFF;
    
    uint16_t difference = abs((int16_t)apps_3v3_scaled - (int16_t)apps_5v_raw);
    faultMsg.data[4] = difference & 0xFF;
    faultMsg.data[5] = (difference >> 8) & 0xFF;

    canDebug.sendMessage(&faultMsg);
    Serial.print("APPS Fault! Difference: ");
    Serial.println(difference);
}

// Check for BMS messages
void checkBMSMessage() {
    struct can_frame bmsMsg;
    if (canBMS.readMessage(&bmsMsg) != MCP2515::ERROR_OK) return;
    
    if (bmsMsg.can_id != CAN_ID_BMS_STATUS) return;

    // Check specific byte for BMS(0x50)
    if (bmsMsg.data[6] == 0x50) {
        if (!bms_ok) {
            bms_ok = true;
            Serial.println("BMS OK signal received");
        }
    } else {
        bms_ok = false;
    }
}

// Update outputs for state
void update_outputs_for_state(VCUState state) {
    switch(state) {
        case INIT:
            digitalWrite(BUZZER, LOW);
            digitalWrite(DRIVE_LED, LOW);
            break;
        case STARTIN:
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


// Main

void setup() {
    // Input
    pinMode(APPS_5V, INPUT);
    pinMode(APPS_3V3, INPUT);
    pinMode(BRAKE, INPUT);
    pinMode(START_BUTTON, INPUT_PULLUP);

    // Output
    pinMode(BRAKE_LIGHT, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(DRIVE_LED, OUTPUT);

    pinMode(CAN_MOTOR_CS, OUTPUT);
    pinMode(CAN_BMS_CS, OUTPUT);
    pinMode(CAN_DEBUG_CS, OUTPUT);

    digitalWrite(BRAKE_LIGHT, LOW);
    digitalWrite(BUZZER, LOW);
    digitalWrite(DRIVE_LED, LOW);

    Serial.begin(115200);
    Serial.println("VCU Starting...");

    if (!initCAN()) {
        Serial.println("CAN initialization failed!");
        while(1);
    }

    update_outputs_for_state(INIT);
}

void loop() {
    uint32_t current_time = millis();

    uint16_t brake_raw = analogRead(BRAKE);
    bool brakesPressed = brake_raw > BRAKE_THRESH;
    
    digitalWrite(BRAKE_LIGHT, brakesPressed ? HIGH : LOW);

    checkBMSMessage();

    switch (current_state) {
        case INIT:
            if (digitalRead(START_BUTTON) == LOW && brakesPressed) {
                current_state = STARTIN;
                state_entry_time = current_time;
                update_outputs_for_state(STARTIN);
                Serial.println("Transitioned to STARTIN");
            }
            break;

        case STARTIN:
            if (!brakesPressed || digitalRead(START_BUTTON) != LOW) {
                current_state = INIT;
                state_entry_time = current_time;
                update_outputs_for_state(INIT);
                Serial.println("Transitioned back to INIT");
            } else if (current_time - state_entry_time >= STARTIN_HOLD_TIME_MS && bms_ok) {
                current_state = BUZZIN;
                state_entry_time = current_time;
                update_outputs_for_state(BUZZIN);
                Serial.println("Transitioned to BUZZIN");
            }
            break;

        case BUZZIN:
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
            sendDebugMessages(apps_3v3_raw, apps_5v_raw, brake_raw, brakesPressed);

            // Fault Checking
            if (check_apps_fault(apps_3v3_scaled, apps_5v_raw)) {
                if (!fault_detected) {
                    fault_detected = true;
                    fault_message_sent = false;
                    fault_start_time = current_time;
                }

                if (!fault_message_sent) {
                    sendFaultMessage(apps_3v3_scaled, apps_5v_raw);
                    fault_message_sent = true;
                }

                // Check critical timeout
                if (current_time - fault_start_time >= FAULT_HOLD_TIME_MS) {
                    Serial.println("APPS Fault > 100ms. Safety Shutdown.");
                    set_motor_output(0);
                    current_state = INIT;
                    state_entry_time = current_time;
                    update_outputs_for_state(INIT);
                    fault_detected = false;
                    fault_message_sent = false;
                }
            } else {
                if (fault_detected) {
                    fault_detected = false;
                    fault_message_sent = false;
                    Serial.println("APPS fault cleared");
                }
            }
            if (current_state == DRIVE) {
                set_motor_output(calculate_torque(apps_5v_raw));
            }
            break;
    }

    delay(10);
}