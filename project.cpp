#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

//Constants
const float BRAKE_THRESHOLD = 0.1f;          // 10% depression for brakes to be considered "depressed"
const float APPS_FAULT_THRESHOLD = 0.1f;     // 10% difference between APPS readings for fault
const uint32_t STARTIN_HOLD_TIME_MS = 2000;  // 2 seconds in milliseconds
const uint32_t BUZZIN_HOLD_TIME_MS = 2000;   // 2 seconds in milliseconds
const uint32_t FAULT_HOLD_TIME_MS = 100;     // 100ms fault hold time
const bool FLIP_MOTOR_DIRECTION = false;     // Set to true to flip motor direction
const float APPS_5V_SCALE = 3.3f / 5.0f;     // Scale 5V APPS to 3.3V range

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


// Utility Functions
uint32_t get_current_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

bool brakes_depressed(void) {
    return read_brake_percent() >= BRAKE_THRESHOLD;
}

bool check_apps_fault(void) {
    float apps_3v3 = read_apps_3v3();
    float apps_5v = read_apps_5v() * APPS_5V_SCALE;
    return fabs(apps_3v3 - apps_5v) > APPS_FAULT_THRESHOLD;
}

int16_t calculate_torque(float pedal_percent) {
    if (FLIP_MOTOR_DIRECTION) {
        return (int16_t)(-32768 + (pedal_percent * 32768));
    } else {
        return (int16_t)(pedal_percent * 32767);
    }
}

// State Machine Functions
void log_state_transition(VCUState old_state, VCUState new_state) {
    const char *state_names[] = {"INIT", "STARTIN", "BUZZIN", "DRIVE"};
    printf("Transition: %s -> %s\n", state_names[old_state], state_names[new_state]);
}

void enter_state(VCUState new_state) {
    log_state_transition(current_state, new_state);
    current_state = new_state;
    state_entry_time = get_current_time_ms();
    fault_detected = false;

    switch (new_state) {
        case STATE_INIT:
            set_motor_output(0);
            set_buzzer(false);
            set_drive_led(false);
            break;

        case STATE_STARTIN:
            set_motor_output(0);
            set_buzzer(false);
            set_drive_led(false);
            break;

        case STATE_BUZZIN:
            set_motor_output(0);
            set_buzzer(true);
            set_drive_led(false);
            break;

        case STATE_DRIVE:
            set_motor_output(0);
            set_buzzer(false);
            set_drive_led(true);
            break;
    }
}

void handle_brake_lights(void) {
    set_brake_light(brakes_depressed());
}

void run_state_machine(void) {
    uint32_t current_time = get_current_time_ms();
    uint32_t time_in_state = current_time - state_entry_time;
    handle_brake_lights();  // all states

    switch (current_state) {
        case STATE_INIT:
            if (read_start_button() && brakes_depressed()) {
                enter_state(STATE_STARTIN);
            }
            break;

        case STATE_STARTIN:
            if (!brakes_depressed() || !read_start_button()) {
                enter_state(STATE_INIT);
            } else if (time_in_state >= STARTIN_HOLD_TIME_MS) {
                enter_state(STATE_BUZZIN);
            }
            break;

        case STATE_BUZZIN:
            if (time_in_state >= BUZZIN_HOLD_TIME_MS) {
                enter_state(STATE_DRIVE);
            }
            break;

        case STATE_DRIVE:
            float apps_3v3 = read_apps_3v3();
            float apps_5v = read_apps_5v() * APPS_5V_SCALE;
            float pedal_percent = (apps_3v3 + apps_5v) / 2.0f;

            if (check_apps_fault()) {
                if (!fault_detected) {
                    fault_detected = true;
                    fault_start_time = current_time;
                } else if (current_time - fault_start_time >= FAULT_HOLD_TIME_MS) {
                    printf("APPS Fault! Transitioning to INIT for safety.\n");
                    enter_state(STATE_INIT);
                }
            } else {
                fault_detected = false;
                int16_t torque = calculate_torque(pedal_percent);
                set_motor_output(torque);
            }
            break;
    }
}

// Main
int main(void) {
    printf("VCU Starting...\n");
    enter_state(STATE_INIT);

    while (1) {
        run_state_machine();
        struct timespec delay = {0, 10000000};  // delay 10 ms
        nanosleep(&delay, NULL);
    }

    return 0;
}