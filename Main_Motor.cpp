/**
 * ============================================================
 * REHABILITATION GLOVE — MOTOR PICO  (U1 / Pico B)
 * ============================================================
 *
 * ──────────────── PIN MAP (from schematic) ───────────────────
 *
 *  GPIO  0   UART0 TX  ──► Pico A RX
 *  GPIO  1   UART0 RX  ◄── Pico A TX
 *
 *  Motor Driver Pins  (INA = direction bit A, INB = direction bit B)
 *  The motor spins based on INA/INB logic:
 *    INA=1 INB=0 → forward   INA=0 INB=1 → reverse
 *    INA=0 INB=0 → coast     INA=1 INB=1 → brake
 *  PWM is applied on INA or INB to control speed.
 *  Here we use INA as PWM and INB as static direction flag.
 *
 *  Motor 1 (Thumb)   INA=GPIO2   INB=GPIO3
 *  Motor 2 (Index)   INA=GPIO6   INB=GPIO7
 *  Motor 3 (Middle)  INA=GPIO10  INB=GPIO11
 *  Motor 4 (Ring)    INA=GPIO14  INB=GPIO15
 *  Motor 5 (Pinky)   INA=GPIO20  INB=GPIO21
 *
 *  Quadrature Encoder Channels (interrupt-driven)
 *  Motor 1   Ch A=GPIO4   Ch B=GPIO5
 *  Motor 2   Ch A=GPIO8   Ch B=GPIO9
 *  Motor 3   Ch A=GPIO12  Ch B=GPIO13
 *  Motor 4   Ch A=GPIO16  Ch B=GPIO17
 *  Motor 5   Ch A=GPIO18  Ch B=GPIO19
 *
 * ──────────────── UART PACKET FORMAT (7 bytes) ───────────────
 *  [0]  0xAA       START
 *  [1]  mode       0x00=OFF  0x01=Manual  0x02=Pulse  0x03=EMG
 *  [2]  speed      5–100
 *  [3]  position   0=hold  1=open  2=close  (Manual mode cmd)
 *  [4]  emg_cmd    0=hold  1=open  2=close
 *  [5]  fsr_flags  bit0=FSR0 … bit4=FSR4
 *  [6]  checksum   [1]^[2]^[3]^[4]^[5]
 * ============================================================
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <math.h>

// ─── UART ─────────────────────────────────────────────────────
#define UART_PORT   uart0
#define BAUD_RATE   115200
#define PIN_TX      0
#define PIN_RX      1
#define START_BYTE  0xAA

// ─── MOTOR COUNT ──────────────────────────────────────────────
#define NUM_MOTORS  5

// ─── MOTOR DRIVER PINS ────────────────────────────────────────
// INA = PWM pin (speed + direction A)
// INB = direction pin (direction B)
// To drive forward:  PWM on INA, INB = LOW
// To drive reverse:  INA = LOW,  PWM on INB
// We PWM only INA and use INB as a static direction flag for simplicity.
static const uint PIN_INA[NUM_MOTORS] = {2,  6,  10, 14, 20};
static const uint PIN_INB[NUM_MOTORS] = {3,  7,  11, 15, 21};

// ─── ENCODER PINS ─────────────────────────────────────────────
static const uint PIN_ENC_A[NUM_MOTORS] = {4,  8,  12, 16, 18};
static const uint PIN_ENC_B[NUM_MOTORS] = {5,  9,  13, 17, 19};

// ─── PWM CONFIG ───────────────────────────────────────────────
// ~25 kHz at 125 MHz sys clock
#define PWM_WRAP  4999

// ─── ENCODER POSITION ─────────────────────────────────────────
// Tune ENC_COUNTS_CLOSED to your specific motor + gearbox travel.
// A good starting point: run the motor to full close manually,
// read the count, and set it here.
#define ENC_COUNTS_OPEN    0
#define ENC_COUNTS_CLOSED  4000

static volatile int32_t enc_count[NUM_MOTORS] = {0};

// ─── PID ──────────────────────────────────────────────────────
#define KP       0.8f
#define KI       0.05f
#define KD       0.1f
#define DT       0.02f    // 50 Hz loop → 20 ms
#define DEADBAND 30       // counts; stop PID if within this of target

static float pid_integral[NUM_MOTORS] = {0};
static float pid_prev_err[NUM_MOTORS] = {0};

// ─── PULSE MODE ───────────────────────────────────────────────
#define PULSE_DWELL_MAX_MS  3000u
#define PULSE_DWELL_MIN_MS   200u
static bool     pulse_closing = true;
static uint32_t pulse_last_ms = 0;

// ─── PROTOCOL ─────────────────────────────────────────────────
typedef enum : unsigned char {
    MODE_OFF    = 0x00,
    MODE_MANUAL = 0x01,
    MODE_PULSE  = 0x02,
    MODE_EMG    = 0x03
} GloveMode;

typedef struct __attribute__((packed)) {
    unsigned char start;
    unsigned char mode;
    unsigned char speed;
    unsigned char position;   // 0=hold 1=open 2=close in Manual mode
    unsigned char emg_cmd;
    unsigned char fsr_flags;
    unsigned char checksum;
} Packet;

// ─── RECEIVED COMMAND STATE ───────────────────────────────────
static volatile GloveMode g_mode      = MODE_OFF;
static volatile unsigned char g_speed     = 50;
static volatile unsigned char g_position  = 0;
static volatile unsigned char g_emg_cmd   = 0;
static volatile unsigned char g_fsr_flags = 0;

// ─── ENCODER ISR ──────────────────────────────────────────────
// Interrupt handler for encoder channel-A edges.
// Finds which motor triggered, samples A/B phase state, and updates that
// motor's position count with direction inferred from quadrature relationship.
static void enc_isr(uint gpio, uint32_t events) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (gpio == PIN_ENC_A[i]) {
            bool a = gpio_get(PIN_ENC_A[i]);
            bool b = gpio_get(PIN_ENC_B[i]);
            // Rising A: B=LOW → forward (+); B=HIGH → reverse (-)
            // Falling A: B=HIGH → forward (+); B=LOW → reverse (-)
            enc_count[i] += (a ^ b) ? -1 : 1;
            return;
        }
    }
}

// ─── MOTOR CONTROL ────────────────────────────────────────────
// duty: 0.0–1.0, forward: true=open direction, false=close direction
// Drives one motor in the requested direction with PWM duty control.
// Clamps duty to [0,1], then applies PWM to INA for forward or INB for
// reverse, keeping the opposite input at 0 to avoid shoot-through behavior.
static void motor_drive(int idx, float duty, bool forward) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    unsigned short level = (unsigned short)(duty * PWM_WRAP);

    if (forward) {
        // INA = PWM, INB = LOW
        uint slice = pwm_gpio_to_slice_num(PIN_INA[idx]);
        uint chan  = pwm_gpio_to_channel(PIN_INA[idx]);
        pwm_set_chan_level(slice, chan, level);
        gpio_put(PIN_INB[idx], 0);
    } else {
        // INA = LOW, INB = PWM
        // INB is a regular GPIO in this setup; for reverse we swap:
        // Set INA to 0 and drive INB high at full power isn't PWM-able
        // unless we configure INB as PWM too. Since motor drivers like
        // DRV8833 / L298N support PWM on either input, we configure
        // BOTH INA and INB as PWM outputs during init, and use them
        // exclusively here.
        uint slice_a = pwm_gpio_to_slice_num(PIN_INA[idx]);
        uint chan_a  = pwm_gpio_to_channel(PIN_INA[idx]);
        pwm_set_chan_level(slice_a, chan_a, 0);

        uint slice_b = pwm_gpio_to_slice_num(PIN_INB[idx]);
        uint chan_b  = pwm_gpio_to_channel(PIN_INB[idx]);
        pwm_set_chan_level(slice_b, chan_b, level);
    }
}

// Stops one motor and resets PID state for that finger.
// Sets both driver inputs to zero (coast) and clears integral/previous-error
// terms so the next motion command starts from a clean controller state.
static void motor_stop(int idx) {
    // Brake: both inputs LOW → coast, or both HIGH → brake.
    // We use coast (both 0) to reduce heat.
    uint slice_a = pwm_gpio_to_slice_num(PIN_INA[idx]);
    pwm_set_chan_level(slice_a, pwm_gpio_to_channel(PIN_INA[idx]), 0);
    uint slice_b = pwm_gpio_to_slice_num(PIN_INB[idx]);
    pwm_set_chan_level(slice_b, pwm_gpio_to_channel(PIN_INB[idx]), 0);
    pid_integral[idx] = 0;
    pid_prev_err[idx] = 0;
}

// ─── PID POSITION DRIVE ───────────────────────────────────────
// Runs one PID step for a finger and applies resulting motor command.
// Computes position error to target, handles deadband stop, updates integral
// and derivative terms, scales output by speed percentage, converts to duty,
// then drives in open/close direction based on output sign.
static void drive_to_position(int idx, int32_t target, unsigned char speed_pct) {
    float error = (float)(target - enc_count[idx]);

    if (fabsf(error) < DEADBAND) {
        motor_stop(idx);
        return;
    }

    pid_integral[idx] += error * DT;
    // Clamp integral to prevent windup
    if (pid_integral[idx] >  1000.0f) pid_integral[idx] =  1000.0f;
    if (pid_integral[idx] < -1000.0f) pid_integral[idx] = -1000.0f;

    float deriv       = (error - pid_prev_err[idx]) / DT;
    pid_prev_err[idx] = error;

    float output = KP * error + KI * pid_integral[idx] + KD * deriv;
    output *= (speed_pct / 100.0f);

    bool  forward = (output >= 0);
    float duty    = fabsf(output) / (float)ENC_COUNTS_CLOSED;
    if (duty > 1.0f) duty = 1.0f;
    // Minimum duty to overcome worm-gear stiction (~8%)
    if (duty > 0.0f && duty < 0.08f) duty = 0.08f;

    motor_drive(idx, duty, forward);
}

// ─── UART PACKET RECEIVE (non-blocking) ───────────────────────
static unsigned char rx_buf[sizeof(Packet)];
static int     rx_idx = 0;

// Non-blocking UART packet parser for 7-byte controller frames.
// Hunts for start byte, accumulates bytes, validates checksum, and on success
// updates the global command state consumed by the control loop.
static void process_uart() {
    while (uart_is_readable(UART_PORT)) {
        unsigned char b = uart_getc(UART_PORT);

        if (rx_idx == 0 && b != START_BYTE) continue;  // hunt for start

        rx_buf[rx_idx++] = b;

        if (rx_idx >= (int)sizeof(Packet)) {
            rx_idx = 0;
            Packet *p = (Packet*)rx_buf;

            unsigned char chk = p->mode ^ p->speed ^ p->position
                        ^ p->emg_cmd ^ p->fsr_flags;

            if (p->start != START_BYTE || chk != p->checksum) {
                printf("[MOTOR] Bad packet – dropped\n");
                return;
            }

            g_mode      = (GloveMode)p->mode;
            g_speed     = p->speed;
            g_position  = p->position;
            g_emg_cmd   = p->emg_cmd;
            g_fsr_flags = p->fsr_flags;
        }
    }
}

// ─── HW INIT ──────────────────────────────────────────────────
// Initializes all motor-side peripherals.
// Configures UART, sets all motor driver pins to PWM outputs, initializes PWM
// slices/channels, and configures encoder GPIOs with pull-ups plus IRQ callback.
static void hw_init() {
    stdio_init_all();

    // UART
    uart_init(UART_PORT, BAUD_RATE);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);

    // Motors: configure both INA and INB as PWM outputs
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_set_function(PIN_INA[i], GPIO_FUNC_PWM);
        gpio_set_function(PIN_INB[i], GPIO_FUNC_PWM);

        uint slice_a = pwm_gpio_to_slice_num(PIN_INA[i]);
        pwm_set_wrap(slice_a, PWM_WRAP);
        pwm_set_enabled(slice_a, true);
        pwm_set_chan_level(slice_a, pwm_gpio_to_channel(PIN_INA[i]), 0);

        uint slice_b = pwm_gpio_to_slice_num(PIN_INB[i]);
        pwm_set_wrap(slice_b, PWM_WRAP);
        pwm_set_enabled(slice_b, true);
        pwm_set_chan_level(slice_b, pwm_gpio_to_channel(PIN_INB[i]), 0);
    }

    // Encoders: pull-ups, interrupt on both edges of Channel A
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_init(PIN_ENC_A[i]);
        gpio_set_dir(PIN_ENC_A[i], GPIO_IN);
        gpio_pull_up(PIN_ENC_A[i]);

        gpio_init(PIN_ENC_B[i]);
        gpio_set_dir(PIN_ENC_B[i], GPIO_IN);
        gpio_pull_up(PIN_ENC_B[i]);

        gpio_set_irq_enabled_with_callback(
            PIN_ENC_A[i],
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
            true,
            &enc_isr
        );
    }

    printf("Motor Pico (Pico B) ready.\n");
}

// ─── MAIN ─────────────────────────────────────────────────────
// Main 50 Hz motor control loop.
// Receives command packets, enforces OFF behavior, evaluates FSR gating, then
// dispatches Manual/Pulse/EMG logic to compute per-finger targets and drive
// motors through PID position control.
int main() {
    hw_init();

    uint32_t last_ms = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_ms) < 20) continue;   // 50 Hz control loop
        last_ms = now;

        process_uart();

        // ── If glove is OFF, stop everything ─────────────
        if (g_mode == MODE_OFF) {
            for (int i = 0; i < NUM_MOTORS; i++) motor_stop(i);
            continue;
        }

        // ── Per-finger FSR contact flags ──────────────────
        // bit N of fsr_flags → finger N is touching something
        auto fsr = [](int i) -> bool {
            return (g_fsr_flags >> i) & 0x01;
        };

        // ── Mode dispatch ─────────────────────────────────
        switch (g_mode) {

        // ══ MANUAL MODE ══════════════════════════════════
        // Glove Open / Close buttons drive the fingers.
        // FSR contact stops closing motion on that finger.
        case MODE_MANUAL:
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (g_position == 2) {          // CLOSE
                    if (fsr(i)) {
                        motor_stop(i);          // contact — stop this finger
                    } else {
                        drive_to_position(i, ENC_COUNTS_CLOSED, g_speed);
                    }
                } else if (g_position == 1) {   // OPEN
                    drive_to_position(i, ENC_COUNTS_OPEN, g_speed);
                } else {                         // HOLD
                    motor_stop(i);
                }
            }
            break;

        // ══ PULSE / REHAB MODE ════════════════════════════
        // Automatically cycles open ↔ close at speed-dependent pace.
        case MODE_PULSE: {
            uint32_t dwell = PULSE_DWELL_MAX_MS
                           - (uint32_t)((g_speed / 100.0f)
                             * (PULSE_DWELL_MAX_MS - PULSE_DWELL_MIN_MS));

            if ((now - pulse_last_ms) >= dwell) {
                pulse_closing = !pulse_closing;
                pulse_last_ms = now;
                printf("[MOTOR] Pulse → %s\n", pulse_closing ? "CLOSE" : "OPEN");
            }

            int32_t target = pulse_closing ? ENC_COUNTS_CLOSED : ENC_COUNTS_OPEN;

            for (int i = 0; i < NUM_MOTORS; i++) {
                if (fsr(i) && pulse_closing) {
                    motor_stop(i);   // stop this finger on contact
                } else {
                    drive_to_position(i, target, g_speed);
                }
            }
            break;
        }

        // ══ EMG / DETECTION MODE ══════════════════════════
        // Muscle signal from DFRobot EMG drives open or close.
        case MODE_EMG:
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (fsr(i) && g_emg_cmd == 2) {
                    motor_stop(i);   // contact while closing → stop finger
                    continue;
                }
                switch (g_emg_cmd) {
                case 1:   // open
                    drive_to_position(i, ENC_COUNTS_OPEN, g_speed);
                    break;
                case 2:   // close
                    drive_to_position(i, ENC_COUNTS_CLOSED, g_speed);
                    break;
                default:  // hold
                    motor_stop(i);
                    break;
                }
            }
            break;

        default:
            for (int i = 0; i < NUM_MOTORS; i++) motor_stop(i);
            break;
        }

        printf("[MOTOR] mode=%u spd=%u pos=%u emg=%u fsr=0x%02X "
               "enc=[%d,%d,%d,%d,%d]\n",
               g_mode, g_speed, g_position, g_emg_cmd, g_fsr_flags,
               enc_count[0], enc_count[1], enc_count[2],
               enc_count[3], enc_count[4]);
    }
}