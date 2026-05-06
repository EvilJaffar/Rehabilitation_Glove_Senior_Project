#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/sync.h"

typedef unsigned int uint;

// This firmware is an actuator endpoint:
// - Receives control packets from UI Pico
// - Drives motors from parsed mode/cmd/speed
// - Streams encoder counts back to UI Pico

// ===== UART CONFIG =====
#define UART_PORT   uart0
#define BAUD_RATE   115200
#define PIN_TX      0
#define PIN_RX      1

// ===== PACKET PROTOCOL =====
#define START_BYTE      0xAAu   // UI → Motor command packet
#define ENC_START_BYTE  0xBBu   // Motor → UI encoder packet
#define MODE_MANUAL     0x01u
#define MODE_ASSIST     0x02u
#define MODE_REHAB      0x03u
#define MODE_HOMING     0x04u

// motor_cmd values (mirror of Main_UI.cpp)
#define MOTOR_CMD_STOP    0u
#define MOTOR_CMD_OPEN    1u
#define MOTOR_CMD_CLOSE   2u

// ===== MOTOR CONFIG =====
static const int NUM_MOTORS = 5;
// EN/IN wiring model:
// - EN pin gets PWM duty (speed)
// - IN pin selects direction (CW/CCW)
static const uint MOTOR_EN_PIN[5] = {2, 6, 10, 14, 20};
static const uint MOTOR_IN_PIN[5] = {3, 7, 11, 15, 21};

// EN/IN direction polarity. Flip these if motor wiring reverses perceived CW/CCW.
#define DIR_CW_LEVEL  1u
#define DIR_CCW_LEVEL 0u

static const uint ENC_A_PIN[5] = {4, 8, 12, 16, 18}; // Channel A
static const uint ENC_B_PIN[5] = {5, 9, 13, 17, 19}; // Channel B

// ===== PWM SETTINGS =====
static const unsigned int PWM_TOP = 1000;
static const unsigned int PWM_FREQ = 20000;   // 20 kHz

// ===== ENCODER COUNTS =====
static volatile int encoder_count[5] = {0};

// ===== UART RECEIVE STATE =====
static unsigned char rx_buf[7];
static int           rx_idx      = 0;
static unsigned char g_uart_mode = MODE_MANUAL;  // overwritten by first valid packet
static unsigned char g_speed_pct = 0u;            // overwritten by first valid packet
static unsigned char g_motor_cmd = MOTOR_CMD_STOP; // overwritten by first valid packet
static unsigned char g_fsr_flags = 0;
static unsigned char g_position = 0u;
static uint32_t g_last_valid_pkt_ms = 0u;  // 0 = stale at boot; motors stop until first packet

// If no valid UI packet arrives within this window, force motor stop.
static const uint32_t UART_CMD_TIMEOUT_MS = 150u;

// ===== REHAB MODE STATE =====
static const uint32_t REHAB_RUN_MS = 1200u;       // Time to run in one direction before slowing down.
static const uint32_t REHAB_PAUSE_DEFAULT_MS = 734u; // Default stop time between direction changes.
static const uint32_t REHAB_PAUSE_MIN_MS = 500u;
static const uint32_t REHAB_PAUSE_MAX_MS = 3000u;
static const uint32_t REHAB_PAUSE_UNIT_MS = 100u;     // UI packs pause duration in 100ms units.
static const uint32_t REHAB_RAMP_TICK_MS = 10u;   // Ramp update period.
static const unsigned char REHAB_ACCEL_STEP_PCT = 2u; // +speed step per ramp tick.
static const unsigned char REHAB_DECEL_STEP_PCT = 2u; // -speed step per ramp tick.

typedef enum {
    REHAB_PHASE_RAMP_UP = 0,
    REHAB_PHASE_RUN,
    REHAB_PHASE_RAMP_DOWN,
    REHAB_PHASE_PAUSE
} rehab_phase_t;

static rehab_phase_t rehab_phase = REHAB_PHASE_RAMP_UP;
static bool rehab_closing = true;
static bool rehab_active_prev = false;
static uint32_t rehab_phase_start_ms = 0u;
static uint32_t rehab_speed_step_last_ms = 0u;
static unsigned char rehab_ramped_speed_pct = 0u;
static uint32_t rehab_pause_ms = REHAB_PAUSE_DEFAULT_MS;

// ===== PWM INIT =====
static bool slice_initialized[8] = {false};  // Track which slices have been initialized

void pwm_init_pin(uint gpio) {
    // Configure each motor PWM pin for a shared frequency and independent duty.
    // Only initialize each slice once since multiple GPIOs can share a slice.
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    
    if (!slice_initialized[slice]) {
        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, PWM_TOP);
        float sys_clk = 125000000.0f;
        float clkdiv = sys_clk / (PWM_FREQ * (PWM_TOP + 1));
        if (clkdiv < 1.0f) clkdiv = 1.0f;
        pwm_config_set_clkdiv(&config, clkdiv);
        pwm_init(slice, &config, true);
        slice_initialized[slice] = true;
        printf("[PWM] Initialized slice %u\n", slice);
    }
    
    pwm_set_gpio_level(gpio, 0);
}

// Convert 0-100% speed to PWM level against PWM_TOP.
static unsigned int speed_pct_to_pwm(unsigned char pct) {
    // Convert 0-100% speed command into PWM compare level.
    if (pct > 100u) pct = 100u;
    return (unsigned int)(((unsigned int)pct * PWM_TOP) / 100u);
}

static void motors_drive_all(unsigned char dir_level, unsigned char speed_pct, const char* tag) {
    unsigned int level = speed_pct_to_pwm(speed_pct);
    printf("%s: level=%u ", tag, level);
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_put(MOTOR_IN_PIN[i], dir_level);

        // Re-assert PWM function on EN pin (may have been switched to GPIO by motors_stop).
        gpio_set_function(MOTOR_EN_PIN[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(MOTOR_EN_PIN[i]);
        pwm_set_enabled(slice, true);

        // Restore proper PWM wrap for problematic motors.
        if (MOTOR_EN_PIN[i] == 6 || MOTOR_EN_PIN[i] == 20) {
            pwm_set_wrap(slice, PWM_TOP);
        }

        pwm_set_gpio_level(MOTOR_EN_PIN[i], level);
        printf("M%d(GPIO%d) ", i, MOTOR_EN_PIN[i]);
    }
    printf("speed=%u%%\n", (unsigned int)speed_pct);
}

// ===== MOTOR CONTROL =====
void motors_forward() {
    // EN/IN CW drive: IN sets CW direction, EN gets PWM speed.
    motors_drive_all(DIR_CW_LEVEL, g_speed_pct, "Forward");
}

void motors_reverse() {
    // EN/IN CCW drive: IN sets CCW direction, EN gets PWM speed.
    motors_drive_all(DIR_CCW_LEVEL, g_speed_pct, "Reverse");
}

static void motors_forward_speed(unsigned char speed_pct) {
    motors_drive_all(DIR_CW_LEVEL, speed_pct, "RehabFwd");
}

static void motors_reverse_speed(unsigned char speed_pct) {
    motors_drive_all(DIR_CCW_LEVEL, speed_pct, "RehabRev");
}

void motors_stop() {
    // EN/IN stop: zero PWM level then switch EN to GPIO-LOW for a definitive off state.
    // Zero the PWM register first (pin still in PWM mode) so there is no spike if PWM
    // is re-enabled by motors_forward/reverse in the next iteration.
    printf("Stop: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Zero the PWM compare level while the pin is still in PWM mode.
        uint slice   = pwm_gpio_to_slice_num(MOTOR_EN_PIN[i]);
        uint channel = pwm_gpio_to_channel(MOTOR_EN_PIN[i]);
        pwm_set_chan_level(slice, channel, 0);
        pwm_set_enabled(slice, false);

        // Now switch EN to GPIO output and drive it LOW.
        gpio_set_function(MOTOR_EN_PIN[i], GPIO_FUNC_SIO);
        gpio_set_dir(MOTOR_EN_PIN[i], GPIO_OUT);
        gpio_put(MOTOR_EN_PIN[i], 0);

        // Keep IN low while stopped.
        gpio_put(MOTOR_IN_PIN[i], DIR_CCW_LEVEL);

        printf("M%d ", i);
    }
    printf("STOPPED\n");
}

// ===== PER-MOTOR SINGLE DRIVE (used by Assistive mode for independent finger control) =====
static void motor_forward_single(int i) {
    unsigned int level = speed_pct_to_pwm(g_speed_pct);
    gpio_put(MOTOR_IN_PIN[i], DIR_CW_LEVEL);
    gpio_set_function(MOTOR_EN_PIN[i], GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_EN_PIN[i]);
    pwm_set_enabled(slice, true);
    if (MOTOR_EN_PIN[i] == 6 || MOTOR_EN_PIN[i] == 20) pwm_set_wrap(slice, PWM_TOP);
    pwm_set_gpio_level(MOTOR_EN_PIN[i], level);
}

static void motor_reverse_single(int i) {
    unsigned int level = speed_pct_to_pwm(g_speed_pct);
    gpio_put(MOTOR_IN_PIN[i], DIR_CCW_LEVEL);
    gpio_set_function(MOTOR_EN_PIN[i], GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_EN_PIN[i]);
    pwm_set_enabled(slice, true);
    if (MOTOR_EN_PIN[i] == 6 || MOTOR_EN_PIN[i] == 20) pwm_set_wrap(slice, PWM_TOP);
    pwm_set_gpio_level(MOTOR_EN_PIN[i], level);
}

static void motor_stop_single(int i) {
    uint slice   = pwm_gpio_to_slice_num(MOTOR_EN_PIN[i]);
    uint channel = pwm_gpio_to_channel(MOTOR_EN_PIN[i]);
    pwm_set_chan_level(slice, channel, 0);
    pwm_set_enabled(slice, false);
    gpio_set_function(MOTOR_EN_PIN[i], GPIO_FUNC_SIO);
    gpio_set_dir(MOTOR_EN_PIN[i], GPIO_OUT);
    gpio_put(MOTOR_EN_PIN[i], 0);
    gpio_put(MOTOR_IN_PIN[i], DIR_CCW_LEVEL);
}

// ===== ENCODER ISR =====
void gpio_irq_callback(uint gpio, unsigned long events) {
   (void)events;
    // Direction comes from quadrature phase relationship (A xor B).
   for (int i = 0; i < NUM_MOTORS; i++) {
      if (gpio == ENC_A_PIN[i]) {
         bool a = gpio_get(ENC_A_PIN[i]);
         bool b = gpio_get(ENC_B_PIN[i]);
         int dir = (a ^ b) ? 1 : -1;
         encoder_count[i] += dir;
         return;
      }
   }
}

// ===== ENCODER PACKET TRANSMIT =====
// Packet layout (12 bytes):
//   [0]     = ENC_START_BYTE (0xBB)
//   [1..10] = encoder_count[0..4] as signed 16-bit little-endian
//   [11]    = XOR checksum of bytes [1..10]
void send_encoder_packet(void) {
    unsigned char pkt[12];
    pkt[0] = ENC_START_BYTE;
    unsigned char chk = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Snapshot volatile count with interrupts disabled briefly.
        int cnt;
        uint32_t save = save_and_disable_interrupts();
        cnt = encoder_count[i];
        restore_interrupts(save);

        int16_t c16 = (int16_t)(cnt < -32768 ? -32768 : (cnt > 32767 ? 32767 : cnt));
        pkt[1 + i * 2]     = (unsigned char)(c16 & 0xFF);
        pkt[1 + i * 2 + 1] = (unsigned char)((c16 >> 8) & 0xFF);
        chk ^= pkt[1 + i * 2];
        chk ^= pkt[1 + i * 2 + 1];
    }
    pkt[11] = chk;
    uart_write_blocking(UART_PORT, pkt, 12);
}

// ===== UART PACKET RECEIVE (non-blocking) =====
void process_uart(void) {
    // Parser resynchronizes on START_BYTE and validates xor checksum.
    while (uart_is_readable(UART_PORT)) {
        unsigned char b = uart_getc(UART_PORT);

        if (rx_idx == 0 && b != (unsigned char)START_BYTE) continue;

        rx_buf[rx_idx++] = b;

        if (rx_idx >= 7) {
            rx_idx = 0;

            unsigned char chk = rx_buf[1] ^ rx_buf[2] ^ rx_buf[3]
                              ^ rx_buf[4] ^ rx_buf[5];

            if (rx_buf[0] != (unsigned char)START_BYTE || chk != rx_buf[6]) {
                printf("[UART] Bad packet\n");
                return;
            }

            g_uart_mode = rx_buf[1];
            g_speed_pct = rx_buf[2];
            if (g_speed_pct > 100u) g_speed_pct = 100u;
            g_position = rx_buf[3];
            g_motor_cmd = rx_buf[4];
            g_fsr_flags = rx_buf[5];

            if (g_uart_mode == MODE_REHAB) {
                uint32_t pause_ms = (uint32_t)g_position * REHAB_PAUSE_UNIT_MS;
                if (pause_ms == 0u) {
                    rehab_pause_ms = REHAB_PAUSE_DEFAULT_MS;
                } else {
                    if (pause_ms < REHAB_PAUSE_MIN_MS) pause_ms = REHAB_PAUSE_MIN_MS;
                    if (pause_ms > REHAB_PAUSE_MAX_MS) pause_ms = REHAB_PAUSE_MAX_MS;
                    rehab_pause_ms = pause_ms;
                }
            }

            g_last_valid_pkt_ms = to_ms_since_boot(get_absolute_time());

                        printf("[UART] mode=%u speed=%u%% pos=%u motor_cmd=%u fsr=0x%02X rehab_pause=%lu\n",
                   (unsigned int)g_uart_mode,
                   (unsigned int)g_speed_pct,
                   (unsigned int)g_position,
                   (unsigned int)g_motor_cmd,
                   (unsigned int)g_fsr_flags,
                   (unsigned long)rehab_pause_ms);
        }
    }
}

// ===== MAIN =====
int main() {
    stdio_init_all();
    sleep_ms(200);

    // Init UART
    uart_init(UART_PORT, BAUD_RATE);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);

    // Init motor pins - PWM first, then direction pins
    for (int i = 0; i < NUM_MOTORS; i++) {
        pwm_init_pin(MOTOR_EN_PIN[i]);  // EN pin is PWM in EN/IN mode
        
        // Then set up IN direction pins as GPIO (after PWM is initialized)
        gpio_init(MOTOR_IN_PIN[i]);
        gpio_set_function(MOTOR_IN_PIN[i], GPIO_FUNC_SIO);
        gpio_set_dir(MOTOR_IN_PIN[i], GPIO_OUT);
        // Default to open/coast-compatible idle level in EN/IN mode.
        gpio_put(MOTOR_IN_PIN[i], DIR_CCW_LEVEL);
    }

    // Init encoder pins
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_init(ENC_A_PIN[i]);
        gpio_set_dir(ENC_A_PIN[i], GPIO_IN);
        gpio_pull_up(ENC_A_PIN[i]);
        gpio_init(ENC_B_PIN[i]);
        gpio_set_dir(ENC_B_PIN[i], GPIO_IN);
        gpio_pull_up(ENC_B_PIN[i]);
    }

    gpio_set_irq_enabled_with_callback(
        ENC_A_PIN[0],
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &gpio_irq_callback
    );
    for (int i = 1; i < NUM_MOTORS; i++) {
        gpio_set_irq_enabled(
            ENC_A_PIN[i],
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
            true
        );
    }

    printf("Motor Ready\n");

    uint32_t enc_tx_last_ms = 0u;
    // g_last_valid_pkt_ms intentionally left at 0 so cmd_stale fires
    // immediately on boot. Motors stay stopped until the first valid
    // UI packet is received — no preset values take effect.
    uint32_t boot_now_ms = to_ms_since_boot(get_absolute_time());
    rehab_phase_start_ms = boot_now_ms;
    rehab_speed_step_last_ms = boot_now_ms;

    while (true) {
        // Always consume newest UI command before making motor decision.
        process_uart();

        // Send encoder counts back to UI Pico every 50 ms.
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - enc_tx_last_ms) >= 50u) {
            enc_tx_last_ms = now_ms;
            send_encoder_packet();
        }

        // Motor action driven by explicit mode + fresh UART command stream.
        bool cmd_stale = ((now_ms - g_last_valid_pkt_ms) > UART_CMD_TIMEOUT_MS);
        bool rehab_run_cmd = (!cmd_stale && g_fsr_flags == 0u && g_uart_mode == MODE_REHAB && g_motor_cmd == MOTOR_CMD_OPEN);
        if (!rehab_run_cmd) {
            rehab_active_prev = false;
            rehab_phase = REHAB_PHASE_RAMP_UP;
            rehab_ramped_speed_pct = 0u;
        }

        if (cmd_stale) {
            motors_stop();
        } else if (g_fsr_flags != 0) {
            // FSR contact — safety stop has highest priority.
            motors_stop();
        } else if (g_uart_mode == MODE_HOMING) {
            // Homing direction is explicitly commanded by UI Pico.
            if (g_motor_cmd == MOTOR_CMD_OPEN) {
                motors_forward();
            } else if (g_motor_cmd == MOTOR_CMD_CLOSE) {
                motors_reverse();
            } else {
                motors_stop();
            }
        } else if (g_uart_mode == MODE_MANUAL) {
            // Manual mode is driven by raw switch states in packet byte[3]:
            // bit0=open, bit1=close. This makes release->stop deterministic.
            bool open_pressed = ((g_position & 0x01u) != 0u);
            bool close_pressed = ((g_position & 0x02u) != 0u);

            if (open_pressed && !close_pressed) {
                motors_forward();
            } else if (close_pressed && !open_pressed) {
                motors_reverse();
            } else {
                motors_stop();
            }
        } else if (g_uart_mode == MODE_ASSIST) {
            // Assistive mode: per-finger packed commands from UI Pico.
            // byte[3] (g_position) packs M0-M3 as 2-bit fields: M0=bits[1:0], M1=bits[3:2], M2=bits[5:4], M3=bits[7:6]
            // byte[4] (g_motor_cmd) packs M4 as bits[1:0]
            // Command values: 0=STOP, 1=OPEN(forward), 2=CLOSE(reverse)
            unsigned char finger_cmd[5];
            finger_cmd[0] = (g_position >> 0) & 0x03u;
            finger_cmd[1] = (g_position >> 2) & 0x03u;
            finger_cmd[2] = (g_position >> 4) & 0x03u;
            finger_cmd[3] = (g_position >> 6) & 0x03u;
            finger_cmd[4] = (g_motor_cmd     ) & 0x03u;
            for (int i = 0; i < NUM_MOTORS; i++) {
                if      (finger_cmd[i] == MOTOR_CMD_OPEN)  motor_forward_single(i);
                else if (finger_cmd[i] == MOTOR_CMD_CLOSE) motor_reverse_single(i);
                else                                        motor_stop_single(i);
            }
        } else if (g_uart_mode == MODE_REHAB) {
            // UI Pico sends MODE_REHAB with an explicit run token in motor_cmd.
            // Only run the smooth cycle when cmd == MOTOR_CMD_OPEN.
            if (g_motor_cmd == MOTOR_CMD_OPEN) {
                if (!rehab_active_prev) {
                    rehab_active_prev = true;
                    rehab_phase = REHAB_PHASE_RAMP_UP;
                    rehab_phase_start_ms = now_ms;
                    rehab_speed_step_last_ms = now_ms;
                    rehab_ramped_speed_pct = 0u;
                    printf("[REHAB] START -> %s\n", rehab_closing ? "CLOSE" : "OPEN");
                }

                unsigned char target_speed_pct = g_speed_pct;
                bool step_due = ((now_ms - rehab_speed_step_last_ms) >= REHAB_RAMP_TICK_MS);

                if (rehab_phase == REHAB_PHASE_RAMP_UP) {
                    if (step_due) {
                        rehab_speed_step_last_ms = now_ms;
                        unsigned int next = (unsigned int)rehab_ramped_speed_pct + (unsigned int)REHAB_ACCEL_STEP_PCT;
                        if (next > target_speed_pct) next = target_speed_pct;
                        rehab_ramped_speed_pct = (unsigned char)next;
                    }
                    if (rehab_closing) motors_reverse_speed(rehab_ramped_speed_pct);
                    else               motors_forward_speed(rehab_ramped_speed_pct);

                    if (rehab_ramped_speed_pct >= target_speed_pct) {
                        rehab_phase = REHAB_PHASE_RUN;
                        rehab_phase_start_ms = now_ms;
                        printf("[REHAB] RUN %s\n", rehab_closing ? "CLOSE" : "OPEN");
                    }
                } else if (rehab_phase == REHAB_PHASE_RUN) {
                    rehab_ramped_speed_pct = target_speed_pct;
                    if (rehab_closing) motors_reverse_speed(rehab_ramped_speed_pct);
                    else               motors_forward_speed(rehab_ramped_speed_pct);

                    if ((now_ms - rehab_phase_start_ms) >= REHAB_RUN_MS) {
                        rehab_phase = REHAB_PHASE_RAMP_DOWN;
                        rehab_speed_step_last_ms = now_ms;
                        printf("[REHAB] RAMP DOWN\n");
                    }
                } else if (rehab_phase == REHAB_PHASE_RAMP_DOWN) {
                    if (step_due) {
                        rehab_speed_step_last_ms = now_ms;
                        int next = (int)rehab_ramped_speed_pct - (int)REHAB_DECEL_STEP_PCT;
                        if (next < 0) next = 0;
                        rehab_ramped_speed_pct = (unsigned char)next;
                    }

                    if (rehab_ramped_speed_pct > 0u) {
                        if (rehab_closing) motors_reverse_speed(rehab_ramped_speed_pct);
                        else               motors_forward_speed(rehab_ramped_speed_pct);
                    } else {
                        motors_stop();
                        rehab_phase = REHAB_PHASE_PAUSE;
                        rehab_phase_start_ms = now_ms;
                        printf("[REHAB] PAUSE %lums\n", (unsigned long)rehab_pause_ms);
                    }
                } else { // REHAB_PHASE_PAUSE
                    motors_stop();
                    if ((now_ms - rehab_phase_start_ms) >= rehab_pause_ms) {
                        rehab_closing = !rehab_closing;
                        rehab_phase = REHAB_PHASE_RAMP_UP;
                        rehab_phase_start_ms = now_ms;
                        rehab_speed_step_last_ms = now_ms;
                        printf("[REHAB] REVERSE -> %s\n", rehab_closing ? "CLOSE" : "OPEN");
                    }
                }
            } else {
                rehab_active_prev = false;
                rehab_phase = REHAB_PHASE_RAMP_UP;
                rehab_ramped_speed_pct = 0u;
                motors_stop();
            }
        } else {
            motors_stop();
        }

        sleep_ms(5);
    }

    return 0;
}