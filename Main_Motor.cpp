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
#define MODE_OFF    0x00u
#define MODE_RUN    0x01u
#define MODE_EMG    0x02u
#define MODE_REHAB  0x03u
#define MODE_HOMING 0x04u

// emg_cmd values (mirror of Main_UI.cpp)
#define EMG_HOLD    0u
#define EMG_OPEN    1u
#define EMG_CLOSE   2u

// ===== MOTOR CONFIG =====
static const int NUM_MOTORS = 5;
static const uint MOTOR_PWM_PIN[5] = {2, 6, 10, 14, 20};   // IN A
static const uint MOTOR_DIR_PIN[5] = {3, 7, 11, 15, 21};   // IN B
static const uint ENC_A_PIN[5] = {4, 8, 12, 16, 18}; // Channel A
static const uint ENC_B_PIN[5] = {5, 9, 13, 17, 19}; // Channel B

// ===== SWITCH PINS =====
// (Switches moved to UI Pico; Motor Pico obeys UART commands only)

// ===== PWM SETTINGS =====
static const unsigned int PWM_TOP = 1000;
static const unsigned int PWM_FREQ = 20000;   // 20 kHz

// ===== ENCODER COUNTS =====
static volatile int encoder_count[5] = {0};

// ===== UART RECEIVE STATE =====
static unsigned char rx_buf[7];
static int           rx_idx      = 0;
static unsigned char g_uart_mode = MODE_RUN;
static unsigned char g_speed_pct = 40u;  // speed from UI Pico (0-100 %)
static unsigned char g_emg_cmd = 0;
static unsigned char g_fsr_flags = 0;

// ===== REHAB MODE STATE =====
static const uint32_t REHAB_DWELL_MS = 1200u;
static bool rehab_closing = true;
static uint32_t rehab_last_toggle_ms = 0u;

// ===== PWM INIT =====
void pwm_init_pin(uint gpio) {
    // Configure each motor PWM pin for a shared frequency and independent duty.
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, PWM_TOP);
    float sys_clk = 125000000.0f;
    float clkdiv = sys_clk / (PWM_FREQ * (PWM_TOP + 1));
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(gpio, 0);
}

// Convert 0-100% speed to PWM level against PWM_TOP.
static unsigned int speed_pct_to_pwm(unsigned char pct) {
    // Convert 0-100% speed command into PWM compare level.
    if (pct > 100u) pct = 100u;
    return (unsigned int)(((unsigned int)pct * PWM_TOP) / 100u);
}

// ===== MOTOR CONTROL =====
void motors_forward() {
    // All motors move in the same direction in current control model.
    unsigned int level = speed_pct_to_pwm(g_speed_pct);
    printf("Forward: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_put(MOTOR_DIR_PIN[i], 1);
        pwm_set_gpio_level(MOTOR_PWM_PIN[i], level);
        printf("M%d ", i);
    }
    printf("speed=%u%%\n", (unsigned int)g_speed_pct);
}

void motors_reverse() {
    // Reverse direction while keeping the same commanded speed.
    unsigned int level = speed_pct_to_pwm(g_speed_pct);
    printf("Reverse: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_put(MOTOR_DIR_PIN[i], 0);
        pwm_set_gpio_level(MOTOR_PWM_PIN[i], level);
        printf("M%d ", i);
    }
    printf("speed=%u%%\n", (unsigned int)g_speed_pct);
}

void motors_stop() {
    // Stop is PWM=0; direction pins are left as-is.
    printf("Stop\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        pwm_set_gpio_level(MOTOR_PWM_PIN[i], 0);
    }
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
            g_emg_cmd   = rx_buf[4];
            g_fsr_flags = rx_buf[5];

            printf("[UART] mode=%u speed=%u%% emg_cmd=%u fsr=0x%02X\n",
                   (unsigned int)g_uart_mode,
                   (unsigned int)g_speed_pct,
                   (unsigned int)g_emg_cmd,
                   (unsigned int)g_fsr_flags);
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

    // Init motor pins
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_init(MOTOR_DIR_PIN[i]);
        gpio_set_dir(MOTOR_DIR_PIN[i], GPIO_OUT);
        gpio_put(MOTOR_DIR_PIN[i], 0);
        pwm_init_pin(MOTOR_PWM_PIN[i]);
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

    while (true) {
        // Always consume newest UI command before making motor decision.
        process_uart();

        // Send encoder counts back to UI Pico every 50 ms.
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - enc_tx_last_ms) >= 50u) {
            enc_tx_last_ms = now_ms;
            send_encoder_packet();
        }

        // Motor action driven entirely by UART packet fields.
        if (g_uart_mode == MODE_OFF || g_fsr_flags != 0) {
            // Safety stop from UI side has highest priority.
            motors_stop();
        } else if (g_uart_mode == MODE_HOMING) {
            // Homing direction is explicitly commanded by UI Pico.
            if (g_emg_cmd == EMG_OPEN) {
                motors_forward();
            } else if (g_emg_cmd == EMG_CLOSE) {
                motors_reverse();
            } else {
                motors_stop();
            }
        } else if (g_emg_cmd == EMG_OPEN) {
            motors_forward();
        } else if (g_emg_cmd == EMG_CLOSE) {
            motors_reverse();
        } else if (g_uart_mode == MODE_REHAB) {
            // UI Pico sends MODE_REHAB; Motor Pico handles the timed cycle.
            if ((now_ms - rehab_last_toggle_ms) >= REHAB_DWELL_MS) {
                rehab_closing = !rehab_closing;
                rehab_last_toggle_ms = now_ms;
                printf("[REHAB] %s\n", rehab_closing ? "CLOSE" : "OPEN");
            }
            if (rehab_closing) motors_reverse();
            else               motors_forward();
        } else {
            motors_stop();
        }

        sleep_ms(5);
    }

    return 0;
}