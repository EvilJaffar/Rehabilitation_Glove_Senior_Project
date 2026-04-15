#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

typedef unsigned int uint;

// ===== UART CONFIG =====
#define UART_PORT   uart0
#define BAUD_RATE   115200
#define PIN_TX      0
#define PIN_RX      1

// ===== PACKET PROTOCOL =====
#define START_BYTE  0xAAu
#define MODE_OFF    0x00u
#define MODE_RUN    0x01u
#define MODE_EMG    0x02u

// ===== MOTOR CONFIG =====
static const int NUM_MOTORS = 5;
static const uint MOTOR_PWM_PIN[5] = {2, 6, 10, 14, 20};   // IN A
static const uint MOTOR_DIR_PIN[5] = {3, 7, 11, 15, 21};   // IN B
static const uint ENC_A_PIN[5] = {4, 8, 12, 16, 18}; // Channel A
static const uint ENC_B_PIN[5] = {5, 9, 13, 17, 19}; // Channel B

// ===== SWITCH PINS =====
static const uint SWITCH_FORWARD_PIN = 22;
static const uint SWITCH_REVERSE_PIN = 23;

// ===== PWM SETTINGS =====
static const unsigned int PWM_TOP = 1000;
static const unsigned int PWM_FREQ = 20000;   // 20 kHz
static const unsigned int MOTOR_SPEED = 400;  // 40% duty

// ===== ENCODER COUNTS =====
static volatile int encoder_count[5] = {0};

// ===== UART RECEIVE STATE =====
static unsigned char rx_buf[7];
static int           rx_idx      = 0;
static unsigned char g_uart_mode = MODE_RUN;
static unsigned char g_emg_cmd = 0;
static unsigned char g_fsr_flags = 0;

// ===== PWM INIT =====
void pwm_init_pin(uint gpio) {
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

// ===== MOTOR CONTROL =====
void motors_forward() {
    printf("Forward: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_put(MOTOR_DIR_PIN[i], 1);
        pwm_set_gpio_level(MOTOR_PWM_PIN[i], MOTOR_SPEED);
        printf("M%d ", i);
    }
    printf("\n");
}

void motors_reverse() {
    printf("Reverse: ");
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_put(MOTOR_DIR_PIN[i], 0);
        pwm_set_gpio_level(MOTOR_PWM_PIN[i], MOTOR_SPEED);
        printf("M%d ", i);
    }
    printf("\n");
}

void motors_stop() {
    printf("Stop\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        pwm_set_gpio_level(MOTOR_PWM_PIN[i], 0);
    }
}

// ===== ENCODER ISR =====
void gpio_irq_callback(uint gpio, unsigned long events) {
   (void)events;
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

// ===== UART PACKET RECEIVE (non-blocking) =====
void process_uart(void) {
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
            g_emg_cmd   = rx_buf[4];
            g_fsr_flags = rx_buf[5];

            printf("[UART] mode=%u emg_cmd=%u fsr=0x%02X\n",
                   (unsigned int)g_uart_mode,
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

    // Init switch
    gpio_init(SWITCH_FORWARD_PIN);
    gpio_set_dir(SWITCH_FORWARD_PIN, GPIO_IN);
    gpio_pull_up(SWITCH_FORWARD_PIN);
    gpio_init(SWITCH_REVERSE_PIN);
    gpio_set_dir(SWITCH_REVERSE_PIN, GPIO_IN);
    gpio_pull_up(SWITCH_REVERSE_PIN);

    printf("Motor + Encoder Switch Test Ready\n");

    while (true) {
        // Check for UART packets from Controller Pico
        process_uart();

        bool forward = (gpio_get(SWITCH_FORWARD_PIN) == 0);
        bool reverse = (gpio_get(SWITCH_REVERSE_PIN) == 0);

        // FSR contact from Controller Pico overrides local switch commands.
        if (g_uart_mode == MODE_OFF || g_fsr_flags != 0) {
            motors_stop();
        } 
        else if (g_uart_mode == MODE_EMG) {
            if (g_emg_cmd == 1) {
                motors_forward();
            } else if (g_emg_cmd == 2) {
                motors_reverse();
            } else {
                motors_stop();
            }
        } else if (forward && !reverse) {
            motors_forward();
        } else if (reverse && !forward) {
            motors_reverse();
        } else {
            motors_stop();
        }
        
        sleep_ms(5);
    }

    return 0;
}