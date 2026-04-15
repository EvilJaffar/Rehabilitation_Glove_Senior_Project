/**
 * ============================================================
 * REHABILITATION GLOVE — CONTROLLER PICO  (U7 / Pico A)
 * ============================================================
 *
 * ──────────────── PIN MAP (from schematic) ───────────────────
 *
 *  GPIO  0   UART0 TX  ──► Pico B RX
 *  GPIO  1   UART0 RX  ◄── Pico B TX
 *  GPIO  2   INPUT     ── Rotary Encoder A (CLK)
 *  GPIO  3   INPUT     ── Rotary Encoder B (DT)
 *  GPIO  4   INPUT     ── Glove ON/OFF toggle
 *  GPIO  6   INPUT     ── Detection Mode button
 *  GPIO  7   INPUT     ── Manual Mode button
 *  GPIO  8   INPUT     ── Rehab (Pulse) Mode button
 *  GPIO  9   INPUT     ── Glove Close (manual command)
 *  GPIO 10   INPUT     ── Glove Open  (manual command)
 *  GPIO 11   I2C1 SCL  ── ADS1115 SCL  (FSR ADC expander)
 *  GPIO 12   I2C0 SDA  ── ADS1115 SDA  (FSR ADC expander)
 *  GPIO 26   ADC0      ── EMG Analog (DFRobot Gravity SEN0240)
 *  GPIO 27   ADC1      ── (spare / future FSR direct)
 *  GPIO 28   ADC2      ── (spare / future FSR direct)
 *
 * NOTE ON I2C PINS:
 *   GPIO11 = I2C1_SCL  and  GPIO12 = I2C0_SDA are on different
 *   I2C peripherals. On the Pico, valid I2C0 pairs are:
 *     SDA: GPIO0/4/8/12/16/20   SCL: GPIO1/5/9/13/17/21
 *   GPIO12 (SDA) pairs with GPIO13 (SCL) for I2C0, but your
 *   schematic uses GPIO11 (SCL). This means both pins must be
 *   driven as a single I2C bus using i2c1 on GP10/11 or i2c0
 *   on GP12/13. The schematic label "ADC SCL / ADC SDA" suggests
 *   a dedicated I2C bus for the ADS1115; we use i2c0 on GP12/13
 *   for SDA/SCL respectively. If your ADS1115 ACTUALLY connects
 *   to GP11 & GP12, use a software I2C library instead.
 *   *** Verify with your physical wiring before flashing. ***
 *
 * ──────────────── MODES ──────────────────────────────────────
 *  Detection Mode (GPIO6) → EMG Mode:  muscles drive the glove
 *  Manual Mode    (GPIO7) → Manual:    Open/Close buttons control
 *  Rehab Mode     (GPIO8) → Pulse:     auto open/close cycle
 *  Glove ON/OFF   (GPIO4) → enables / disables motor output
 *
 * ──────────────── ADS1115 (FSR ADC expander) ─────────────────
 *  ADS1115-A  ADDR→GND  (0x48): AIN0=FSR0, AIN1=FSR1,
 *                                AIN2=FSR2, AIN3=FSR3
 *  ADS1115-B  ADDR→VDD  (0x49): AIN0=FSR4
 *  Each FSR: 3.3V ── FSR ── AINx ── 10kΩ ── GND
 *
 * ──────────────── UART PACKET FORMAT (7 bytes) ───────────────
 *  [0]  0xAA       START
 *  [1]  mode       0x01=Manual  0x02=Pulse  0x03=EMG  0x00=OFF
 *  [2]  speed      5–100  (rotary encoder)
 *  [3]  position   0=open, 1=hold, 2=close  (manual cmd)
 *  [4]  emg_cmd    0=hold  1=open  2=close
 *  [5]  fsr_flags  bit0=FSR0 … bit4=FSR4  (1=contact)
 *  [6]  checksum   [1]^[2]^[3]^[4]^[5]
 * ============================================================
 */

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
typedef unsigned int uint;
using namespace std;

// ─── UART ─────────────────────────────────────────────────────
#define UART_PORT     uart0
#define BAUD_RATE     115200
#define PIN_TX        0   // → Pico B RX
#define PIN_RX        1   // ← Pico B TX

// ─── ROTARY ENCODER (speed) ───────────────────────────────────
#define PIN_ENC_A     2   // CLK / A phase
#define PIN_ENC_B     3   // DT  / B phase

#define SPEED_MIN     5
#define SPEED_MAX     100
#define SPEED_DEFAULT 50
#define SPEED_STEP    5   // % per detent; increase for faster response

static volatile int g_speed = SPEED_DEFAULT;

// ─── CONTROL BUTTONS ──────────────────────────────────────────
#define PIN_GLOVE_ONOFF       4   // toggle motor enable
#define PIN_BTN_DETECTION     6   // → EMG mode
#define PIN_BTN_MANUAL        7   // → Manual mode
#define PIN_BTN_REHAB         8   // → Pulse/Rehab mode
#define PIN_BTN_GLOVE_CLOSE   9   // manual close command
#define PIN_BTN_GLOVE_OPEN   10   // manual open  command

// ─── I2C (ADS1115 FSR expander) ───────────────────────────────
// Using i2c0: SDA=GPIO12, SCL=GPIO13
// Verify this matches your physical PCB traces.
#define I2C_PORT      i2c0
#define PIN_I2C_SDA   12   // ADC SDA from schematic
#define PIN_I2C_SCL   11   // ADC SCL from schematic
// NOTE: GP11 is technically I2C1_SCL and GP12 is I2C0_SDA.
// If the ADS1115 fails to respond, swap to i2c1 with GP10(SDA)/GP11(SCL).
#define I2C_FREQ      400000

#define ADS_ADDR_A    0x48   // ADDR → GND  (FSR 0–3)
#define ADS_ADDR_B    0x49   // ADDR → VDD  (FSR 4)
#define ADS_REG_CONV  0x00
#define ADS_REG_CFG   0x01
// Config: OS=1 start, MUX set per channel, PGA=±4.096V, single-shot,
//         128 SPS, comparator disabled
#define ADS_CFG_BASE  0xC383
static const unsigned short ADS_MUX[4] = {0x4000, 0x5000, 0x6000, 0x7000};

// Tune this threshold to your FSR + 10kΩ divider.
// ±4.096V FS, 1LSB = 0.125mV; light touch ≈ 5000 counts.
#define FSR_THRESHOLD 5000

// ─── ONBOARD ADC ──────────────────────────────────────────────
#define ADC_CH_EMG    0   // GPIO26 — DFRobot EMG analog out

// EMG thresholds (12-bit, 0-4095) — calibrate per user
#define EMG_CLOSE_THRESH  2400
#define EMG_OPEN_THRESH    600

// ─── PROTOCOL ─────────────────────────────────────────────────
#define START_BYTE  0xAA

typedef enum {
    MODE_OFF    = 0x00,
    MODE_MANUAL = 0x01,
    MODE_PULSE  = 0x02,
    MODE_EMG    = 0x03
} GloveMode;

// Manual position command (sent in position byte)
typedef enum {
    POS_HOLD  = 0,
    POS_OPEN  = 1,
    POS_CLOSE = 2
} ManualCmd;

typedef struct __attribute__((packed)) {
    unsigned char start;
    unsigned char mode;
    unsigned char speed;
    unsigned char position;   // ManualCmd when in Manual mode
    unsigned char emg_cmd;    // 0=hold 1=open 2=close
    unsigned char fsr_flags;  // bit0–bit4
    unsigned char checksum;
} Packet;

// ─── GLOVE ENABLE STATE ───────────────────────────────────────
static volatile bool g_glove_enabled = false;
static uint32_t      onoff_last_ms   = 0;
#define DEBOUNCE_MS  200

// ─── ROTARY ENCODER ISR ───────────────────────────────────────
// Interrupt handler for rotary encoder speed updates.
// On falling edge of channel A, it samples channel B to determine direction,
// increments or decrements speed by SPEED_STEP, then clamps to valid bounds.
static void encoder_isr(uint gpio, uint32_t events) {
    if (gpio != PIN_ENC_A) return;
    // On falling edge of A: sample B to get direction
    bool b = gpio_get(PIN_ENC_B);
    g_speed += b ? SPEED_STEP : -SPEED_STEP;
    if (g_speed > SPEED_MAX) g_speed = SPEED_MAX;
    if (g_speed < SPEED_MIN) g_speed = SPEED_MIN;
}

// ─── ADS1115 (ADC) ──────────────────────────────────────────────────
// Writes a 16-bit value to an ADS1115 register over I2C.
// Message format is: register address, high byte, low byte.
static void ads_write_reg(unsigned char addr, unsigned char reg, unsigned short val) {
    unsigned char buf[3] = {reg, (unsigned char)(val >> 8), (unsigned char)(val & 0xFF)};
    i2c_write_blocking(I2C_PORT, addr, buf, 3, false);
}

// Reads the current ADS1115 conversion register as a signed 16-bit value.
// Performs register pointer write followed by a 2-byte read transaction.
static short ads_read_conv(unsigned char addr) {
    unsigned char reg = ADS_REG_CONV;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    unsigned char buf[2];
    i2c_read_blocking(I2C_PORT, addr, buf, 2, false);
    return (short)((buf[0] << 8) | buf[1]);
}

// Triggers and reads a single-shot conversion for one ADS1115 channel.
// Selects channel via MUX bits in config register, waits for conversion,
// then returns the converted sample.
static short ads_read_channel(unsigned char addr, unsigned char ch) {
    unsigned short cfg = ADS_CFG_BASE | ADS_MUX[ch & 0x03];
    ads_write_reg(addr, ADS_REG_CFG, cfg);
    sleep_ms(9);   // 128 SPS → 7.8ms conversion time
    return ads_read_conv(addr);
}

// Samples all FSR channels and packs touch/contact state into a bitmask.
// Bit 0..3 map to ADS1115-A channels 0..3; bit 4 maps to ADS1115-B channel 0.
// A bit is set when its reading exceeds FSR_THRESHOLD.
static unsigned char read_fsr_flags() {
    unsigned char flags = 0;
    for (int ch = 0; ch < 4; ch++) {
        if (ads_read_channel(ADS_ADDR_A, ch) > FSR_THRESHOLD)
            flags |= (1u << ch);
    }
    if (ads_read_channel(ADS_ADDR_B, 0) > FSR_THRESHOLD)
        flags |= (1u << 4);
    return flags;
}

// ─── ONBOARD ADC ───────────────────────────────────────────────
// Reads one onboard RP2040 ADC input channel.
// Selects the channel, allows brief analog mux settle time, then returns
// the raw ADC reading.
static inline unsigned short read_adc(unsigned char ch) {
    adc_select_input(ch);
    sleep_us(10);
    return adc_read();
}

// ─── PACKET SEND ─────────────────────────────────────────────
// Builds and transmits a controller packet to the motor Pico.
// Populates all packet fields, computes checksum as XOR of bytes [1..5],
// and writes the packed struct through UART.
static void send_packet(GloveMode mode, unsigned char speed, unsigned char position,
                        unsigned char emg_cmd, unsigned char fsr_flags) {
    Packet p;
    p.start     = START_BYTE;
    p.mode      = mode;
    p.speed     = speed;
    p.position  = position;
    p.emg_cmd   = emg_cmd;
    p.fsr_flags = fsr_flags;
    p.checksum  = p.mode ^ p.speed ^ p.position ^ p.emg_cmd ^ p.fsr_flags;
    uart_write_blocking(UART_PORT, (const unsigned char*)&p, sizeof(p));
}

// ─── INIT ─────────────────────────────────────────────────────
// Initializes all peripherals used by the controller firmware.
// Configures UART, encoder GPIO + interrupt, control button inputs,
// I2C bus for ADS1115 sensors, and onboard ADC for EMG sampling.
static void hw_init() {
    stdio_init_all();

    // UART0
    uart_init(UART_PORT, BAUD_RATE);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);

    // Rotary encoder — pull-ups, interrupt on falling CLK edge
    gpio_init(PIN_ENC_A);
    gpio_set_dir(PIN_ENC_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_A);

    gpio_init(PIN_ENC_B);
    gpio_set_dir(PIN_ENC_B, GPIO_IN);
    gpio_pull_up(PIN_ENC_B);

    gpio_set_irq_enabled_with_callback(
        PIN_ENC_A, GPIO_IRQ_EDGE_FALL, true, &encoder_isr);

    // Control buttons — pull-downs, active HIGH
    const unsigned int btn_pins[] = {
        PIN_GLOVE_ONOFF, PIN_BTN_DETECTION,
        PIN_BTN_MANUAL,  PIN_BTN_REHAB,
        PIN_BTN_GLOVE_CLOSE, PIN_BTN_GLOVE_OPEN
    };
    for (unsigned int pin : btn_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin);
    }

    // I2C for ADS1115
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // Onboard ADC — EMG only
    adc_init();
    adc_gpio_init(26);   // ADC0 = EMG

    printf("Controller Pico (Pico A) ready. Speed=%d%%\n", SPEED_DEFAULT);
}

// ─── MAIN ─────────────────────────────────────────────────────
// Main control loop.
// Reads user controls and sensors, resolves active mode/state,
// assembles outgoing command packet, and transmits it periodically.
int main() {
    hw_init();

    GloveMode current_mode = MODE_MANUAL;   // default to Manual

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // ── Glove ON/OFF (debounced toggle) ───────────────
        if (gpio_get(PIN_GLOVE_ONOFF) && (now - onoff_last_ms) > DEBOUNCE_MS) {
            g_glove_enabled = !g_glove_enabled;
            onoff_last_ms   = now;
            printf("[CTRL] Glove %s\n", g_glove_enabled ? "ON" : "OFF");
        }

        // ── Mode buttons (priority: Detection > Manual > Rehab) ──
        if (gpio_get(PIN_BTN_DETECTION))   current_mode = MODE_EMG;
        else if (gpio_get(PIN_BTN_MANUAL)) current_mode = MODE_MANUAL;
        else if (gpio_get(PIN_BTN_REHAB))  current_mode = MODE_PULSE;

        // If glove is off, override mode to OFF
        GloveMode tx_mode = g_glove_enabled ? current_mode : MODE_OFF;

        // ── Speed (from rotary encoder ISR, atomic read) ──
        uint32_t saved = save_and_disable_interrupts();
        unsigned char speed = (unsigned char)g_speed;
        restore_interrupts(saved);

        // ── Manual open/close buttons ─────────────────────
        unsigned char manual_cmd = POS_HOLD;
        if      (gpio_get(PIN_BTN_GLOVE_CLOSE)) manual_cmd = POS_CLOSE;
        else if (gpio_get(PIN_BTN_GLOVE_OPEN))  manual_cmd = POS_OPEN;

        // ── EMG (read only when in EMG mode) ──────────────
        unsigned char emg_cmd = 0;
        if (tx_mode == MODE_EMG) {
            unsigned short raw = read_adc(ADC_CH_EMG);
            if      (raw > EMG_CLOSE_THRESH) emg_cmd = 2;
            else if (raw < EMG_OPEN_THRESH)  emg_cmd = 1;
        }

        // ── FSRs via ADS1115 (~45ms for all 5 channels) ───
        unsigned char fsr_flags = read_fsr_flags();

        // ── Transmit ──────────────────────────────────────
        send_packet(tx_mode, speed, manual_cmd, emg_cmd, fsr_flags);

        printf("[CTRL] mode=%u spd=%u cmd=%u emg=%u fsr=0x%02X enabled=%d\n",
               tx_mode, speed, manual_cmd, emg_cmd, fsr_flags, g_glove_enabled);

        sleep_ms(5);
    }
}