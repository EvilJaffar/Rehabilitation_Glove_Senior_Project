#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

typedef unsigned int uint;

// ===== I2C / ADS7830 CONFIG =====
#define I2C_PORT        i2c0
#define PIN_I2C_SCL     13
#define PIN_I2C_SDA     12
#define I2C_FREQ        100000
#define ADS7830_ADDR    0x48

// ADS7830 single-ended channel command bytes
static const unsigned char ADS7830_CMD[8] = {
    0x84, 0xC4, 0x94, 0xD4,
    0xA4, 0xE4, 0xB4, 0xF4
};

// FSR is considered "pressed" above this raw value (0-255, 8-bit ADC)
// Tune this to your FSR + resistor divider
#define FSR_THRESHOLD   50

// EMG is on ADS7830 channel 5 (channels 0-4 are FSRs)
// Use startup baseline calibration plus relative thresholds so the
// EMG command logic works with a real (non-floating) sensor signal.
#define EMG_CHANNEL         5
#define EMG_CLOSE_DELTA     12    // close when filtered > baseline + delta
#define EMG_OPEN_DELTA      8     // open  when filtered < baseline + delta
#define EMG_FILTER_SHIFT    2     // 1/(2^n)=1/4 IIR smoothing
#define EMG_CONFIRM_SAMPLES 3     // require N consecutive samples

static int emg_baseline = 128;

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

// emg_cmd values
#define EMG_HOLD    0u
#define EMG_OPEN    1u
#define EMG_CLOSE   2u

// ===== MODE CONTROL GPIO =====
#define PIN_MODE_EMG      6    // HIGH = EMG mode
#define PIN_MODE_MANUAL   7    // HIGH = Manual mode

// ===== USER INTERFACE GPIO =====
#define SWITCH_OPEN_PIN         10  // Open the user's hand (Manual Mode only)
#define SWITCH_CLOSE_PIN        11  // Close the user's hand (Manual Mode only)
#define EMG_CALIBRATE_PIN       8  // Calibrate EMG baseline
#define ROTARY_ENCODER_A_PIN    13  // Rotary switch for speed adjustment (not implemented yet)
#define ROTARY_ENCODER_B_PIN    14  // Rotary switch for speed adjustment (not implemented yet)
#define GLOVE_START_PIN         15  // Start the glove operation
#define GLOVE_STOP_PIN          16  // Stop the glove operation

#define CALIBRATE_DEBOUNCE_MS   250u

// ===== ADS7830 READ =====
unsigned char ads7830_read_channel(unsigned char channel) {
    unsigned char cmd = ADS7830_CMD[channel & 0x07];
    unsigned char val = 0;
    i2c_write_blocking(I2C_PORT, ADS7830_ADDR, &cmd, 1, false);
    sleep_us(200);
    i2c_read_blocking(I2C_PORT, ADS7830_ADDR, &val, 1, false);
    return val;
}

// ===== SEND UART PACKET =====
void send_packet(unsigned char mode, unsigned char emg_cmd, unsigned char fsr_flags) {
    unsigned char pkt[7];
    pkt[0] = START_BYTE;
    pkt[1] = mode;
    pkt[2] = 0u;        // speed
    pkt[3] = 0u;        // position
    pkt[4] = emg_cmd;
    pkt[5] = fsr_flags;
    pkt[6] = pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5]; // checksum
    uart_write_blocking(UART_PORT, pkt, 7);
}

static void calibrate_emg_baseline() {
    // Keep your hand relaxed during this short sampling window.
    int sum = 0;
    const int samples = 64;
    for (int i = 0; i < samples; i++) {
        sum += ads7830_read_channel(EMG_CHANNEL);
        sleep_ms(5);
    }
    emg_baseline = sum / samples;
    if (emg_baseline < 10) emg_baseline = 10;
    if (emg_baseline > 245) emg_baseline = 245;
}

// Reject short spikes by smoothing EMG and requiring repeated threshold hits.
unsigned char get_emg_cmd_filtered(unsigned char* raw_out, unsigned char* filt_out) {
    static int emg_filt = 128;
    static unsigned char close_hits = 0;
    static unsigned char open_hits = 0;

    unsigned char raw = ads7830_read_channel(EMG_CHANNEL);
    emg_filt += ((int)raw - emg_filt) >> EMG_FILTER_SHIFT;

    int close_thresh = emg_baseline + EMG_CLOSE_DELTA;
    int open_thresh = emg_baseline + EMG_OPEN_DELTA;
    if (close_thresh > 250) close_thresh = 250;
    if (open_thresh < 0) open_thresh = 0;
    if (open_thresh >= close_thresh) open_thresh = close_thresh - 1;

    unsigned char cmd = EMG_HOLD;
    if (emg_filt > close_thresh) {
        if (close_hits < 255) close_hits++;
        open_hits = 0;
        if (close_hits >= EMG_CONFIRM_SAMPLES) {
            cmd = EMG_CLOSE;
            close_hits = 0;
        }
    } else if (emg_filt < open_thresh) {
        if (open_hits < 255) open_hits++;
        close_hits = 0;
        if (open_hits >= EMG_CONFIRM_SAMPLES) {
            cmd = EMG_OPEN;
            open_hits = 0;
        }
    } else {
        close_hits = 0;
        open_hits = 0;
    }

    if (raw_out)  *raw_out = raw;
    if (filt_out) *filt_out = (unsigned char)emg_filt;
    return cmd;
}

// ===== MAIN =====
int main() {
    stdio_init_all();
    sleep_ms(2000);

    // I2C init
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SCL);
    gpio_pull_up(PIN_I2C_SDA);

    // UART init
    uart_init(UART_PORT, BAUD_RATE);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);

        calibrate_emg_baseline();
        printf("EMG baseline=%d (close>%d open<%d)\n",
            emg_baseline,
            emg_baseline + EMG_CLOSE_DELTA,
            emg_baseline + EMG_OPEN_DELTA);

    // GPIO mode inputs
    gpio_init(PIN_MODE_EMG);
    gpio_set_dir(PIN_MODE_EMG, GPIO_IN);
    gpio_pull_down(PIN_MODE_EMG);
    
    gpio_init(PIN_MODE_MANUAL);
    gpio_set_dir(PIN_MODE_MANUAL, GPIO_IN);
    gpio_pull_down(PIN_MODE_MANUAL);

    // Calibrate input: set HIGH to re-capture a relaxed EMG baseline.
    gpio_init(EMG_CALIBRATE_PIN);
    gpio_set_dir(EMG_CALIBRATE_PIN, GPIO_IN);
    gpio_pull_down(EMG_CALIBRATE_PIN);

    printf("FSR + EMG + Manual Mode Ready\n");

    bool calibrate_prev = false;
    uint32_t calibrate_last_ms = 0;

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        unsigned char fsr_flags = 0;
        bool mode_emg = gpio_get(PIN_MODE_EMG);
        bool mode_manual = gpio_get(PIN_MODE_MANUAL);
        bool calibrate_now = gpio_get(EMG_CALIBRATE_PIN);

        // Read FSR channels 0-4 from ADS7830
        for (int i = 0; i < 5; i++) {
            unsigned char val = ads7830_read_channel((unsigned char)i);
            if (val > FSR_THRESHOLD) {
                fsr_flags |= (unsigned char)(1u << i);
            }
            printf("FSR%d=%3u ", i, (unsigned int)val);
        }
        printf("\n");
        
        // === EMG MODE (GPIO 5 HIGH) ===
        if (mode_emg) {
            unsigned char emg_raw = 0;
            unsigned char emg_filt = 0;
            unsigned char emg_cmd = get_emg_cmd_filtered(&emg_raw, &emg_filt);
             printf("[EMG MODE] raw=%3u filt=%3u base=%3d cmd=%u\n",
                   (unsigned int)emg_raw,
                   (unsigned int)emg_filt,
                 emg_baseline,
                   (unsigned int)emg_cmd);

            // Calibrate EMG baseline if pin is pressed
            if (mode_emg && calibrate_now && !calibrate_prev
            && (now_ms - calibrate_last_ms) > CALIBRATE_DEBOUNCE_MS) {
            printf("[CAL] Calibrate pin HIGH -> measuring relaxed EMG baseline...\n");
            // Keep motors stopped while baseline is being refreshed.
            send_packet(MODE_OFF, EMG_HOLD, 0u);
            calibrate_emg_baseline();
            calibrate_last_ms = now_ms;
            printf("[CAL] New baseline=%d (close>%d open<%d)\n",
                   emg_baseline,
                   emg_baseline + EMG_CLOSE_DELTA,
                   emg_baseline + EMG_OPEN_DELTA);
            calibrate_prev = calibrate_now;
            sleep_ms(20);
            continue;
             }
            calibrate_prev = calibrate_now;

            // FSR acts as safety stop: if contact detected, stop immediately
            if (fsr_flags != 0) {
                send_packet(MODE_OFF, EMG_HOLD, fsr_flags);
                printf("[UART] FSR contact (0x%02X) — Emergency stop\n", fsr_flags);
            } else {
                send_packet(MODE_EMG, emg_cmd, 0);
                printf("[UART] EMG cmd=%u sent\n", (unsigned int)emg_cmd);
            }
        }
        // === MANUAL MODE (GPIO 6 HIGH) ===
        else if (mode_manual) {
            // Read the 2-position momentary switch from EMG channel (repurposed)
            unsigned char manual_raw = ads7830_read_channel(EMG_CHANNEL);
            unsigned char manual_cmd = EMG_HOLD;
            int manual_close_thresh = emg_baseline + EMG_CLOSE_DELTA;
            int manual_open_thresh = emg_baseline + EMG_OPEN_DELTA;
            if (manual_close_thresh > 250) manual_close_thresh = 250;
            if (manual_open_thresh < 0) manual_open_thresh = 0;
            
            if ((int)manual_raw > manual_close_thresh) {
                manual_cmd = EMG_CLOSE;  // Switch position 1
            } else if ((int)manual_raw < manual_open_thresh) {
                manual_cmd = EMG_OPEN;   // Switch position 2
            }
            printf("[MANUAL MODE] Switch=%3u cmd=%u\n", (unsigned int)manual_raw, (unsigned int)manual_cmd);

            // FSR acts as safety stop
            if (fsr_flags != 0) {
                send_packet(MODE_OFF, EMG_HOLD, fsr_flags);
                printf("[UART] FSR contact (0x%02X) — Emergency stop\n", fsr_flags);
            } else {
                send_packet(MODE_EMG, manual_cmd, 0);
                printf("[UART] Manual cmd=%u sent\n", (unsigned int)manual_cmd);
            }
        }
        // === DEFAULT MODE ===
        else {
            unsigned char emg_raw = 0;
            unsigned char emg_filt = 0;
            unsigned char emg_cmd = get_emg_cmd_filtered(&emg_raw, &emg_filt);
            printf("[DEFAULT MODE] raw=%3u filt=%3u cmd=%u\n",
                   (unsigned int)emg_raw,
                   (unsigned int)emg_filt,
                   (unsigned int)emg_cmd);

            // Default: FSR contact overrides EMG
            if (fsr_flags != 0) {
                send_packet(MODE_OFF, EMG_HOLD, fsr_flags);
                printf("[UART] FSR contact (0x%02X) — Stop sent\n", fsr_flags);
            } else {
                send_packet(MODE_EMG, emg_cmd, 0);
                printf("[UART] EMG cmd=%u sent\n", (unsigned int)emg_cmd);
            }
        }

        sleep_ms(20);
    }

    return 0;
}