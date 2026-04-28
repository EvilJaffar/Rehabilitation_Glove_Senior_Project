#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

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
#define FSR_THRESHOLD   50  // *Needs to be tuned when glove is built

// EMG is on ADS7830 channel 5 (channels 0-4 are FSRs)
// Use startup baseline calibration plus relative thresholds so the
// EMG command logic works with a real (non-floating) sensor signal.
#define EMG_CHANNEL         5
#define EMG_CLOSE_DELTA     4     // close when filtered > baseline + delta (tuned for ~100mV idle, small swing signals)
#define EMG_OPEN_DELTA      2     // open  when filtered < baseline - delta (keep small: idle is near ADC floor)
#define EMG_FILTER_SHIFT    1     // 1/(2^n)=1/2 IIR smoothing — faster response for weak signals
#define EMG_CONFIRM_SAMPLES 2     // require N consecutive samples (lower = more sensitive, raise if false triggers)

static int emg_baseline = 128;

// ===== UART CONFIG =====
#define UART_PORT   uart0
#define BAUD_RATE   115200
#define PIN_TX      0
#define PIN_RX      1

// ===== PACKET PROTOCOL =====
#define START_BYTE  0xAAu
#define MODE_MANUAL 0x01u
#define MODE_EMG    0x02u
#define MODE_REHAB  0x03u
#define MODE_HOMING 0x04u
#define ENC_FB_START_BYTE 0xBBu

// motor_cmd values
#define MOTOR_CMD_STOP    0u
#define MOTOR_CMD_OPEN    1u
#define MOTOR_CMD_CLOSE   2u

// ===== MODE CONTROL GPIO =====
#define PIN_MODE_EMG      6    // HIGH = EMG mode
#define PIN_MODE_REHAB    8    // HIGH = Rehab mode
#define PIN_MODE_MANUAL   7    // HIGH = Manual mode

// ===== USER INTERFACE GPIO =====
#define SWITCH_OPEN_PIN         10  // Open the user's hand (Manual Mode only)
#define SWITCH_CLOSE_PIN        11  // Close the user's hand (Manual Mode only)
#define EMG_CALIBRATE_PIN       9   // Calibrate EMG baseline
#define ROTARY_ENCODER_A_PIN    2   // Rotary encoder A (clock)
#define ROTARY_ENCODER_B_PIN    3   // Rotary encoder B (data)
#define GLOVE_START_PIN         15  // Rotary encoder push button (toggle run/stop)
#define HOMING_CALIBRATE_PIN    14  // Calibrate home position/maping out range of motion
#define HOMING_PIN              16  // Moves the Motors back to home before starting their functions

// Manual switch electrical polarity.
// 1: switch is active-low (pressed reads 0)
// 0: switch is active-high (pressed reads 1)
#define MANUAL_SWITCH_ACTIVE_LOW 1

#define CALIBRATE_DEBOUNCE_MS   250u   // Minimum ms between EMG calibration button presses (debounce)
#define CALIBRATE_DURATION_MS  10000u  // Total duration (ms) of the EMG calibration window
#define CALIBRATE_SAMPLE_MS       10u  // Interval (ms) between EMG samples during calibration

#define HOMING_DEBOUNCE_MS       250u  // Minimum ms between homing button presses (debounce)
#define HOMING_SPEED_PCT          15u  // Motor speed percentage used during the homing sequence
#define HOMING_STALL_MS          500u  // Time (ms) with no encoder movement before declaring a stall
#define HOMING_PHASE_TIMEOUT_MS 12000u // Max time (ms) allowed for a single homing phase before failure
#define HOMING_FSR_CONFIRM_SAMPLES 5u  // Number of consecutive FSR readings required to confirm contact
#define HOMING_MIN_TRAVEL_COUNTS  20   // Minimum encoder counts a motor must travel to be considered moving
#define HOMING_RETURN_TOL_COUNTS   4   // Encoder count tolerance when returning to the home position
#define ROM_LIMIT_TOL_COUNTS       4   // Encoder tolerance used to stop before calibrated ROM endpoints

#define ENCODER_STALE_MS         300u  // Time (ms) after which the last encoder reading is considered stale

#define HOMING_NV_MAGIC 0x484F4D45u    // Magic number ('HOME') used to validate stored homing data in flash
#define HOMING_NV_VERSION 1u           // Version of the homing data structure stored in flash

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)  // Default flash size (2 MB) if not defined by the SDK
#endif
#define HOMING_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  // Flash address of the last sector, used to persist homing data

#define BUTTON_DEBOUNCE_MS        80u  // Minimum ms between open/close/start button presses (debounce)
#define SPEED_MIN_PCT            10u   // Minimum allowable motor speed (percent)
#define SPEED_MAX_PCT           100u   // Maximum allowable motor speed (percent)
#define SPEED_DEFAULT_PCT        40u   // Default motor speed on startup (percent)

// 4-state gray-code lookup table for quadrature decoding.
// Index = (prev_AB << 2) | curr_AB; value = direction step.
static const signed char ENC_LUT[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

typedef enum {
    HOMING_IDLE = 0,
    HOMING_OPENING,
    HOMING_CLOSING,
    HOMING_RETURN_HOME,
    HOMING_COMPLETE,
    HOMING_FAILED
} homing_state_t;

static int g_encoder_count[5] = {0};
static bool g_encoder_valid = false;
static uint32_t g_encoder_last_rx_ms = 0;

typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t open_count[5];
    int32_t close_count[5];
    uint8_t calibrated[5];
    uint8_t reserved[3];
    uint32_t checksum;
} homing_nv_t;

// Returns the absolute value of an integer.
static int abs_i(int x) {
    return (x < 0) ? -x : x;
}

// Returns the sign of an integer: 1 for positive, -1 for negative, 0 for zero.
static int sign_i(int x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Checks if at least one motor has been calibrated for range-of-motion (ROM).
static bool has_any_rom_calibration(const bool calibrated[5]) {
    for (int i = 0; i < 5; i++) {
        if (calibrated[i]) return true;
    }
    return false;
}

// Checks if a requested open/close command would exceed the calibrated range-of-motion
// (ROM) limits for any motor. Returns true if the command would push past a ROM boundary.
static bool cmd_hits_rom_limit(unsigned char desired_cmd,
                               const int curr_count[5],
                               const int open_count[5],
                               const int close_count[5],
                               const bool calibrated[5]) {
    if (desired_cmd != MOTOR_CMD_OPEN && desired_cmd != MOTOR_CMD_CLOSE) {
        return false;
    }

    for (int i = 0; i < 5; i++) {
        if (!calibrated[i]) continue;

        const int curr = curr_count[i];
        const int open_v = open_count[i];
        const int close_v = close_count[i];
        const int dir_to_open = sign_i(open_v - close_v);

        if (dir_to_open == 0) {
            continue;
        }

        if (desired_cmd == MOTOR_CMD_OPEN) {
            if ((dir_to_open > 0 && curr >= (open_v - ROM_LIMIT_TOL_COUNTS)) ||
                (dir_to_open < 0 && curr <= (open_v + ROM_LIMIT_TOL_COUNTS))) {
                return true;
            }
        } else { // desired_cmd == MOTOR_CMD_CLOSE
            if ((dir_to_open > 0 && curr <= (close_v + ROM_LIMIT_TOL_COUNTS)) ||
                (dir_to_open < 0 && curr >= (close_v - ROM_LIMIT_TOL_COUNTS))) {
                return true;
            }
        }
    }

    return false;
}

// Checks if the rehab mode operation would hit a ROM edge. Since rehab alternates
// direction, this stops if any calibrated motor is already at either ROM endpoint.
static bool rehab_hits_rom_edge(const int curr_count[5],
                                const int open_count[5],
                                const int close_count[5],
                                const bool calibrated[5]) {
    for (int i = 0; i < 5; i++) {
        if (!calibrated[i]) continue;

        const int curr = curr_count[i];
        const int lo = (open_count[i] < close_count[i]) ? open_count[i] : close_count[i];
        const int hi = (open_count[i] > close_count[i]) ? open_count[i] : close_count[i];

        if (curr <= (lo + ROM_LIMIT_TOL_COUNTS) || curr >= (hi - ROM_LIMIT_TOL_COUNTS)) {
            return true;
        }
    }

    return false;
}

// Computes XOR checksum of the homing calibration data for flash storage validation.
static uint32_t homing_checksum(const homing_nv_t* nv) {
    uint32_t sum = 0;
    const uint32_t* p = (const uint32_t*)nv;
    const int words = (int)(sizeof(homing_nv_t) / sizeof(uint32_t)) - 1;
    for (int i = 0; i < words; i++) {
        sum ^= p[i];
    }
    return sum;
}

// Loads previously saved homing calibration (open/close positions and ROM data) from flash.
// Returns true if valid data was found and checksum verified; false if flash is empty or corrupted.
static bool load_homing_from_flash(int open_count[5], int close_count[5], bool calibrated[5]) {
    const uint8_t* flash_ptr = (const uint8_t*)(XIP_BASE + HOMING_FLASH_OFFSET);
    homing_nv_t nv;
    memcpy(&nv, flash_ptr, sizeof(nv));

    if (nv.magic != HOMING_NV_MAGIC || nv.version != HOMING_NV_VERSION) {
        return false;
    }
    if (homing_checksum(&nv) != nv.checksum) {
        return false;
    }

    for (int i = 0; i < 5; i++) {
        open_count[i] = (int)nv.open_count[i];
        close_count[i] = (int)nv.close_count[i];
        calibrated[i] = (nv.calibrated[i] != 0u);
    }
    return true;
}

// Erases the homing calibration sector in flash, preparing for a fresh calibration.
static void clear_homing_in_flash(void) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(HOMING_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

// Persists the homing calibration (open/close encoder positions and ROM validity flags)
// to flash memory with magic number, version, and checksum for integrity validation.
static void save_homing_to_flash(const int open_count[5], const int close_count[5], const bool calibrated[5]) {
    uint8_t sector_buf[FLASH_SECTOR_SIZE];
    memset(sector_buf, 0xFF, sizeof(sector_buf));

    homing_nv_t* nv = (homing_nv_t*)sector_buf;
    nv->magic = HOMING_NV_MAGIC;
    nv->version = HOMING_NV_VERSION;
    for (int i = 0; i < 5; i++) {
        nv->open_count[i] = (int32_t)open_count[i];
        nv->close_count[i] = (int32_t)close_count[i];
        nv->calibrated[i] = calibrated[i] ? 1u : 0u;
    }
    nv->checksum = homing_checksum(nv);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(HOMING_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(HOMING_FLASH_OFFSET, sector_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

// Returns true if encoder feedback from the Motor Pico is valid and recently received
// (within ENCODER_STALE_MS); otherwise returns false indicating stale/missing data.
static bool encoder_feedback_fresh(uint32_t now_ms) {
    return g_encoder_valid && ((now_ms - g_encoder_last_rx_ms) <= ENCODER_STALE_MS);
}

// ===== ADS7830 READ =====
// Reads a single 8-bit analog value from the specified ADS7830 ADC channel (0-7).
// Used for FSR and EMG sensor inputs via I2C interface.
unsigned char ads7830_read_channel(unsigned char channel) {
    unsigned char cmd = ADS7830_CMD[channel & 0x07];
    unsigned char val = 0;
    i2c_write_blocking(I2C_PORT, ADS7830_ADDR, &cmd, 1, false);
    sleep_us(200);
    i2c_read_blocking(I2C_PORT, ADS7830_ADDR, &val, 1, false);
    return val;
}

// ===== SEND UART PACKET =====
// Transmits a 7-byte control packet to the Motor Pico over UART.
// Packet format: [0]=0xAA [1]=mode [2]=speed_pct [3]=position [4]=motor_cmd [5]=fsr_flags [6]=xor_checksum
// This packet controls motor operation, speed, and communicates FSR safety data.
void send_packet(unsigned char mode, unsigned char motor_cmd,unsigned char speed_pct, unsigned char position,
                unsigned char fsr_flags) {
    unsigned char pkt[7];
    pkt[0] = START_BYTE;
    pkt[1] = mode;
    pkt[2] = speed_pct; // speed (0-100 %)
    pkt[3] = position;  // position (reserved)
    pkt[4] = motor_cmd;
    pkt[5] = fsr_flags;
    pkt[6] = pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5]; // checksum
    uart_write_blocking(UART_PORT, pkt, 7);
}

// Processes incoming 12-byte encoder feedback packets from Motor Pico over UART.
// Packet format: [0]=0xBB [1..10]=five int16_t encoder counts [11]=xor_checksum
// Extracts encoder positions and validates checksum; updates global encoder state and timestamp.
static void process_motor_feedback(void) {
    static unsigned char fb_buf[12];
    static int fb_idx = 0;

    while (uart_is_readable(UART_PORT)) {
        unsigned char b = uart_getc(UART_PORT);

        if (fb_idx == 0 && b != ENC_FB_START_BYTE) {
            continue;
        }

        fb_buf[fb_idx++] = b;

        if (fb_idx >= 12) {
            fb_idx = 0;

            unsigned char chk = 0;
            for (int i = 1; i <= 10; i++) {
                chk ^= fb_buf[i];
            }
            if (chk != fb_buf[11]) {
                continue;
            }

            for (int i = 0; i < 5; i++) {
                int16_t c = (int16_t)((uint16_t)fb_buf[1 + i * 2]
                                    | ((uint16_t)fb_buf[1 + i * 2 + 1] << 8));
                g_encoder_count[i] = (int)c;
            }
            g_encoder_valid = true;
            g_encoder_last_rx_ms = to_ms_since_boot(get_absolute_time());
        }
    }
}

// Clamps motor speed percentage to the safe operational range [SPEED_MIN_PCT, SPEED_MAX_PCT].
// Ensures user speed adjustments remain within defined limits.
static unsigned char clamp_speed_pct(int speed) {
    if (speed < (int)SPEED_MIN_PCT) return SPEED_MIN_PCT;
    if (speed > (int)SPEED_MAX_PCT) return SPEED_MAX_PCT;
    return (unsigned char)speed;
}

// Performs a quick EMG baseline calibration at startup (64 samples over ~320ms).
// Captures the resting EMG signal so subsequent thresholds are relative to user baseline.
//static void calibrate_emg_baseline() {
//    int sum = 0;
//    const int samples = 64;
//    for (int i = 0; i < samples; i++) {
//        sum += ads7830_read_channel(EMG_CHANNEL);
//        sleep_ms(5);
//    }
//    emg_baseline = sum / samples;
//    if (emg_baseline < 10) emg_baseline = 10;
//    if (emg_baseline > 245) emg_baseline = 245;
//}

// Performs a long on-demand EMG baseline calibration (10 seconds, user-triggered).
// Samples EMG extensively and prints countdown to console. Used to re-center baseline
// if user conditions change or signal needs adjustment.
static void calibrate_emg_baseline_timed() {
    printf("[CAL] Starting 10s calibration. Keep hand relaxed...\n");

    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_print_s = 0;
    unsigned int sum = 0;
    unsigned int samples = 0;

    while ((to_ms_since_boot(get_absolute_time()) - start_ms) < CALIBRATE_DURATION_MS) {
        // Keep motors disabled throughout timed calibration.
        send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);

        sum += ads7830_read_channel(EMG_CHANNEL);
        samples++;

        uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - start_ms;
        uint32_t elapsed_s = elapsed_ms / 1000u;
        if (elapsed_s != last_print_s) {
            last_print_s = elapsed_s;
            printf("[CAL] %lus/%lus\n",
                   (unsigned long)elapsed_s,
                   (unsigned long)(CALIBRATE_DURATION_MS / 1000u));
        }

        sleep_ms(CALIBRATE_SAMPLE_MS);
    }

    if (samples == 0) {
        samples = 1;
    }

    emg_baseline = (int)(sum / samples);
    if (emg_baseline < 10) emg_baseline = 10;
    if (emg_baseline > 245) emg_baseline = 245;

    printf("[CAL] Done. New baseline=%d (close>%d open<%d)\n",
           emg_baseline,
           emg_baseline + EMG_CLOSE_DELTA,
           emg_baseline - EMG_OPEN_DELTA);
}

// Computes a debounced EMG-based motor command using IIR filtering and confirmation logic.
// Rejects short noise spikes by requiring EMG_CONFIRM_SAMPLES consecutive threshold hits.
// Returns MOTOR_CMD_CLOSE when EMG > baseline+delta, MOTOR_CMD_OPEN when < baseline-delta, else MOTOR_CMD_STOP.
unsigned char get_emg_cmd_filtered(unsigned char* raw_out, unsigned char* filt_out) {
    static int emg_filt = 128;
    static unsigned char close_hits = 0;
    static unsigned char open_hits = 0;

    unsigned char raw = ads7830_read_channel(EMG_CHANNEL);
    emg_filt += ((int)raw - emg_filt) >> EMG_FILTER_SHIFT;

    int close_thresh = emg_baseline + EMG_CLOSE_DELTA;
    int open_thresh = emg_baseline - EMG_OPEN_DELTA;
    if (close_thresh > 250) close_thresh = 250;
    if (open_thresh < 0) open_thresh = 0;
    if (open_thresh >= close_thresh) open_thresh = close_thresh - 1;

    unsigned char cmd = MOTOR_CMD_STOP;
    if (emg_filt > close_thresh) {
        if (close_hits < 255) close_hits++;
        open_hits = 0;
        if (close_hits >= EMG_CONFIRM_SAMPLES) {
            cmd = MOTOR_CMD_CLOSE;
            close_hits = 0;
        }
    } else if (emg_filt < open_thresh) {
        if (open_hits < 255) open_hits++;
        close_hits = 0;
        if (open_hits >= EMG_CONFIRM_SAMPLES) {
            cmd = MOTOR_CMD_OPEN;
            open_hits = 0;
        }
    } else {
        close_hits = 0;
        open_hits = 0;
    }

    // DEBUG: comment this out after tuning is complete
    printf("[EMG] raw=%3d filt=%3d base=%3d close_thr=%3d open_thr=%3d cmd=%d\n",
           raw, emg_filt, emg_baseline, close_thresh, open_thresh, (int)cmd);

    if (raw_out)  *raw_out = raw;
    if (filt_out) *filt_out = (unsigned char)emg_filt;
    return cmd;
}

// ===== MAIN =====
// Initializes hardware (I2C, UART, GPIO), loads saved homing calibration, and runs the main control loop.
// Continuously samples inputs (EMG, FSR, rotary encoder, mode pins), processes motor feedback,
// executes homing sequence if requested, and sends appropriate motor commands via UART based on active mode.
int main() {
    stdio_init_all();

    // Initialize critical control inputs before startup delay to avoid float-induced false edges.
    gpio_init(HOMING_CALIBRATE_PIN);
    gpio_set_dir(HOMING_CALIBRATE_PIN, GPIO_IN);
    gpio_pull_down(HOMING_CALIBRATE_PIN);
    gpio_init(HOMING_PIN);
    gpio_set_dir(HOMING_PIN, GPIO_IN);
    gpio_pull_down(HOMING_PIN);

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

        //calibrate_emg_baseline();
        printf("EMG baseline=%d (close>%d open<%d)\n",
            emg_baseline,
            emg_baseline + EMG_CLOSE_DELTA,
                emg_baseline - EMG_OPEN_DELTA);

    // GPIO mode inputs
    gpio_init(PIN_MODE_EMG);
    gpio_set_dir(PIN_MODE_EMG, GPIO_IN);
    gpio_pull_down(PIN_MODE_EMG);
    
    gpio_init(PIN_MODE_MANUAL);
    gpio_set_dir(PIN_MODE_MANUAL, GPIO_IN);
    gpio_pull_down(PIN_MODE_MANUAL);

    gpio_init(PIN_MODE_REHAB);
    gpio_set_dir(PIN_MODE_REHAB, GPIO_IN);
    gpio_pull_down(PIN_MODE_REHAB);

    // Calibrate input: set HIGH to re-capture a relaxed EMG baseline.
    gpio_init(EMG_CALIBRATE_PIN);
    gpio_set_dir(EMG_CALIBRATE_PIN, GPIO_IN);
    gpio_pull_down(EMG_CALIBRATE_PIN);

    // Manual open/close inputs.
    gpio_init(SWITCH_OPEN_PIN);
    gpio_set_dir(SWITCH_OPEN_PIN, GPIO_IN);
    gpio_pull_up(SWITCH_OPEN_PIN);
    gpio_init(SWITCH_CLOSE_PIN);
    gpio_set_dir(SWITCH_CLOSE_PIN, GPIO_IN);
    gpio_pull_up(SWITCH_CLOSE_PIN);

    // Homing button input (separate from EMG calibration).
    gpio_init(HOMING_CALIBRATE_PIN);
    gpio_set_dir(HOMING_CALIBRATE_PIN, GPIO_IN);
    gpio_pull_down(HOMING_CALIBRATE_PIN);
    gpio_init(HOMING_PIN);
    gpio_set_dir(HOMING_PIN, GPIO_IN);
    gpio_pull_down(HOMING_PIN);

    // Rotary encoder speed control (EMG, Rehab, and Manual modes)
    gpio_init(ROTARY_ENCODER_A_PIN);
    gpio_set_dir(ROTARY_ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ROTARY_ENCODER_A_PIN);
    gpio_init(ROTARY_ENCODER_B_PIN);
    gpio_set_dir(ROTARY_ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ROTARY_ENCODER_B_PIN);

    // Rotary push-button toggles run/stop in active control modes.
    gpio_init(GLOVE_START_PIN);
    gpio_set_dir(GLOVE_START_PIN, GPIO_IN);
    gpio_pull_up(GLOVE_START_PIN);

    printf("FSR + EMG + Manual Mode Ready\n");

    bool emg_calibrate_prev = false;
    uint32_t calibrate_last_ms = 0;
    bool motors_enabled = false;
    bool button_prev = (gpio_get(GLOVE_START_PIN) == 0);
    uint32_t button_last_ms = 0;
    unsigned char speed_pct = SPEED_DEFAULT_PCT;
    unsigned char enc_prev = (unsigned char)((gpio_get(ROTARY_ENCODER_A_PIN) << 1)
                                            | gpio_get(ROTARY_ENCODER_B_PIN));
    int enc_accum = 0;

    homing_state_t homing_state = HOMING_IDLE;
    uint32_t homing_state_start_ms = 0;
    uint32_t homing_trigger_last_ms = 0;
    bool homing_prev = (gpio_get(HOMING_PIN) != 0);
    bool homing_armed = false;
    int homing_open_count[5] = {0};
    int homing_close_count[5] = {0};
    bool homing_calibrated[5] = {false, false, false, false, false};
    bool homing_open_done[5] = {false, false, false, false, false};
    int homing_last_count[5] = {0};
    uint32_t homing_last_move_ms[5] = {0, 0, 0, 0, 0};
    unsigned char homing_fsr_hits[5] = {0, 0, 0, 0, 0};
    uint32_t rom_last_block_print_ms = 0;
    uint32_t manual_diag_last_ms = 0;

    if (load_homing_from_flash(homing_open_count, homing_close_count, homing_calibrated)) {
        printf("[HOME] Loaded saved calibration from flash\n");
    } else {
        printf("[HOME] No valid saved calibration\n");
    }

    while (true) {
        // Sample mode selection pins once per loop.
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        process_motor_feedback();

        unsigned char fsr_flags = 0;
        bool mode_emg = gpio_get(PIN_MODE_EMG);
        bool mode_manual = gpio_get(PIN_MODE_MANUAL);
        bool mode_rehab = gpio_get(PIN_MODE_REHAB);
        bool emg_calibrate_now = gpio_get(EMG_CALIBRATE_PIN);
        // Homing trigger is only from HOMING_PIN.
        bool homing_now = (gpio_get(HOMING_PIN) != 0);
        bool has_rom = has_any_rom_calibration(homing_calibrated);
        bool enc_fresh = encoder_feedback_fresh(now_ms);
        // Arm homing only after both homing inputs are observed LOW after boot.
        if (!homing_armed) {
            if (!homing_now) {
                homing_armed = true;
            }
            homing_prev = homing_now;
        }

        if (homing_armed && homing_now && !homing_prev
            && (now_ms - homing_trigger_last_ms) > HOMING_DEBOUNCE_MS
            && homing_state == HOMING_IDLE) {
            homing_trigger_last_ms = now_ms;

            // Every press resets previous home/range before recalibration.
            clear_homing_in_flash();

            homing_state = HOMING_OPENING;
            homing_state_start_ms = now_ms;

            if (!g_encoder_valid) {
                printf("[HOME] Waiting for encoder stream from Motor Pico...\n");
            }
            for (int i = 0; i < 5; i++) {
                homing_open_done[i] = false;
                homing_fsr_hits[i] = 0;
                homing_open_count[i] = g_encoder_count[i];
                homing_close_count[i] = g_encoder_count[i];
                homing_last_count[i] = g_encoder_count[i];
                homing_last_move_ms[i] = now_ms;
                homing_calibrated[i] = false;
            }

            printf("[HOME] Reset + start: opening to home until encoder stall...\n");
        }
        homing_prev = homing_now;

        // === ROTARY ENCODER (EMG, Rehab, and Manual modes) ===
        if ((mode_emg || mode_rehab || mode_manual) && homing_state == HOMING_IDLE) {
            // Quadrature decode using lookup table.
            unsigned char enc_now = (unsigned char)((gpio_get(ROTARY_ENCODER_A_PIN) << 1)
                                                   | gpio_get(ROTARY_ENCODER_B_PIN));
            signed char step = ENC_LUT[(enc_prev << 2) | enc_now];
            enc_prev = enc_now;
            if (step != 0) {
                enc_accum += (int)step;
                if (enc_accum >= 4) {
                    speed_pct = clamp_speed_pct((int)speed_pct + 1);
                    enc_accum = 0;
                    printf("[SPEED] %u%%\n", (unsigned int)speed_pct);
                } else if (enc_accum <= -4) {
                    speed_pct = clamp_speed_pct((int)speed_pct - 1);
                    enc_accum = 0;
                    printf("[SPEED] %u%%\n", (unsigned int)speed_pct);
                }
            }

            // Debounced push-button toggles motor run/stop only for EMG/Rehab.
            // Manual mode is hold-to-run from the OPEN/CLOSE inputs.
            if (mode_emg || mode_rehab) {
                bool button_now = (gpio_get(GLOVE_START_PIN) == 0);
                if (button_now != button_prev && (now_ms - button_last_ms) > BUTTON_DEBOUNCE_MS) {
                    button_last_ms = now_ms;
                    button_prev = button_now;
                    if (button_now) {
                        motors_enabled = !motors_enabled;
                        printf("[RUN] Motors %s\n", motors_enabled ? "ENABLED" : "STOPPED");
                        if (!motors_enabled) {
                            // Push an immediate hard-stop frame on toggle-off.
                            send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
                        }
                    }
                }
            } else {
                button_prev = (gpio_get(GLOVE_START_PIN) == 0);
            }
        }

        // Read FSR channels 0-4 from ADS7830
        for (int i = 0; i < 5; i++) {
            unsigned char val = ads7830_read_channel((unsigned char)i);
            if (val > FSR_THRESHOLD) {
                fsr_flags |= (unsigned char)(1u << i);
            }
            printf("FSR%d=%3u ", i, (unsigned int)val);
        }
        printf("\n");
        // Homing sequence overrides normal operating modes while active.
        if (homing_state == HOMING_OPENING) {
            if (!encoder_feedback_fresh(now_ms)) {
                homing_state = HOMING_FAILED;
                printf("[HOME] Encoder feedback lost during opening\n");
            } else {
                bool all_open_done = true;
                for (int i = 0; i < 5; i++) {
                    int curr = g_encoder_count[i];
                    int delta = abs_i(curr - homing_last_count[i]);
                    if (delta >= 1) {
                        homing_last_count[i] = curr;
                        homing_last_move_ms[i] = now_ms;
                    }
                    if (!homing_open_done[i]) {
                        if ((now_ms - homing_last_move_ms[i]) >= HOMING_STALL_MS) {
                            homing_open_done[i] = true;
                            homing_open_count[i] = curr;
                            printf("[HOME] M%d open=%d\n", i, curr);
                        } else {
                            all_open_done = false;
                        }
                    }
                }

                send_packet(MODE_HOMING, MOTOR_CMD_OPEN, HOMING_SPEED_PCT, 0u, 0u);

                if (all_open_done) {
                    homing_state = HOMING_CLOSING;
                    homing_state_start_ms = now_ms;
                    for (int i = 0; i < 5; i++) {
                        homing_fsr_hits[i] = 0;
                    }
                    printf("[HOME] Opening done. Closing until all FSRs confirm...\n");
                } else if ((now_ms - homing_state_start_ms) > HOMING_PHASE_TIMEOUT_MS) {
                    homing_state = HOMING_FAILED;
                }
            }
        }

        if (homing_state == HOMING_CLOSING) {
            if (!encoder_feedback_fresh(now_ms)) {
                homing_state = HOMING_FAILED;
                printf("[HOME] Encoder feedback lost during closing\n");
                } else {
                send_packet(MODE_HOMING, MOTOR_CMD_CLOSE, HOMING_SPEED_PCT, 0u, 0u);

                bool all_fsr_confirmed = true;
                for (int i = 0; i < 5; i++) {
                    bool fsr_on = ((fsr_flags & (1u << i)) != 0u);
                    if (fsr_on) {
                        if (homing_fsr_hits[i] < 255u) homing_fsr_hits[i]++;
                    } else {
                        homing_fsr_hits[i] = 0;
                    }

                    if (homing_fsr_hits[i] < HOMING_FSR_CONFIRM_SAMPLES) {
                        all_fsr_confirmed = false;
                    }
                }

                if (all_fsr_confirmed) {
                    for (int i = 0; i < 5; i++) {
                        homing_close_count[i] = g_encoder_count[i];
                        homing_calibrated[i] =
                            (abs_i(homing_close_count[i] - homing_open_count[i]) >= HOMING_MIN_TRAVEL_COUNTS);
                    }

                    save_homing_to_flash(homing_open_count, homing_close_count, homing_calibrated);

                    homing_state = HOMING_RETURN_HOME;
                    homing_state_start_ms = now_ms;
                    printf("[HOME] Close calibrated. Returning slowly to home...\n");
                } else if ((now_ms - homing_state_start_ms) > HOMING_PHASE_TIMEOUT_MS) {
                    homing_state = HOMING_FAILED;
                }
            }
        }

        if (homing_state == HOMING_RETURN_HOME) {
            if (!encoder_feedback_fresh(now_ms)) {
                homing_state = HOMING_FAILED;
                printf("[HOME] Encoder feedback lost while returning home\n");
            } else {
                bool all_at_home = true;
                for (int i = 0; i < 5; i++) {
                    int curr = g_encoder_count[i];
                    if (abs_i(curr - homing_open_count[i]) > HOMING_RETURN_TOL_COUNTS) {
                        all_at_home = false;
                        break;
                    }
                }

                if (all_at_home) {
                    send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
                    homing_state = HOMING_COMPLETE;
                } else {
                    send_packet(MODE_HOMING, MOTOR_CMD_OPEN, HOMING_SPEED_PCT, 0u, 0u);
                }

                if ((now_ms - homing_state_start_ms) > HOMING_PHASE_TIMEOUT_MS) {
                    homing_state = HOMING_FAILED;
                }
            }
        }

        if (homing_state == HOMING_COMPLETE) {
            printf("[HOME] Complete:\n");
            for (int i = 0; i < 5; i++) {
                printf("[HOME] M%d open=%d close=%d range=%d %s\n",
                       i,
                       homing_open_count[i],
                       homing_close_count[i],
                       homing_close_count[i] - homing_open_count[i],
                       homing_calibrated[i] ? "OK" : "LOW_TRAVEL");
            }
            homing_state = HOMING_IDLE;
            motors_enabled = false;
            sleep_ms(20);
            continue;
        }

        if (homing_state == HOMING_FAILED) {
            send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
            printf("[HOME] Failed or timed out. Motors stopped.\n");
            homing_state = HOMING_IDLE;
            motors_enabled = false;
            sleep_ms(20);
            continue;
        }
        if (homing_state != HOMING_IDLE) {
            sleep_ms(20);
            continue;
        }
        
        // === EMG MODE (GPIO 5 HIGH) ===
        if (mode_emg) {
            unsigned char emg_raw = 0;
            unsigned char emg_filt = 0;
            unsigned char motor_cmd = get_emg_cmd_filtered(&emg_raw, &emg_filt);
             printf("[EMG MODE] raw=%3u filt=%3u base=%3d cmd=%u\n",
                   (unsigned int)emg_raw,
                   (unsigned int)emg_filt,
                 emg_baseline,
                   (unsigned int)motor_cmd);

            // One-shot timed calibration on rising edge in EMG mode.
            if (emg_calibrate_now && !emg_calibrate_prev
                && (now_ms - calibrate_last_ms) > CALIBRATE_DEBOUNCE_MS) {
                calibrate_emg_baseline_timed();
                calibrate_last_ms = to_ms_since_boot(get_absolute_time());
                emg_calibrate_prev = emg_calibrate_now;
                sleep_ms(20);
                continue;
            }
            emg_calibrate_prev = emg_calibrate_now;

            // Safety priority: FSR stop > user stop-toggle > normal EMG command.
            if (fsr_flags != 0) {
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, fsr_flags);
                printf("[UART] FSR contact (0x%02X) — Emergency stop\n", fsr_flags);
            } else if (!motors_enabled) {
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
            } else if (has_rom && !enc_fresh) {
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
                if ((now_ms - rom_last_block_print_ms) > 400u) {
                    rom_last_block_print_ms = now_ms;
                    printf("[ROM] Encoder stale; blocking EMG motion until feedback recovers\n");
                }
            } else if (cmd_hits_rom_limit(motor_cmd, g_encoder_count, homing_open_count, homing_close_count, homing_calibrated)) {
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
                if ((now_ms - rom_last_block_print_ms) > 400u) {
                    rom_last_block_print_ms = now_ms;
                    printf("[ROM] EMG command blocked at calibrated endpoint\n");
                }
            } else {
                send_packet(MODE_EMG, motor_cmd, speed_pct, 0u, 0u);
                printf("[UART] EMG cmd=%u speed=%u%% sent\n",
                       (unsigned int)motor_cmd, (unsigned int)speed_pct);
            }
        }
        // === MANUAL MODE (GPIO 6 HIGH) ===
        else if (mode_manual) {
            unsigned char manual_cmd = MOTOR_CMD_STOP;
            bool open_level_high  = (gpio_get(SWITCH_OPEN_PIN)  != 0);
            bool close_level_high = (gpio_get(SWITCH_CLOSE_PIN) != 0);
#if MANUAL_SWITCH_ACTIVE_LOW
            bool open_pressed  = !open_level_high;
            bool close_pressed = !close_level_high;
#else
            bool open_pressed  = open_level_high;
            bool close_pressed = close_level_high;
#endif
            unsigned char manual_inputs = (unsigned char)((open_pressed ? 0x01u : 0u)
                                                         | (close_pressed ? 0x02u : 0u));

            // Manual direction still comes from open/close buttons.
            // Rotary encoder only changes speed_pct.
            if (open_pressed && !close_pressed) {
                manual_cmd = MOTOR_CMD_OPEN;
            } else if (close_pressed && !open_pressed) {
                manual_cmd = MOTOR_CMD_CLOSE;
            }
            else if ((!open_pressed && !close_pressed) || (open_pressed && close_pressed)) {
                manual_cmd = MOTOR_CMD_STOP;
            }
            printf("[MANUAL MODE] open=%u close=%u cmd=%u speed=%u%%\n",
                   (unsigned int)open_pressed, (unsigned int)close_pressed,
                   (unsigned int)manual_cmd,   (unsigned int)speed_pct);

            if ((now_ms - manual_diag_last_ms) > 300u) {
                if (open_pressed && close_pressed) {
                    manual_diag_last_ms = now_ms;
                    printf("[MANUAL MODE] OPEN and CLOSE are both HIGH -> forced STOP (check switch wiring/noise)\n");
                }
            }

            // Manual mode is hold-to-run: an active OPEN/CLOSE input directly drives motion.
            // FSR and ROM limits still override with STOP.
            if (fsr_flags != 0) {
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, manual_inputs, fsr_flags);
                printf("[UART] FSR contact (0x%02X) — Emergency stop\n", fsr_flags);
            //} else if (has_rom && !enc_fresh) {
            //    send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
            //    if ((now_ms - rom_last_block_print_ms) > 400u) {
            //        rom_last_block_print_ms = now_ms;
            //        printf("[ROM] Encoder stale; blocking manual motion until feedback recovers\n");
            //    }
            //} else if (cmd_hits_rom_limit(manual_cmd,
            //                              g_encoder_count, homing_open_count, homing_close_count, homing_calibrated)) {
            //    send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
            //    if ((now_ms - rom_last_block_print_ms) > 400u) {
            //        rom_last_block_print_ms = now_ms;
            //        printf("[ROM] Manual command blocked at calibrated endpoint\n");
            //    }
            } else if (manual_cmd == MOTOR_CMD_STOP) {
                // In manual mode, no active open/close input means hard stop.
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, manual_inputs, 0u);
            } else {
                // Unified packet format: mode determines command semantics.
                send_packet(MODE_MANUAL, manual_cmd, speed_pct, manual_inputs, 0u);
                printf("[UART] Manual cmd=%u speed=%u%% sent\n",
                       (unsigned int)manual_cmd, (unsigned int)speed_pct);
            }
        }
        // === REHAB MODE (GPIO 8 HIGH) ===
        else if (mode_rehab) {
            // Motor Pico handles the timed open/close cycle in this mode.
            // UI Pico still sends speed and global run/stop intent.
            if (fsr_flags != 0) {
                send_packet(MODE_REHAB, MOTOR_CMD_STOP, 0u, 0u, fsr_flags);
                printf("[UART] REHAB: FSR contact (0x%02X) - stop sent\n", fsr_flags);
            } else if (!motors_enabled) {
                send_packet(MODE_REHAB, MOTOR_CMD_STOP, 0u, 0u, 0u);
            } else if (has_rom && !enc_fresh) {
                send_packet(MODE_REHAB, MOTOR_CMD_STOP, 0u, 0u, 0u);
                if ((now_ms - rom_last_block_print_ms) > 400u) {
                    rom_last_block_print_ms = now_ms;
                    printf("[ROM] Encoder stale; blocking rehab motion until feedback recovers\n");
                }
            } else if (rehab_hits_rom_edge(g_encoder_count, homing_open_count, homing_close_count, homing_calibrated)) {
                send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, 0u);
                if ((now_ms - rom_last_block_print_ms) > 400u) {
                    rom_last_block_print_ms = now_ms;
                    printf("[ROM] Rehab blocked at calibrated endpoint\n");
                }
            } else {
                // Explicit rehab run token: Motor Pico only cycles when cmd == MOTOR_CMD_OPEN.
                send_packet(MODE_REHAB, MOTOR_CMD_OPEN, speed_pct, 0u, 0u);
                printf("[UART] REHAB mode active speed=%u%%\n", (unsigned int)speed_pct);
            }
        }
        // === DEFAULT MODE (fail-safe stop) ===
        else {
            send_packet(MODE_MANUAL, MOTOR_CMD_STOP, 0u, 0u, fsr_flags);
        }

        sleep_ms(20);
    }

    return 0;
}