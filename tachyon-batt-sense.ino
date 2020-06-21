#include "common.h"
#include "learning.h"
#include "eeprom_profile.h"

// Profile holding detection thresholds
profile_t profile;

// Downsampling by factor 2^4, Ts ~ 1.6ms (according to 10kHz measured)
#define MEAN_SHIFT 4
// Pin used to signal detection
#define PIN_DETECT 10
// Pin used for button (pulled up internally)
#define PIN_BTN 2
// First detection when difference between two downsampled samples is < DET_HL_THRESHOLD
// (pos->neg)
#define DET_HL_THRESHOLD -100
// Timeout in samples while waiting for rise/fall
#define DET_TIMEOUT 100
// Get largest of the two lookbacks
#define LARGEST_LOOKBACK (profile.lookback_rising > profile.lookback_falling ? profile.lookback_rising : profile.lookback_falling)

// Sample delay line
static int16_t adc_value_hist[NHIST];
static int16_t ptr;

static volatile bool adc_ready = false; // new sample ready
static volatile int16_t adc_value;      // the new sample

// misc state
static bool first = true; // handle initialization
static uint16_t n_samples_in_state; // number of samples processed while in current state
static bool learning_mode = false; // learning voltage curve
enum state_t {STATE_IDLE, // waiting for first sudden voltage drop
              STATE_FILL_DELAYLINE, // waiting for delayline to fill
              STATE_DETECTING_RISING, // waiting for slow rise in voltage
              STATE_DETECTING_FALLING}; // waiting for slow drop in voltage
static state_t state, state_prev; // current and previous state of state machine
static uint16_t ndet = 0; // total number of detections
static bool first_sample_after_learning = false;
static bool do_debug = false; // do debug logging, only used if DEBUG is defined in common.h

// ADC interrupt service routine. Handles decimation.
ISR(ADC_vect)
{
    static uint32_t mean = 0;
    static uint16_t mean_cntr = 0;

    mean += ADC;
    mean_cntr++;

    if (mean_cntr & (1 << MEAN_SHIFT)) {
        adc_value = (int16_t)(mean >> MEAN_SHIFT);
        adc_ready = true;
        mean_cntr = 0;
        mean = 0;
    }
}

void setup() {
    Serial.begin(500000);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_DETECT, OUTPUT);
    pinMode(PIN_BTN, INPUT_PULLUP);
    digitalWrite(PIN_DETECT, HIGH);

    if (eeprom_check_crc()) {
        Serial.println("CRC OK, loading config");
        eeprom_load(&profile);
        Serial.print("lookback_rising: ");
        Serial.println(profile.lookback_rising);
        Serial.print("lookback_falling: ");
        Serial.println(profile.lookback_falling);
        Serial.print("det_rising_threshold: ");
        Serial.println(profile.det_rising_threshold);
        Serial.print("det_falling_threshold: ");
        Serial.println(profile.det_falling_threshold);
    } else {
        Serial.println("Invalid CRC, using defaults");
        // Some sensible defaults
        profile.det_rising_threshold = 25;
        profile.det_falling_threshold = -22;
        profile.lookback_rising = 15;
        profile.lookback_falling = 21;
    }

    // initial state
    state = STATE_IDLE;
    state_prev = STATE_IDLE;

    // With these settings, sample rate is measured to around 10kHz (9.6kHz?) with
    // ADALM2000
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    // Use AVcc as ref
    ADMUX = (1 << REFS0);
    // Use ADC0 input (MUX=0000)
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
    // Enable auto trigger (default free running)
    ADCSRA |= (1 << ADATE);
    // Enable ADC
    ADCSRA |= (1 << ADEN);
    // Enable ADC interrupt
    ADCSRA |= (1 << ADIE);
    // Enable interrupts
    sei();

    // Start conversion
    ADCSRA |= (1 << ADSC);
}

int16_t poll_sample() {
    // Turn on LED if we ever loose a sample - except the first sample after running
    // learning routine (expected loss).
    if (adc_ready && !first_sample_after_learning) {
        PORTB |= (1 << 5);
    }
    // PORTB |= (1 << 5); // debug pin - is high while program idle
    while (!adc_ready) { } // poll until sample ready
    // PORTB &= ~(1 << 5);
    cli();
    int16_t ret = adc_value;
    adc_ready = false;
    sei();

    // Clear flag if it was set
    first_sample_after_learning = false;

    return ret;
}

void detection() {
    digitalWrite(PIN_DETECT, LOW);
    delayMicroseconds(500);
    digitalWrite(PIN_DETECT, HIGH);
    ndet++;
}

void blink() {
    PORTB |= (1 << 5);
    delay(100);
    PORTB &= ~(1 << 5);
    delay(100);
}

void start_learning_mode() {
    // Start learning mode, starting by flushing the delay line
    Serial.println("Learning");
    learning_mode = true;
    first = true;
    ptr = 0;
}

void loop() {
    if (Serial.available() > 0) {
        char in = Serial.read();
        Serial.print(ndet);
        Serial.print(",");
        Serial.print(state);
        Serial.print(",");
        Serial.println(n_samples_in_state);

        if (in == 'l') {  // press l for learning mode
            start_learning_mode();
        } else if (in == 'c') {  // press c to clear profile
            Serial.println("Clearing config CRC");
            eeprom_clear();
        }
    }

    // Start learning mode when button pressed
    if (!learning_mode && !digitalRead(PIN_BTN)) {
        start_learning_mode();
    }

    // Poll until sample ready and manage delayline ptr
    adc_value_hist[ptr++] = poll_sample();
    if (ptr >= NHIST) {
        ptr = 0;
        first = false;
    }

    // Delay any decisions until delay line has been filled at least once
    if (first)
        return;

    // Manage n_samples_in_state
    if (state != state_prev)
        n_samples_in_state = 0;
    state_prev = state;
    n_samples_in_state++;

    if (state == STATE_IDLE) {
        // Just wrote a new sample to index (ptr-1) % NHIST
        // Previous sample is at (ptr-2) % NHIST
        int16_t diff = adc_value_hist[idx_wraparound(ptr-1)] -
                adc_value_hist[idx_wraparound(ptr-2)];

        // Large negative jump between two samples => initial detection (motor not running
        // and everything idle -> large initial current draw to start moving gears and piston)
        if (diff < DET_HL_THRESHOLD) {
            detection();
            state = STATE_FILL_DELAYLINE;
        }

#ifdef DEBUG
        if (do_debug) {
            Serial.print("[");
            for (unsigned int i = 0; i < NHIST; i++) {
                Serial.print(adc_value_hist[idx_wraparound(ptr+i)]);
                if (i != NHIST-1)
                    Serial.print(",");
            }
            Serial.println("]");

            do_debug = false;
        }
#endif

    } else if (state == STATE_FILL_DELAYLINE) {
        // Wait for delay line to fill...
        if ((int16_t)n_samples_in_state >= (learning_mode ? NHIST : LARGEST_LOOKBACK)) {
            if (!learning_mode) {
                // start looking for a rising slope now
                state = STATE_DETECTING_RISING;
#ifdef DEBUG
                do_debug = true;
#endif
            } else {
                // in learning mode, start by printing all the samples
                Serial.print("[");
                for (unsigned int i = 0; i < NHIST; i++) {
                    Serial.print(adc_value_hist[idx_wraparound(ptr+i)]);
                    if (i != NHIST-1)
                        Serial.print(",");
                }
                Serial.println("]");
                // then attempt to guess some good thresholds
                if (learn(ptr, adc_value_hist, NHIST, &profile)) {
                    // if that seems to go well, save the profile
                    eeprom_save(&profile);
                    // and blink twice
                    for (int i = 0; i < 2; i++)
                        blink();
                } else {
                    // blink thrice to indicate failure
                    for (int i = 0; i < 3; i++)
                        blink();
                }
                // then reset and return to idle
                first_sample_after_learning = true;
                learning_mode = false;
                ptr = 0;
                first = true;
                state = STATE_IDLE;
            }
        }

    } else if (state == STATE_DETECTING_RISING) {
        // Newest sample
        int16_t now = adc_value_hist[idx_wraparound(ptr-1)];

        // The sample from lookback_rising samples ago
        int16_t then = adc_value_hist[idx_wraparound(ptr-profile.lookback_rising)];

        // Waiting for significant rise in voltage between <now> and lookback_rising
        // samples ago. This should either be the initial voltage rise where battery
        // voltage settles to its nominal voltage (motor still running) or when the piston
        // is released.
        if ((now - then) > profile.det_rising_threshold) {
            state = STATE_DETECTING_FALLING;
        } else if (n_samples_in_state > DET_TIMEOUT) {
            // Timeout waiting for rising - go back to idle
            state = STATE_IDLE;
        }

    } else if (state == STATE_DETECTING_FALLING) {
        // Newest sample
        int16_t now = adc_value_hist[idx_wraparound(ptr-1)];

        // The sample from lookback_falling samples ago
        int16_t then = adc_value_hist[idx_wraparound(ptr-profile.lookback_falling)];

        // Waiting for significant drop in voltage between <now> and lookback_falling
        // samples ago. This should be when the gears are drawing back the piston again.
        if ((now - then) < profile.det_falling_threshold) {
            detection(); // trigger detection
            state = STATE_DETECTING_RISING;
        } else if (n_samples_in_state > DET_TIMEOUT) {
            // Timeout waiting for falling - go back to idle
            state = STATE_IDLE;
        }

    }
}

// Local Variables:
// mode: c++
// End:
