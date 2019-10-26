#include <EEPROM.h>
#include "learning.h"
#include "defs.h"
#include "crc.h"

// Next detection when voltage has slowly risen by det_rising_threshold, then fallen by
// det_falling_threshold
int16_t det_rising_threshold = 25;
int16_t det_falling_threshold = -22;

// Number of samples in delay line and number of samples we look back when
// detecting slow rise/fall.  Warning: Will read LARGEST_LOOKBACK samples after the
// initial downward spike before starting to scan for rising slope, so must not be too
// high. EVO measurements suggest first "real" rising slope is about 40 samples after the
// spike.
int16_t lookback_rising = 15;
int16_t lookback_falling = 21;

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
    digitalWrite(PIN_DETECT, HIGH);

    unsigned long crc = eeprom_crc();
    unsigned long crc_rd;
    EEPROM.get(EEPROM_ADDR_CRC, crc_rd);
    if (crc_rd == crc) {
        Serial.println("CRC OK, loading config");
        EEPROM.get(EEPROM_ADDR_RT, det_rising_threshold);
        EEPROM.get(EEPROM_ADDR_FT, det_falling_threshold);
        EEPROM.get(EEPROM_ADDR_LR, lookback_rising);
        EEPROM.get(EEPROM_ADDR_LF, lookback_falling);
        Serial.print("lookback_rising: ");
        Serial.println(lookback_rising);
        Serial.print("lookback_falling: ");
        Serial.println(lookback_falling);
        Serial.print("det_rising_threshold: ");
        Serial.println(det_rising_threshold);
        Serial.print("det_falling_threshold: ");
        Serial.println(det_falling_threshold);
    } else {
        Serial.println("Invalid CRC, using defaults");
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
    // Turn on LED if we every loose a sample - except the first sample after running
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

    return ret;
}

void detection() {
    digitalWrite(PIN_DETECT, LOW);
    // TODO: timer with interrupt to clear the pin would be clever, here
    // not so critical with this low sample rate tho... longer pulse needed?
    delayMicroseconds(500);
    digitalWrite(PIN_DETECT, HIGH);
    ndet++;
}

void loop() {
    if (Serial.available() > 0) {
        char in = Serial.read();
        Serial.print(ndet);
        Serial.print(",");
        Serial.print(state);
        Serial.print(",");
        Serial.println(n_samples_in_state);

        if (in == 'l') {
            // Start learning mode, starting by flushing the delay line
            learning_mode = true;
            first = true;
            ptr = 0;
        } else if (in == 'c') {
            Serial.println("Clearing config CRC");
            for (unsigned int i = 0; i < sizeof(unsigned long); i++)
                EEPROM.write(EEPROM_ADDR_CRC+i, 0);
        }
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

    } else if (state == STATE_FILL_DELAYLINE) {
        // Wait for delay line to fill...
        if ((int16_t)n_samples_in_state >= (learning_mode ? NHIST : LARGEST_LOOKBACK)) {
            if (!learning_mode) {
                // start looking for a rising slope now
                state = STATE_DETECTING_RISING;
            } else {
                // in learning mode, just print all the samples instead
                Serial.print("[");
                for (unsigned int i = 0; i < NHIST; i++) {
                    Serial.print(adc_value_hist[idx_wraparound(ptr+i)]);
                    if (i != NHIST-1)
                        Serial.print(",");
                }
                Serial.println("]");
                // then attempt to guess some good thresholds
                learn(ptr, adc_value_hist, NHIST);
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
        int16_t then = adc_value_hist[idx_wraparound(ptr-lookback_rising)];

        // Waiting for significant rise in voltage between <now> and NHIST samples ago.
        // This should either be the initial voltage rise where battery voltage settles
        // to its nominal voltage (motor still running) or when the piston is released.
        if ((now - then) > det_rising_threshold) {
            state = STATE_DETECTING_FALLING;
        } else if (n_samples_in_state > DET_TIMEOUT) {
            // Timeout waiting for rising - go back to idle
            state = STATE_IDLE;
        }

    } else if (state == STATE_DETECTING_FALLING) {
        // Newest sample
        int16_t now = adc_value_hist[idx_wraparound(ptr-1)];

        // The sample from lookback_falling samples ago
        int16_t then = adc_value_hist[idx_wraparound(ptr-lookback_falling)];

        // Waiting for significant drop in voltage between <now> and NHIST samples ago.
        // This should be when the gears are drawing back the piston again.
        if ((now - then) < det_falling_threshold) {
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
