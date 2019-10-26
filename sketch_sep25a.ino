// Downsampling by factor 2^4, Ts ~ 1.6ms (according to 10kHz measured)
#define MEAN_SHIFT 4
// Pin used to signal detection
#define PIN_DETECT 10
// First detection when difference between two downsampled samples is < DET_HL_THRESHOLD
// (pos->neg)
#define DET_HL_THRESHOLD -100
// Timeout in samples while waiting for rise/fall
#define DET_TIMEOUT 100
// Default number of samples in delay line and number of samples we look back when
// detecting slow rise/fall.  Warning: Will read this many samples after the initial
// downward spike before starting to scan for rising slope, so must not be too high. First
// "real" rising slope is about 40 samples after the spike.
#define DEFAULT_NLOOKBACK 15
// Maximum history
#define NHIST 150

// Next detection when voltage has slowly risen by det_rising_threshold, then fallen by
// det_falling_threshold
static int16_t det_rising_threshold = 20;
static int16_t det_falling_threshold = -20;

// Sample delay line
static int16_t adc_value_hist[NHIST];
static int16_t ptr;
static int16_t lookback_rising = DEFAULT_NLOOKBACK,
    lookback_falling = DEFAULT_NLOOKBACK;
// Get largest of the two lookbacks
#define LARGEST_LOOKBACK (lookback_rising > lookback_falling ? lookback_rising : lookback_falling)

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

// ADC interrupt service routine
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

    // initial state
    state = STATE_IDLE;
    state_prev = STATE_IDLE;

    // Set ADC prescaler to 128 => 125KHz sample rate @ 16MHz clk
    // update: Measured to around 10kHz sample rate (9.6kHz, actually) with adalm2k
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
    PORTB |= (1 << 5); // debug pin - is high while program idle
    while (!adc_ready) { } // poll until sample ready
    PORTB &= ~(1 << 5);
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
    delayMicroseconds(100);
    digitalWrite(PIN_DETECT, HIGH);
    ndet++;
}

// Handle delay line pointer wraparound, assuming idx is never more than nlookback off
static inline int16_t idx_wraparound(int16_t idx) {
    if (idx < 0) {
        return idx + NHIST;
    } else if (idx >= NHIST) {
        return idx - NHIST;
    } else {
        return idx;
    }
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
                // then reset and return to idle
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
