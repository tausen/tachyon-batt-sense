// Downsampling by factor 2^4, Ts ~ 1.6ms (according to 10kHz measured)
#define MEAN_SHIFT 4
// Pin used to signal detection
#define PIN_DETECT 10
// First detection when difference between two downsampled samples is < DET_HL_THRESHOLD
// (pos->neg)
#define DET_HL_THRESHOLD -100
// Timeout in samples while waiting for rise/fall
#define DET_TIMEOUT 100
// Maximum history. Must be high enough to capture samples from initial detection until
// end of burst for learning mode to work.
#define NHIST 150

// Next detection when voltage has slowly risen by det_rising_threshold, then fallen by
// det_falling_threshold
static int16_t det_rising_threshold = 25;
static int16_t det_falling_threshold = -22;

// Sample delay line
static int16_t adc_value_hist[NHIST];
static int16_t ptr;
// Default number of samples in delay line and number of samples we look back when
// detecting slow rise/fall.  Warning: Will read LARGEST_LOOKBACK samples after the
// initial downward spike before starting to scan for rising slope, so must not be too
// high. EVO measurements suggest first "real" rising slope is about 40 samples after the
// spike.
static int16_t lookback_rising = 15,
    lookback_falling = 21;
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

// Check whether data[ptr+i] is considered a local maximum (maxnmin=true) or minimum
// (maxnmin=false).
bool is_extrema(int i, int16_t ptr, int16_t *data, bool maxnmin, int16_t offset,
                int16_t diffthr, int16_t *incr) {
    if (maxnmin) {
        // max is where voltage decreases "significantly" looking far ahead and decreases
        // to some extent looking just a bit ahead
        if ((data[idx_wraparound(ptr+i)] - data[idx_wraparound(ptr+i+offset)] < -diffthr) &&
            (data[idx_wraparound(ptr+i+offset)] - data[idx_wraparound(ptr+i+offset+3)] < 0) &&
            (data[idx_wraparound(ptr+i+offset)] - data[idx_wraparound(ptr+i+offset+1)] < 0)) {
            *incr = 2; // best guess, empirical
            return true;
        } else {
            return false;
        }
    } else {
        // min is where voltage increases "significantly" looking far ahead and decreases
        // to some extent looking just a bit ahead
        if ((data[idx_wraparound(ptr+i)] - data[idx_wraparound(ptr+i+offset)] > diffthr) &&
            (data[idx_wraparound(ptr+i+offset)] - data[idx_wraparound(ptr+i+offset+3)] > 0) &&
            (data[idx_wraparound(ptr+i+offset)] - data[idx_wraparound(ptr+i+offset+1)] > 0)) {
            *incr = 2; // best guess, empirical
            return true;
        } else {
            return false;
        }
    }
}

bool learn(int16_t ptr, int16_t *data, uint16_t n) {
    const int16_t offset = 10; // minimum time offset (in samples) between extrema
    const int16_t diffthr = 10; // minimum voltage difference between extrema
    int16_t incr;

    // find the last minima (approx)
    uint16_t last_min_idx = 0;
    for (int i = n-1-offset; i >= 0; i--) {
        if (data[idx_wraparound(ptr+i)] - data[idx_wraparound(ptr+i+offset)] > diffthr) {
            last_min_idx = i+offset;
            break;
        }
    }

    if (last_min_idx == 0) {
        Serial.println("last_min not found");
        return false;
    }

    // wind back to closest maxima
    int16_t last_max = 0;
    uint16_t last_max_idx = 0;
    for (int i = last_min_idx-offset; i >= 0; i--) {
        if (is_extrema(i, ptr, data, true, offset, diffthr, &incr)) {
            last_max = data[idx_wraparound(ptr+i+offset+incr)];
            last_max_idx = i+offset+incr;
            break;
        }
    }

    if (last_max_idx == 0) {
        Serial.println("last_max not found");
        return false;
    }

    // wind back to closest minima
    int16_t mid_min = 0;
    uint16_t mid_min_idx = 0;
    for (int i = last_max_idx-offset; i >= 0; i--) {
        if (is_extrema(i, ptr, data, false, offset, diffthr, &incr)) {
            mid_min = data[idx_wraparound(ptr+i+offset+incr)];
            mid_min_idx = i+offset+incr;
            break;
        }
    }

    if (mid_min_idx == 0) {
        Serial.println("mid_min not found");
        return false;
    }

    // wind back to closest maxima
    int16_t first_max = 0;
    uint16_t first_max_idx = 0;
    for (int i = mid_min_idx-offset; i >= 0; i--) {
        if (is_extrema(i, ptr, data, true, offset, diffthr, &incr)) {
            first_max = data[idx_wraparound(ptr+i+offset+incr)];
            first_max_idx = i+offset+incr;
            break;
        }
    }

    if (first_max_idx == 0) {
        Serial.println("first_max not found");
        return false;
    }

    // rise time in samples
    uint16_t rise_tdiff = last_max_idx - mid_min_idx;
    // fall time in samples
    uint16_t fall_tdiff = mid_min_idx - first_max_idx;

    if (rise_tdiff < offset || fall_tdiff < offset) {
        Serial.println("rise tdiff/fall tdiff too small");
        return false;
    }

    // difference in voltage when rising
    int16_t rise_diff = last_max - mid_min;
    // difference in voltage when falling
    int16_t fall_diff = first_max - mid_min;

    if (rise_diff <= 0 || fall_diff <= 0 || rise_diff <= diffthr || fall_diff < diffthr) {
        Serial.println("rise/fall diff too small");
        return false;
    }

    lookback_rising = rise_tdiff;
    lookback_falling = fall_tdiff;
    det_rising_threshold = (int16_t)(rise_diff*0.6);
    det_falling_threshold = (int16_t)(-fall_diff*0.6);

    Serial.print("lookback_rising: ");
    Serial.println(lookback_rising);
    Serial.print("lookback_falling: ");
    Serial.println(lookback_falling);
    Serial.print("det_rising_threshold: ");
    Serial.println(det_rising_threshold);
    Serial.print("det_falling_threshold: ");
    Serial.println(det_falling_threshold);

    return true;
}

void detection() {
    digitalWrite(PIN_DETECT, LOW);
    // TODO: timer with interrupt to clear the pin would be clever, here
    // not so critical with this low sample rate tho... longer pulse needed?
    delayMicroseconds(500);
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
