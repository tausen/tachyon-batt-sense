#include <HardwareSerial.h>
#include <EEPROM.h>
#include <stdint.h>
#include "defs.h"
#include "crc.h"
#include "extern.h"

static bool is_extrema(int i, int16_t ptr, int16_t *data, bool maxnmin, int16_t offset,
                       int16_t diffthr, int16_t *incr);

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

    // Save configuration to EEPROM
    EEPROM.put(EEPROM_ADDR_RT, det_rising_threshold);
    EEPROM.put(EEPROM_ADDR_FT, det_falling_threshold);
    EEPROM.put(EEPROM_ADDR_LR, lookback_rising);
    EEPROM.put(EEPROM_ADDR_LF, lookback_falling);
    unsigned long crc = eeprom_crc();
    EEPROM.put(EEPROM_ADDR_CRC, crc);

    return true;
}

// Check whether data[ptr+i] is considered a local maximum (maxnmin=true) or minimum
// (maxnmin=false).
static bool is_extrema(int i, int16_t ptr, int16_t *data, bool maxnmin, int16_t offset,
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
