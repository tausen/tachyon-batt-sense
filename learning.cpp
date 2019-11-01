#include <HardwareSerial.h>
#include <stdint.h>
#include "common.h"

// Attempt to learn a sensible profile from the n-sample burst held in data.
// The data is expected to look something like this:
//
//                ,------------ first maximum
//                |   ,-------- mid minimum
//                |   |  ,----- last maximum
//                |   |  |   ,- last minimum
//                v   v  v   v
// ___,                         ______
//    |                       ,'
//    |           ,      ,    |
//    |          / ',   / ',  |
//    |         /    ','    ','
//    |   ,----'
//    |  /       |-------------------|
//    | /          '-> learn() uses these samples
//    |/
//    ' <- large initial drop used as trigger in loop()
//
//     |-----------------------------|
//      data[] contains these samples

// The procedure below reads data[] from the end and backwards, attempting to find the
// "last" minimum, then the "last" maximum, then the "mid" minimum and finally the "first"
// maximum. No assumptions are made on the remainder of the data (from T0 to first max),
// so as long as there is a sensible minimum to be found in the end of data[] followed by
// two peaks with a drop in between, the procedure should be OK.
//
// The time difference between extrema is assumed to be at least <offset> samples and the
// voltage difference between <offset> samples after a minimum to a maximum is assumed to
// be at least <diffthr>. Otherwise, no assumptions are made on the shape of the waveform.
//
// The fall time from first max to mid min and rise time from mid min to last max is used
// as best guesses for the new profile - rise/fall times are used as-is and the voltage
// difference threshold is set to 0.6 times the difference (emperical value).

static bool is_extrema(int i, int16_t ptr, int16_t *data, bool maxnmin, int16_t offset,
                       int16_t diffthr, int16_t *incr);

bool learn(int16_t ptr, int16_t *data, uint16_t n, profile_t *profile) {
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

    profile->lookback_rising = rise_tdiff;
    profile->lookback_falling = fall_tdiff;
    profile->det_rising_threshold = (int16_t)(rise_diff*0.6);
    profile->det_falling_threshold = (int16_t)(-fall_diff*0.6);

    Serial.print("lookback_rising: ");
    Serial.println(profile->lookback_rising);
    Serial.print("lookback_falling: ");
    Serial.println(profile->lookback_falling);
    Serial.print("det_rising_threshold: ");
    Serial.println(profile->det_rising_threshold);
    Serial.print("det_falling_threshold: ");
    Serial.println(profile->det_falling_threshold);

    return true;
}

// Check whether data[ptr+i+offset] is considered a local maximum (maxnmin=true) or minimum
// (maxnmin=false). This is a bit of a hack and is heavily based on test measurements.
// diffthr: smallest absolute difference between data[ptr+i+offset] and data[ptr+i] to
//          consider this an extrema
// incr: best-guess extrema location is at data[ptr+i+offset+incr]
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
