#include <stdint.h>
#include "extern.h"

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

// Persist configuration locations
#define EEPROM_ADDR_CRC 0
#define EEPROM_ADDR_RT (EEPROM_ADDR_CRC+sizeof(unsigned long))
#define EEPROM_ADDR_FT (EEPROM_ADDR_RT+sizeof(det_rising_threshold))
#define EEPROM_ADDR_LR (EEPROM_ADDR_FT+sizeof(det_falling_threshold))
#define EEPROM_ADDR_LF (EEPROM_ADDR_LR+sizeof(lookback_rising))
#define EEPROM_ADDR_END (EEPROM_ADDR_LF+sizeof(lookback_falling))

// Get largest of the two lookbacks
#define LARGEST_LOOKBACK (lookback_rising > lookback_falling ? lookback_rising : lookback_falling)

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
