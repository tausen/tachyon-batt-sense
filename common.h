#include <stdint.h>

// Define this to increase history length and log entire history to UART after a detection
// and return to idle
// #define DEBUG

// Maximum history. Must be high enough to capture samples from initial detection until
// end of burst for learning mode to work.
#ifdef DEBUG
#define NHIST 400
#else
#define NHIST 150
#endif

typedef struct {
    // Next detection when voltage has slowly risen by det_rising_threshold, then fallen by
    // det_falling_threshold
    int16_t det_rising_threshold;
    int16_t det_falling_threshold;
    // Number of samples we look back when detecting slow rise/fall.  Warning: Will read
    // LARGEST_LOOKBACK samples after the initial downward spike before starting to scan
    // for rising slope, so must not be too high. EVO measurements suggest first "real"
    // rising slope is about 40 samples after the spike.
    int16_t lookback_rising;
    int16_t lookback_falling;
} profile_t;

// Handle delay line pointer wraparound, assuming idx is never more than NHIST off
int16_t idx_wraparound(int16_t idx);
