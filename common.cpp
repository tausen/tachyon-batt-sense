#include "common.h"

int16_t idx_wraparound(int16_t idx) {
    if (idx < 0) {
        return idx + NHIST;
    } else if (idx >= NHIST) {
        return idx - NHIST;
    } else {
        return idx;
    }
}
